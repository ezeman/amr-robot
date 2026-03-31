import argparse
import json
import os
import queue
import re
import signal
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_slug(value: str) -> bool:
    allowed = set('abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_')
    return bool(value) and all(ch in allowed for ch in value)


@dataclass
class ManagedProc:
    name: str
    cmd: list[str]
    process: subprocess.Popen
    started_at: float
    log_path: Path


class ApiApp:
    def __init__(self, workspace: Path, host: str, port: int, api_key: str):
        self.workspace = workspace
        self.host = host
        self.port = port
        self.api_key = api_key
        self.agv_sh = self.workspace / 'scripts' / 'agv.sh'
        self.save_map_sh = self.workspace / 'scripts' / 'save_fixed_map.sh'
        self.kill_ros_sh = self.workspace / 'scripts' / 'kill_ros.sh'
        self.logs_dir = self.workspace / 'log' / 'api'
        self.logs_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        self._procs: Dict[str, ManagedProc] = {}
        self._event_lock = threading.Lock()
        self._event_subscribers: list[queue.Queue] = []
        self._event_seq = 0
        self._mission_components = {
            'bring_up': False,
            'slam': False,
            'localization': False,
            'navigation': False,
            'waypoint_record': False,
            'route_follow': False,
        }
        self._ros_detect_cache: dict[str, Any] = {}
        self._ros_detect_time: float = 0.0
        self._node_health_cache: dict[str, Any] = {}
        self._node_health_time: float = 0.0
        self._battery_cache: dict[str, Any] = {}
        self._battery_time: float = 0.0
        self._estop_cache: dict[str, Any] = {}
        self._estop_time: float = 0.0
        self._route_progress_thread: Optional[threading.Thread] = None
        self._route_progress_stop = threading.Event()
        self._route_feedback_re = re.compile(
            r'remaining_poses=(?P<remaining>-?\d+)\s+'
            r'distance_remaining=(?P<distance>[0-9.+-]+)\s+m\s+'
            r'recoveries=(?P<recoveries>-?\d+)'
        )
        # Background health poller — keeps caches warm without blocking SSE.
        self._health_stop = threading.Event()
        self._health_thread = threading.Thread(
            target=self._health_poll_loop, daemon=True, name='health-poll'
        )
        self._health_thread.start()
        self._estop_stop = threading.Event()
        self._estop_thread = threading.Thread(
            target=self._estop_watch_loop, daemon=True, name='estop-watch'
        )
        self._estop_thread.start()

    def _next_event_id_locked(self) -> int:
        self._event_seq += 1
        return self._event_seq

    def _publish_event(self, event_type: str, payload: dict[str, Any]) -> None:
        with self._event_lock:
            event = {
                'id': self._next_event_id_locked(),
                'type': event_type,
                'time_utc': _utc_now(),
                'payload': payload,
            }
            dead_queues: list[queue.Queue] = []
            for q in self._event_subscribers:
                try:
                    q.put_nowait(event)
                except queue.Full:
                    pass
                except Exception:
                    dead_queues.append(q)
            for q in dead_queues:
                try:
                    self._event_subscribers.remove(q)
                except ValueError:
                    pass

    # --- External ROS component detection (for systemd-launched nodes) ---
    _COMPONENT_NODES = {
        'slam': '/slam_toolbox',
        'localization': '/amcl',
        'navigation': '/bt_navigator',
    }

    def _health_poll_loop(self) -> None:
        """Background thread: refresh health caches every ~8 s."""
        while not self._health_stop.wait(8.0):
            try:
                self._detect_ros_components()
            except Exception:
                pass
            try:
                self.api_node_health()
            except Exception:
                pass
            try:
                self.api_battery_status()
            except Exception:
                pass
            try:
                self.api_estop_status()
            except Exception:
                pass

    def _estop_watch_loop(self) -> None:
        """Monitor E-Stop edges with a persistent pull-up request."""
        prev_engaged: Optional[bool] = None
        while not self._estop_stop.is_set():
            try:
                data = self._read_estop_once()
                if data.get('engaged') is not None:
                    self._estop_cache = data
                    self._estop_time = time.monotonic()
                    prev_engaged = data.get('engaged')

                proc = subprocess.Popen(
                    [
                        'gpiomon', '-b', '-B', 'pull-up', '-F', '%e',
                        self._ESTOP_GPIOCHIP, str(self._ESTOP_LINE),
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                )
                try:
                    while not self._estop_stop.is_set():
                        line = proc.stdout.readline() if proc.stdout else ''
                        if not line:
                            break
                        event = line.strip()
                        if event not in ('0', '1'):
                            continue
                        gpio_value = 0 if event == '0' else 1
                        engaged = (gpio_value == self._ESTOP_ACTIVE_VALUE)
                        data = {
                            'engaged': engaged,
                            'gpio_value': gpio_value,
                            'gpio_chip': self._ESTOP_GPIOCHIP,
                            'gpio_line': self._ESTOP_LINE,
                            'active_value': self._ESTOP_ACTIVE_VALUE,
                        }
                        self._estop_cache = data
                        self._estop_time = time.monotonic()
                        if prev_engaged is None or engaged != prev_engaged:
                            prev_engaged = engaged
                            self._publish_event('estop_changed', {'estop': data})
                            if engaged:
                                threading.Thread(
                                    target=self._handle_estop_engaged,
                                    daemon=True,
                                    name='estop-trip-stop',
                                ).start()
                finally:
                    if proc.poll() is None:
                        proc.terminate()
                        try:
                            proc.wait(timeout=1.0)
                        except Exception:
                            proc.kill()
            except Exception:
                pass
            self._estop_stop.wait(1.0)

    def _cached_node_health(self) -> dict[str, Any]:
        """Return cached health (never blocks on subprocess)."""
        return self._node_health_cache or {}

    def _cached_battery(self) -> dict[str, Any]:
        """Return cached battery (never blocks on subprocess)."""
        return self._battery_cache or {
            'available': False, 'voltage': None, 'percentage': None,
            'state': 'offline', 'low_threshold': 0.20, 'critical_threshold': 0.10,
        }

    def _cached_estop(self) -> dict[str, Any]:
        """Return cached E-Stop state (never blocks on subprocess)."""
        return self._estop_cache or {'engaged': None, 'gpio_value': None}

    # --- E-Stop (Emergency Switch via GPIO) ---
    # Jetson Orin Nano: Pin 32 (BOARD) = GPIO07 = PG.06 = gpiochip0 line 41
    _ESTOP_GPIOCHIP = os.environ.get('AGV_ESTOP_GPIOCHIP', 'gpiochip0')
    _ESTOP_LINE = int(os.environ.get('AGV_ESTOP_LINE', '41'))
    # This E-Stop closes GND to pin 32 when pressed, so pressed = GPIO low.
    _ESTOP_ACTIVE_VALUE = int(os.environ.get('AGV_ESTOP_ACTIVE_VALUE', '0'))

    def _read_estop_once(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            'engaged': None,
            'gpio_value': None,
            'gpio_chip': self._ESTOP_GPIOCHIP,
            'gpio_line': self._ESTOP_LINE,
            'active_value': self._ESTOP_ACTIVE_VALUE,
        }
        try:
            result = subprocess.run(
                ['gpioget', '-B', 'pull-up', self._ESTOP_GPIOCHIP, str(self._ESTOP_LINE)],
                capture_output=True, text=True, timeout=2.0,
            )
            val = int(result.stdout.strip())
            out['gpio_value'] = val
            out['engaged'] = (val == self._ESTOP_ACTIVE_VALUE)
        except Exception:
            pass
        return out

    def api_estop_status(self, force_refresh: bool = False) -> dict[str, Any]:
        """Read E-Stop switch state from GPIO."""
        now = time.monotonic()
        if not force_refresh and self._estop_cache:
            return self._estop_cache

        out = self._read_estop_once()
        self._estop_cache = out
        self._estop_time = now
        return out

    def _is_estop_engaged(self) -> bool:
        return bool(self.api_estop_status().get('engaged'))

    def _reject_if_estop_engaged(self) -> Optional[tuple[bool, dict[str, Any]]]:
        estop = self.api_estop_status()
        if estop.get('engaged'):
            return False, {
                'error': 'E-Stop is engaged. Release E-Stop before starting a mission.',
                'estop': estop,
            }
        return None

    def _handle_estop_engaged(self) -> None:
        stopped: list[str] = []
        for name in ('route_follow', 'waypoint_record', 'navigation', 'localization', 'slam', 'bring_up'):
            try:
                ok, _ = self._stop_component(name)
                if ok:
                    stopped.append(name)
            except Exception:
                pass
        self._publish_event('estop_trip', {
            'estop': self._cached_estop(),
            'stopped': stopped,
        })

    def _cached_mission_state(self) -> dict[str, bool]:
        """Return mission state using cached external detect (never blocks)."""
        state = dict(self._mission_components)
        with self._lock:
            managed = set(self._procs.keys())
        for comp, active in self._ros_detect_cache.items():
            if comp not in managed:
                state[comp] = active
        return state

    def _detect_ros_components(self) -> dict[str, bool]:
        """Check which mission components are running externally (not via _procs)."""
        now = time.monotonic()
        if now - self._ros_detect_time < 8.0 and self._ros_detect_cache:
            return self._ros_detect_cache
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=3.0,
            )
            nodes = set(result.stdout.strip().splitlines())
        except Exception:
            nodes = set()
        detected = {}
        for comp, node_name in self._COMPONENT_NODES.items():
            detected[comp] = node_name in nodes
        self._ros_detect_cache = detected
        self._ros_detect_time = now
        return detected

    def mission_state(self) -> dict[str, bool]:
        """Return merged mission state (API-managed + externally detected)."""
        state = dict(self._mission_components)
        # For components not managed by _procs, check ROS nodes
        with self._lock:
            managed = set(self._procs.keys())
        external = self._detect_ros_components()
        for comp, active in external.items():
            if comp not in managed:
                state[comp] = active
        return state

    # --- Hardware node health (cached) ---
    _HEALTH_NODES = {
        'lidar': {'node': '/ldlidar', 'topic': '/scan_raw'},
        'imu': {'node': '/imu_driver', 'topic': '/imu/data'},
        'encoder': {'node': '/base_controller', 'topic': '/odom'},
        'camera': {'node': '/oak', 'topic': '/oak/rgb/image_raw'},
        'battery': {'node': None, 'topic': '/battery_state'},
    }

    def api_node_health(self) -> dict[str, Any]:
        """Return live/stale status of hardware nodes and their topics."""
        now = time.monotonic()
        if now - self._node_health_time < 8.0 and self._node_health_cache:
            return self._node_health_cache
        # Single subprocess: get nodes
        try:
            r = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=3.0)
            nodes = set(r.stdout.strip().splitlines())
        except Exception:
            nodes = set()
        # Single subprocess: get topics with publisher counts
        topic_publishers: dict[str, int] = {}
        try:
            r = subprocess.run(
                ['ros2', 'topic', 'list', '-v'],
                capture_output=True, text=True, timeout=3.0,
            )
            # Parse output: lines like " * /scan_raw [sensor_msgs/msg/LaserScan] 1 publisher"
            in_published = False
            for line in r.stdout.splitlines():
                stripped = line.strip()
                if stripped.startswith('Published topics:'):
                    in_published = True
                    continue
                if stripped.startswith('Subscribed topics:'):
                    in_published = False
                    continue
                if not in_published or not stripped.startswith('*'):
                    continue
                parts = stripped.split()
                # parts: ['*', '/topic', '[type]', 'N', 'publisher(s)']
                if len(parts) >= 4 and parts[1].startswith('/'):
                    topic_name = parts[1]
                    for i, p in enumerate(parts):
                        if 'publisher' in p and i > 0:
                            try:
                                topic_publishers[topic_name] = int(parts[i - 1])
                            except ValueError:
                                pass
                            break
        except Exception:
            pass

        result = {}
        for name, cfg in self._HEALTH_NODES.items():
            node_ok = cfg['node'] in nodes if cfg['node'] else None
            topic_ok = topic_publishers.get(cfg['topic'], 0) > 0 if cfg['topic'] else None
            if node_ok is None:
                status = 'active' if topic_ok else 'offline'
            elif node_ok and topic_ok:
                status = 'active'
            elif node_ok:
                status = 'warn'
            elif topic_ok:
                status = 'warn'
            else:
                status = 'offline'
            result[name] = {
                'status': status,
                'node': cfg['node'],
                'node_alive': node_ok,
                'topic': cfg['topic'],
                'topic_alive': topic_ok,
            }
        self._node_health_cache = result
        self._node_health_time = now
        return result

    _BATTERY_STATE_FILE = os.path.join(tempfile.gettempdir(), 'battery_state.json')

    def api_battery_status(self) -> dict[str, Any]:
        """Return latest /battery_state values with low/critical classification."""
        now = time.monotonic()
        if now - self._battery_time < 8.0 and self._battery_cache:
            return self._battery_cache

        out: dict[str, Any] = {
            'available': False,
            'voltage': None,
            'percentage': None,
            'state': 'offline',
            'low_threshold': 0.20,
            'critical_threshold': 0.10,
        }
        try:
            with open(self._BATTERY_STATE_FILE, 'r') as f:
                data = json.load(f)
            # File is stale if older than 10 seconds.
            age = time.time() - os.path.getmtime(self._BATTERY_STATE_FILE)
            if age > 10.0:
                raise ValueError('stale battery file')
            voltage = data.get('voltage')
            percentage = data.get('percentage')

            out['voltage'] = voltage
            out['percentage'] = percentage
            out['available'] = voltage is not None or percentage is not None

            if percentage is None:
                out['state'] = 'unknown' if out['available'] else 'offline'
            elif percentage <= out['critical_threshold']:
                out['state'] = 'critical'
            elif percentage <= out['low_threshold']:
                out['state'] = 'low'
            else:
                out['state'] = 'normal'
        except Exception:
            pass

        self._battery_cache = out
        self._battery_time = now
        return out

    def _ros2_param_get(self, node_name: str, param_name: str, timeout_sec: float = 4.0) -> Optional[Any]:
        try:
            result = subprocess.run(
                [str(self.agv_sh), 'ros2', 'param', 'get', node_name, param_name],
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
            if result.returncode != 0:
                return None
            text = (result.stdout or '').strip()
            if ': ' not in text:
                return None
            raw = text.split(': ', 1)[1].strip()
            if raw in ('true', 'false'):
                return raw == 'true'
            if raw.startswith("'") and raw.endswith("'"):
                return raw[1:-1]
            try:
                if any(ch in raw for ch in ('.', 'e', 'E')):
                    return float(raw)
                return int(raw)
            except Exception:
                return raw
        except Exception:
            return None

    def _ros2_param_set(self, node_name: str, param_name: str, value: Any, timeout_sec: float = 5.0) -> tuple[bool, str]:
        try:
            result = subprocess.run(
                [str(self.agv_sh), 'ros2', 'param', 'set', node_name, param_name, str(value)],
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
            ok = result.returncode == 0
            msg = (result.stdout or result.stderr or '').strip()
            return ok, msg
        except Exception as exc:
            return False, str(exc)

    def api_battery_params(self) -> dict[str, Any]:
        params = {
            'voltage_scale': self._ros2_param_get('/battery_node', 'voltage_scale'),
            'battery_voltage_min': self._ros2_param_get('/battery_node', 'battery_voltage_min'),
            'battery_voltage_max': self._ros2_param_get('/battery_node', 'battery_voltage_max'),
            'percentage_mode': self._ros2_param_get('/battery_node', 'percentage_mode'),
        }
        return {
            'available': all(v is not None for v in params.values()),
            'params': params,
        }

    def api_battery_set_params(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        allowed = ('voltage_scale', 'battery_voltage_min', 'battery_voltage_max', 'percentage_mode')
        updates = {k: body[k] for k in allowed if k in body}
        if not updates:
            return False, {'error': f'No params provided. Allowed: {allowed}'}

        if 'percentage_mode' in updates:
            mode = str(updates['percentage_mode']).strip()
            if mode not in ('lifepo4_8s', 'linear'):
                return False, {'error': 'percentage_mode must be one of: lifepo4_8s, linear'}
            updates['percentage_mode'] = mode

        if 'voltage_scale' in updates:
            updates['voltage_scale'] = float(updates['voltage_scale'])
        if 'battery_voltage_min' in updates:
            updates['battery_voltage_min'] = float(updates['battery_voltage_min'])
        if 'battery_voltage_max' in updates:
            updates['battery_voltage_max'] = float(updates['battery_voltage_max'])

        applied: dict[str, Any] = {}
        errors: dict[str, str] = {}
        for key, value in updates.items():
            ok, msg = self._ros2_param_set('/battery_node', key, value)
            if ok:
                applied[key] = value
            else:
                errors[key] = msg

        return len(errors) == 0, {
            'applied': applied,
            'errors': errors,
            'current': self.api_battery_params(),
        }

    def _emit_mission_state(self, component: str, active: bool, reason: str) -> None:
        if component in self._mission_components:
            self._mission_components[component] = bool(active)
        merged = self._cached_mission_state()
        active_components = [k for k, v in merged.items() if v]
        self._publish_event(
            'mission_state_changed',
            {
                'component': component,
                'active': bool(active),
                'reason': reason,
                'active_components': active_components,
                'mission_state': merged,
            },
        )

    def _stop_route_progress_monitor(self) -> None:
        self._route_progress_stop.set()
        t = self._route_progress_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._route_progress_thread = None
        self._route_progress_stop.clear()
        self._publish_event('route_progress', {'status': 'STOPPED'})

    def _start_route_progress_monitor(self, proc: ManagedProc) -> None:
        self._stop_route_progress_monitor()
        self._publish_event('route_progress', {'status': 'STARTED', 'log_path': str(proc.log_path)})

        def _runner() -> None:
            while not self._route_progress_stop.is_set() and not proc.log_path.exists():
                if proc.process.poll() is not None:
                    return
                time.sleep(0.2)

            if not proc.log_path.exists():
                return

            try:
                with open(proc.log_path, 'r', encoding='utf-8', errors='ignore') as f:
                    f.seek(0, os.SEEK_END)
                    while not self._route_progress_stop.is_set():
                        line = f.readline()
                        if not line:
                            if proc.process.poll() is not None:
                                return
                            time.sleep(0.2)
                            continue

                        m = self._route_feedback_re.search(line)
                        if m:
                            self._publish_event(
                                'route_progress',
                                {
                                    'remaining_poses': int(m.group('remaining')),
                                    'distance_remaining_m': float(m.group('distance')),
                                    'recoveries': int(m.group('recoveries')),
                                    'log_path': str(proc.log_path),
                                },
                            )
                            continue

                        if 'Route finished successfully.' in line:
                            self._publish_event(
                                'route_progress',
                                {
                                    'status': 'SUCCEEDED',
                                    'log_path': str(proc.log_path),
                                },
                            )
                        elif 'Route finished with status' in line:
                            self._publish_event(
                                'route_progress',
                                {
                                    'status': 'FINISHED_WITH_NON_SUCCESS',
                                    'detail': line.strip(),
                                    'log_path': str(proc.log_path),
                                },
                            )
            except Exception as exc:
                self._publish_event('route_progress', {'status': 'MONITOR_ERROR', 'error': str(exc)})

        self._route_progress_thread = threading.Thread(target=_runner, daemon=True)
        self._route_progress_thread.start()

    def subscribe_events(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=64)
        with self._event_lock:
            self._event_subscribers.append(q)
        return q

    def unsubscribe_events(self, q: queue.Queue) -> None:
        with self._event_lock:
            try:
                self._event_subscribers.remove(q)
            except ValueError:
                pass

    def _run_sync(self, cmd: list[str], timeout_sec: float = 120.0) -> dict[str, Any]:
        completed = subprocess.run(
            cmd,
            cwd=str(self.workspace),
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
        return {
            'exit_code': completed.returncode,
            'stdout': completed.stdout,
            'stderr': completed.stderr,
        }

    def _log_file_for(self, name: str) -> Path:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return self.logs_dir / f'{name}_{stamp}.log'

    def _cleanup_dead_locked(self) -> None:
        dead = []
        for name, proc in self._procs.items():
            rc = proc.process.poll()
            if rc is not None:
                dead.append(name)
        for name in dead:
            proc = self._procs.get(name)
            rc = proc.process.poll() if proc else None
            self._procs.pop(name, None)
            self._publish_event('process_exited', {'name': name, 'exit_code': rc})
            self._emit_mission_state(name, False, 'process_exited')
            if name == 'route_follow':
                self._stop_route_progress_monitor()

    def _status_locked(self) -> dict[str, Any]:
        self._cleanup_dead_locked()
        result: dict[str, Any] = {}
        for name, proc in self._procs.items():
            result[name] = {
                'running': proc.process.poll() is None,
                'pid': proc.process.pid,
                'started_at_epoch': proc.started_at,
                'started_at_utc': datetime.fromtimestamp(proc.started_at, tz=timezone.utc).isoformat(),
                'command': proc.cmd,
                'log_path': str(proc.log_path),
            }
        return result

    def status(self) -> dict[str, Any]:
        with self._lock:
            return self._status_locked()

    # Processes that launch hardware nodes — only one may run at a time.
    _HARDWARE_PROCS = frozenset({'bring_up', 'slam', 'localization', 'navigation'})

    def _stop_systemd_mapping(self) -> None:
        """Stop agv-mapping.service if it is active (avoids duplicate hardware)."""
        try:
            result = subprocess.run(
                ['systemctl', '--user', 'is-active', '--quiet', 'agv-mapping.service'],
                timeout=3,
            )
            if result.returncode == 0:
                subprocess.run(
                    ['systemctl', '--user', 'stop', 'agv-mapping.service'],
                    timeout=10,
                )
        except Exception:
            pass

    def _stop_conflicting_procs(self, name: str) -> list[str]:
        """Auto-stop any running hardware proc that conflicts with *name*."""
        stopped: list[str] = []
        if name not in self._HARDWARE_PROCS:
            return stopped
        for other in list(self._HARDWARE_PROCS - {name}):
            with self._lock:
                proc = self._procs.get(other)
                alive = proc is not None and proc.process.poll() is None
            if alive:
                self._stop_proc(other)
                stopped.append(other)
        return stopped

    def _start_proc(self, name: str, cmd: list[str]) -> tuple[bool, dict[str, Any]]:
        # --- mutual exclusion: stop conflicting hardware procs + systemd ---
        if name in self._HARDWARE_PROCS:
            # Stop systemd mapping in background (non-blocking)
            threading.Thread(
                target=self._stop_systemd_mapping, daemon=True
            ).start()
            stopped = self._stop_conflicting_procs(name)
        else:
            stopped = []

        with self._lock:
            self._cleanup_dead_locked()
            if name in self._procs and self._procs[name].process.poll() is None:
                return False, {'error': f'Process {name} is already running.'}

            log_path = self._log_file_for(name)
            logf = open(log_path, 'a', encoding='utf-8')
            env = os.environ.copy()
            # Skip preflight for processes that start hardware themselves
            if name in self._HARDWARE_PROCS:
                env['AGV_PREFLIGHT_TOPICS'] = '0'
            process = subprocess.Popen(
                cmd,
                cwd=str(self.workspace),
                stdout=logf,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
                env=env,
            )
            self._procs[name] = ManagedProc(
                name=name,
                cmd=cmd,
                process=process,
                started_at=time.time(),
                log_path=log_path,
            )
            payload = {
                'name': name,
                'pid': process.pid,
                'log_path': str(log_path),
                'command': cmd,
            }
            if stopped:
                payload['auto_stopped'] = stopped
            self._publish_event('process_started', payload)
            self._emit_mission_state(name, True, 'process_started')
            if name == 'route_follow':
                self._start_route_progress_monitor(self._procs[name])
            return True, payload

    def _stop_proc(self, name: str, timeout_sec: float = 10.0) -> tuple[bool, dict[str, Any]]:
        with self._lock:
            self._cleanup_dead_locked()
            proc = self._procs.get(name)
            if proc is None or proc.process.poll() is not None:
                self._procs.pop(name, None)
                return False, {'error': f'Process {name} is not running.'}

        pgid = os.getpgid(proc.process.pid)
        os.killpg(pgid, signal.SIGINT)

        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if proc.process.poll() is not None:
                break
            time.sleep(0.2)

        if proc.process.poll() is None:
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(1.0)

        with self._lock:
            rc = proc.process.poll()
            if rc is None:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                time.sleep(0.2)
                rc = proc.process.poll()
            self._procs.pop(name, None)

        payload = {'name': name, 'exit_code': rc}
        self._publish_event('process_stopped', payload)
        self._emit_mission_state(name, False, 'process_stopped')
        if name == 'route_follow':
            self._stop_route_progress_monitor()
        return True, payload

    def _stop_component(self, name: str) -> tuple[bool, dict[str, Any]]:
        """Stop component: API-managed first, then external (systemd)."""
        ok, data = self._stop_proc(name)
        if ok:
            return True, data
        # Not API-managed — check if running externally via systemd
        external = self._detect_ros_components()
        if external.get(name, False):
            self._stop_systemd_mapping()
            self._ros_detect_time = 0.0  # invalidate cache
            self._emit_mission_state(name, False, 'external_stop')
            return True, {'name': name, 'stopped_via': 'systemd'}
        return False, data

    def _map_yaml_path(self, value: str) -> Path:
        p = Path(value).expanduser()
        if p.is_absolute() and p.exists():
            return p

        if p.suffix != '.yaml':
            p = p.with_suffix('.yaml')

        candidate = self.workspace / 'maps' / p
        if candidate.exists():
            return candidate

        alt = self.workspace / 'maps' / p.stem / p.name
        if alt.exists():
            return alt

        raise FileNotFoundError(f'Map YAML not found for value: {value}')

    def _route_yaml_path(self, value: str) -> Path:
        p = Path(value).expanduser()
        if p.is_absolute() and p.exists():
            return p

        if p.suffix != '.yaml':
            p = p.with_suffix('.yaml')

        candidate = self.workspace / 'routes' / p
        if candidate.exists():
            return candidate

        raise FileNotFoundError(f'Route YAML not found for value: {value}')

    def _to_ros_args(self, values: dict[str, Any]) -> list[str]:
        result: list[str] = []
        for key, value in values.items():
            if value is None:
                continue
            result.append(f'{key}:={value}')
        return result

    def api_health(self) -> dict[str, Any]:
        return {
            'ok': True,
            'time_utc': _utc_now(),
            'workspace': str(self.workspace),
            'status': self.status(),
        }

    def api_list_maps(self) -> dict[str, Any]:
        maps_root = self.workspace / 'maps'
        items = []
        if maps_root.exists():
            for yaml_path in sorted(maps_root.rglob('*.yaml')):
                fixed = yaml_path.parent / 'FIXED_MAP.txt'
                items.append({
                    'name': yaml_path.stem,
                    'yaml': str(yaml_path),
                    'fixed': fixed.exists(),
                })
        return {'items': items}

    def api_list_routes(self) -> dict[str, Any]:
        routes_root = self.workspace / 'routes'
        items = []
        if routes_root.exists():
            for yaml_path in sorted(routes_root.glob('*.yaml')):
                items.append({'name': yaml_path.stem, 'yaml': str(yaml_path)})
        return {'items': items}

    def _hw_launch_args(self, body: dict[str, Any]) -> dict[str, Any]:
        """Common launch arguments shared by bring_up and slam."""
        return {
            'lidar_product': body.get('lidar_product', 'LDLiDAR_STL27L'),
            'lidar_baud': body.get('lidar_baud', 921600),
            'lidar_port': body.get('lidar_port', '/dev/lidar'),
            'use_topic_health_fail_hard': 'true' if body.get('use_topic_health_fail_hard', False) else 'false',
            'topic_health_fail_grace_sec': body.get('topic_health_fail_grace_sec', 8.0),
            'base_angular_sign': body.get('base_angular_sign', 1.0),
        }

    def api_bringup_start(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        blocked = self._reject_if_estop_engaged()
        if blocked is not None:
            return blocked
        args = self._hw_launch_args(body)
        cmd = [str(self.agv_sh), 'ros2', 'launch', 'amr_bringup', 'agv_bringup.launch.py'] + self._to_ros_args(args)
        return self._start_proc('bring_up', cmd)

    def api_bringup_stop(self) -> tuple[bool, dict[str, Any]]:
        return self._stop_component('bring_up')

    def api_slam_start(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        blocked = self._reject_if_estop_engaged()
        if blocked is not None:
            return blocked
        args = self._hw_launch_args(body)
        cmd = [str(self.agv_sh), 'ros2', 'launch', 'amr_bringup', 'agv_mapping.launch.py'] + self._to_ros_args(args)
        return self._start_proc('slam', cmd)

    def api_slam_stop(self) -> tuple[bool, dict[str, Any]]:
        return self._stop_component('slam')

    def api_slam_save_map(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        map_name = str(body.get('map_name', '')).strip()
        if not _safe_slug(map_name):
            return False, {'error': 'map_name is required and must match [A-Za-z0-9_-]'}

        cmd = [
            str(self.agv_sh),
            'bash',
            str(self.save_map_sh),
            map_name,
        ]
        result = self._run_sync(cmd, timeout_sec=float(body.get('timeout_sec', 180.0)))
        ok = result['exit_code'] == 0
        payload = {
            'map_name': map_name,
            'map_yaml': str(self.workspace / 'maps' / map_name / f'{map_name}.yaml'),
            'map_pgm': str(self.workspace / 'maps' / map_name / f'{map_name}.pgm'),
            **result,
        }
        if ok:
            self._publish_event('map_saved', {
                'map_name': map_name,
                'map_yaml': payload['map_yaml'],
                'map_pgm': payload['map_pgm'],
            })
        else:
            self._publish_event('map_save_failed', {
                'map_name': map_name,
                'exit_code': payload['exit_code'],
                'stderr': payload.get('stderr', ''),
            })
        return ok, payload

    def api_localization_start(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        blocked = self._reject_if_estop_engaged()
        if blocked is not None:
            return blocked
        map_yaml = self._map_yaml_path(str(body.get('map')))
        args = {
            'map': str(map_yaml),
            'use_topic_health_fail_hard': 'true' if body.get('use_topic_health_fail_hard', False) else 'false',
            'topic_health_fail_grace_sec': body.get('topic_health_fail_grace_sec', 8.0),
            'base_angular_sign': body.get('base_angular_sign', 1.0),
        }
        cmd = [str(self.agv_sh), 'ros2', 'launch', 'amr_bringup', 'agv_localization.launch.py'] + self._to_ros_args(args)
        return self._start_proc('localization', cmd)

    def api_localization_stop(self) -> tuple[bool, dict[str, Any]]:
        return self._stop_component('localization')

    def api_navigation_start(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        blocked = self._reject_if_estop_engaged()
        if blocked is not None:
            return blocked
        map_yaml = self._map_yaml_path(str(body.get('map')))
        args = {
            'map': str(map_yaml),
            'use_topic_health_fail_hard': 'true' if body.get('use_topic_health_fail_hard', False) else 'false',
            'topic_health_fail_grace_sec': body.get('topic_health_fail_grace_sec', 8.0),
            'base_angular_sign': body.get('base_angular_sign', 1.0),
            'use_yield_requester': 'true' if body.get('use_yield_requester', True) else 'false',
        }
        cmd = [str(self.agv_sh), 'ros2', 'launch', 'amr_bringup', 'agv_nav.launch.py'] + self._to_ros_args(args)
        return self._start_proc('navigation', cmd)

    def api_navigation_stop(self) -> tuple[bool, dict[str, Any]]:
        return self._stop_component('navigation')

    def api_waypoints_record_start(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        blocked = self._reject_if_estop_engaged()
        if blocked is not None:
            return blocked
        output_path = str(body.get('output_path', '')).strip()
        if not output_path:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_path = str(self.workspace / 'routes' / f'route_{stamp}_waypoints.yaml')

        cmd = [
            str(self.agv_sh),
            'ros2',
            'run',
            'amr_tools',
            'record_waypoints',
            '--ros-args',
            '-p',
            f'output_path:={output_path}',
        ]

        map_frame = body.get('map_frame')
        base_frame = body.get('base_frame')
        if map_frame:
            cmd.extend(['-p', f'map_frame:={map_frame}'])
        if base_frame:
            cmd.extend(['-p', f'base_frame:={base_frame}'])

        ok, payload = self._start_proc('waypoint_record', cmd)
        if ok:
            payload['output_path'] = output_path
        return ok, payload

    def api_waypoints_record_stop(self) -> tuple[bool, dict[str, Any]]:
        return self._stop_proc('waypoint_record')

    def api_route_start(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        blocked = self._reject_if_estop_engaged()
        if blocked is not None:
            return blocked
        route = self._route_yaml_path(str(body.get('route')))

        cmd = [
            str(self.agv_sh),
            'ros2',
            'run',
            'amr_tools',
            'route_loop_runner',
            '--route',
            str(route),
            '--yaw-mode',
            str(body.get('yaw_mode', 'heading')),
            '--start-mode',
            str(body.get('start_mode', 'nearest')),
            '--pause',
            str(body.get('pause_sec', 3.0)),
        ]

        if body.get('loop', False):
            cmd.append('--loop')
        if body.get('reverse', False):
            cmd.append('--reverse')
        if body.get('roundtrips') is not None:
            cmd.extend(['--roundtrips', str(body.get('roundtrips'))])
        if body.get('stop_on_failure', True):
            cmd.append('--stop-on-failure')
        else:
            cmd.append('--no-stop-on-failure')

        return self._start_proc('route_follow', cmd)

    def api_route_stop(self) -> tuple[bool, dict[str, Any]]:
        return self._stop_proc('route_follow')

    def api_validate_configs(self, body: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        cmd = [
            str(self.agv_sh),
            'ros2',
            'run',
            'amr_tools',
            'validate_configs',
            '--workspace',
            str(self.workspace),
        ]
        if bool(body.get('strict', False)):
            cmd.append('--strict')
        if body.get('strict_profile'):
            cmd.extend(['--strict-profile', str(body.get('strict_profile'))])

        result = self._run_sync(cmd, timeout_sec=float(body.get('timeout_sec', 90.0)))
        return result['exit_code'] == 0, result

    def api_system_kill_ros(self) -> tuple[bool, dict[str, Any]]:
        cmd = [str(self.agv_sh), 'bash', str(self.kill_ros_sh)]
        result = self._run_sync(cmd, timeout_sec=30.0)
        return result['exit_code'] == 0, result


class ApiHandler(BaseHTTPRequestHandler):
    server_version = 'AgvApiServer/1.0'

    def _app(self) -> ApiApp:
        return self.server.app  # type: ignore[attr-defined]

    def _send_json(self, status_code: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-API-Key')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.end_headers()
        self.wfile.write(body)

    def _json_body(self) -> dict[str, Any]:
        length = int(self.headers.get('Content-Length', '0'))
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if not raw:
            return {}
        return json.loads(raw.decode('utf-8'))

    def _send_sse_event(self, event: dict[str, Any]) -> None:
        event_id = event.get('id', 0)
        event_type = str(event.get('type', 'message'))
        data = json.dumps(event, ensure_ascii=False)
        chunk = f"id: {event_id}\nevent: {event_type}\ndata: {data}\n\n".encode('utf-8')
        self.wfile.write(chunk)
        self.wfile.flush()

    def _serve_events(self) -> None:
        app = self._app()
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-API-Key')
        self.end_headers()

        q = app.subscribe_events()
        try:
            self._send_sse_event({
                'id': 0,
                'type': 'snapshot',
                'time_utc': _utc_now(),
                'payload': {
                    'status': app.status(),
                    'mission_state': app._cached_mission_state(),
                    'node_health': app._cached_node_health(),
                    'battery': app._cached_battery(),
                    'estop': app._cached_estop(),
                },
            })

            while True:
                try:
                    event = q.get(timeout=2.0)
                except queue.Empty:
                    event = {
                        'id': 0,
                        'type': 'heartbeat',
                        'time_utc': _utc_now(),
                        'payload': {
                            'status': app.status(),
                            'mission_state': app._cached_mission_state(),
                            'node_health': app._cached_node_health(),
                            'battery': app._cached_battery(),
                            'estop': app._cached_estop(),
                        },
                    }
                self._send_sse_event(event)
        except (BrokenPipeError, ConnectionResetError):
            return
        finally:
            app.unsubscribe_events(q)

    def _authorized(self) -> bool:
        app = self._app()
        if not app.api_key:
            return True
        provided = self.headers.get('X-API-Key', '')
        if provided == app.api_key:
            return True
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        query_key = (query.get('api_key') or [''])[0]
        return query_key == app.api_key

    def _require_auth(self) -> bool:
        if self._authorized():
            return True
        self._send_json(401, {'ok': False, 'error': 'Unauthorized'})
        return False

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send_json(200, {'ok': True})

    def do_GET(self) -> None:  # noqa: N802
        if not self._require_auth():
            return

        app = self._app()
        parsed = urlparse(self.path)
        route = parsed.path
        try:
            if route == '/api/v1/events':
                self._serve_events()
                return
            if route == '/api/v1/health':
                self._send_json(200, {'ok': True, 'data': app.api_health()})
                return
            if route == '/api/v1/status':
                self._send_json(200, {'ok': True, 'data': app.status()})
                return
            if route == '/api/v1/node_health':
                self._send_json(200, {'ok': True, 'data': app.api_node_health()})
                return
            if route == '/api/v1/battery':
                self._send_json(200, {'ok': True, 'data': app.api_battery_status()})
                return
            if route == '/api/v1/estop':
                self._send_json(200, {'ok': True, 'data': app.api_estop_status()})
                return
            if route == '/api/v1/battery/params':
                self._send_json(200, {'ok': True, 'data': app.api_battery_params()})
                return
            if route == '/api/v1/maps':
                self._send_json(200, {'ok': True, 'data': app.api_list_maps()})
                return
            if route == '/api/v1/routes':
                self._send_json(200, {'ok': True, 'data': app.api_list_routes()})
                return
            self._send_json(404, {'ok': False, 'error': 'Not found'})
        except Exception as exc:
            self._send_json(500, {'ok': False, 'error': str(exc)})

    def do_POST(self) -> None:  # noqa: N802
        if not self._require_auth():
            return

        app = self._app()
        try:
            body = self._json_body()

            routes = {
                '/api/v1/bringup/start': app.api_bringup_start,
                '/api/v1/bringup/stop': lambda _b: app.api_bringup_stop(),
                '/api/v1/slam/start': app.api_slam_start,
                '/api/v1/slam/stop': lambda _b: app.api_slam_stop(),
                '/api/v1/slam/save_map': app.api_slam_save_map,
                '/api/v1/localization/start': app.api_localization_start,
                '/api/v1/localization/stop': lambda _b: app.api_localization_stop(),
                '/api/v1/navigation/start': app.api_navigation_start,
                '/api/v1/navigation/stop': lambda _b: app.api_navigation_stop(),
                '/api/v1/waypoints/record/start': app.api_waypoints_record_start,
                '/api/v1/waypoints/record/stop': lambda _b: app.api_waypoints_record_stop(),
                '/api/v1/route/start': app.api_route_start,
                '/api/v1/route/stop': lambda _b: app.api_route_stop(),
                '/api/v1/config/validate': app.api_validate_configs,
                '/api/v1/system/kill_ros': lambda _b: app.api_system_kill_ros(),
                '/api/v1/battery/params': app.api_battery_set_params,
            }

            fn = routes.get(self.path)
            if fn is None:
                self._send_json(404, {'ok': False, 'error': 'Not found'})
                return

            ok, data = fn(body)
            data['mission_state'] = app._cached_mission_state()
            self._send_json(200 if ok else 400, {'ok': ok, 'data': data})
        except json.JSONDecodeError:
            self._send_json(400, {'ok': False, 'error': 'Invalid JSON body'})
        except FileNotFoundError as exc:
            self._send_json(400, {'ok': False, 'error': str(exc)})
        except Exception as exc:
            self._send_json(500, {'ok': False, 'error': str(exc)})


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description='REST API server for AGV ROS2 control from external web UI.')
    parser.add_argument('--host', default='0.0.0.0', help='Bind host (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8088, help='Bind port (default: 8088)')
    parser.add_argument('--workspace', default=str(Path.home() / 'agv_ws'), help='AGV workspace path')
    parser.add_argument('--api-key', default=os.environ.get('AGV_API_KEY', ''), help='Optional API key (or AGV_API_KEY env)')
    args = parser.parse_args(argv)

    workspace = Path(args.workspace).expanduser().resolve()
    app = ApiApp(workspace=workspace, host=args.host, port=args.port, api_key=args.api_key)

    if not app.agv_sh.exists():
        raise FileNotFoundError(f'agv.sh not found: {app.agv_sh}')

    server = ThreadingHTTPServer((args.host, args.port), ApiHandler)
    server.app = app  # type: ignore[attr-defined]

    print(f'AGV API server listening on http://{args.host}:{args.port}')
    print(f'Workspace: {workspace}')
    print('Use /api/v1/health for readiness check.')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
