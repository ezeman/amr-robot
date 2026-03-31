const el = (id) => document.getElementById(id);

const state = {
  events: null,
  presetBusy: false,
  mission: {
    bring_up: false,
    slam: false,
    localization: false,
    navigation: false,
    waypoint_record: false,
    route_follow: false,
  },
  nodeHealth: {},
  battery: null,
  estop: null,
};

let sseReconnectDelay = 1000;
let sseReconnectTimer = null;

const estopLockedButtons = new Set([
  'btnBringUpStart',
  'btnSlamStart',
  'btnLocStart',
  'btnNavStart',
  'btnRecordStart',
  'btnRouteStart',
  'btnPresetBringUp',
  'btnPresetStartMapping',
  'btnPresetStartDelivery',
]);

function defaultApiBaseUrl() {
  const host = window.location.hostname || '127.0.0.1';
  return `http://${host}:8088`;
}

function baseUrl() {
  return el('baseUrl').value.trim().replace(/\/$/, '');
}

function apiKey() {
  return el('apiKey').value.trim();
}

function headers() {
  const h = { 'Content-Type': 'application/json' };
  const key = apiKey();
  if (key) {
    h['X-API-Key'] = key;
  }
  return h;
}

function appendLog(target, message, payload) {
  const box = el(target);
  const line = `[${new Date().toLocaleTimeString()}] ${message}`;
  box.textContent = `${line}\n${payload ? JSON.stringify(payload, null, 2) : ''}\n\n${box.textContent}`;
}

function toast(msg, type = 'info') {
  const c = el('toasts');
  if (!c) return;
  const d = document.createElement('div');
  d.className = `toast toast-${type}`;
  d.textContent = msg;
  c.appendChild(d);
  setTimeout(() => {
    d.classList.add('fade-out');
    setTimeout(() => d.remove(), 350);
  }, 3200);
}

function bindLogFoldButtons() {
  const bindOne = (btnId, logId) => {
    const btn = el(btnId);
    const log = el(logId);
    if (!btn || !log) return;
    btn.onclick = () => {
      const isHidden = log.hasAttribute('hidden');
      if (isHidden) {
        log.removeAttribute('hidden');
        btn.textContent = 'hide';
      } else {
        log.setAttribute('hidden', 'hidden');
        btn.textContent = 'view';
      }
    };
  };
  bindOne('btnToggleEventsLog', 'eventsLog');
  bindOne('btnToggleResponseLog', 'responseLog');
}

function setPresetStatus(text) {
  const node = el('presetStatus');
  if (!node) return;
  node.textContent = `Preset status: ${text}`;
}

async function apiGet(path) {
  const res = await fetch(`${baseUrl()}${path}`, {
    method: 'GET',
    headers: headers(),
  });
  const data = await res.json();
  appendLog('responseLog', `GET ${path} -> ${res.status}`, data);
  return data;
}

async function apiPost(path, body = {}) {
  const res = await fetch(`${baseUrl()}${path}`, {
    method: 'POST',
    headers: headers(),
    body: JSON.stringify(body),
  });
  const data = await res.json();
  appendLog('responseLog', `POST ${path} -> ${res.status}`, data);
  // Auto-update mission state from response
  const ms = data?.data?.mission_state;
  if (ms && typeof ms === 'object') {
    for (const key of Object.keys(state.mission)) {
      if (typeof ms[key] === 'boolean') state.mission[key] = ms[key];
    }
    refreshMissionUI();
  }
  return data;
}

async function apiPostChecked(path, body = {}) {
  const out = await apiPost(path, body);
  if (!out?.ok) {
    throw new Error(`API failed at ${path}`);
  }
  return out;
}

function refreshMissionUI() {
  for (const [name, active] of Object.entries(state.mission)) {
    const node = document.querySelector(`.state-item[data-name="${name}"]`);
    if (!node) continue;
    node.setAttribute('data-state', active ? 'running' : 'idle');
    node.classList.toggle('active', !!active);
  }
}

function refreshHardwareUI() {
  const idMap = { lidar: 'hwLidar', imu: 'hwImu', encoder: 'hwEncoder', camera: 'hwCamera', battery: 'hwBattery' };
  for (const [name, info] of Object.entries(state.nodeHealth)) {
    const statusEl = el(idMap[name]);
    const item = statusEl?.closest('.hw-item');
    if (!statusEl || !item) continue;
    const st = info.status || 'offline';
    item.setAttribute('data-state', st);
    statusEl.textContent = st === 'active' ? 'ONLINE' : st === 'warn' ? 'NO DATA' : 'OFFLINE';
  }

  // Overlay battery condition from live battery payload if available.
  const b = state.battery;
  const battItem = document.querySelector('.hw-item[data-hw="battery"]');
  const battStatus = el('hwBattery');
  if (!battItem || !battStatus || !b) return;

  if (b.state === 'critical') {
    battItem.setAttribute('data-state', 'critical');
    battStatus.textContent = 'CRITICAL';
  } else if (b.state === 'low') {
    battItem.setAttribute('data-state', 'warn');
    battStatus.textContent = 'LOW';
  } else if (b.state === 'normal') {
    battItem.setAttribute('data-state', 'active');
    battStatus.textContent = 'NORMAL';
  }
}

function refreshBatteryDetails() {
  const pct = el('hwBatteryPct');
  const volt = el('hwBatteryVolt');
  const b = state.battery;
  if (!pct || !volt || !b) return;

  pct.textContent = typeof b.percentage === 'number' ? `${(b.percentage * 100).toFixed(1)}%` : '--%';
  volt.textContent = typeof b.voltage === 'number' ? `${b.voltage.toFixed(2)} V` : '-- V';
}

function refreshEstopUI() {
  const banner = el('estopBanner');
  const statusEl = el('estopStatus');
  if (!banner || !statusEl) return;
  const e = state.estop;
  if (!e || e.engaged === null) {
    banner.setAttribute('data-state', 'unknown');
    statusEl.textContent = 'UNKNOWN';
  } else if (e.engaged) {
    banner.setAttribute('data-state', 'engaged');
    statusEl.textContent = 'ENGAGED';
  } else {
    banner.setAttribute('data-state', 'released');
    statusEl.textContent = 'RELEASED';
  }
  refreshEstopLocks();
}

function isEstopEngaged() {
  return !!state.estop?.engaged;
}

function refreshEstopLocks() {
  const locked = isEstopEngaged();
  for (const id of estopLockedButtons) {
    const btn = el(id);
    if (!btn) continue;
    btn.disabled = locked;
    btn.classList.toggle('estop-locked', locked);
    btn.title = locked ? 'Blocked while E-Stop is engaged' : '';
  }
}

async function refreshEstop() {
  try {
    const out = await apiGet('/api/v1/estop');
    if (out?.ok && out.data) {
      state.estop = out.data;
      refreshEstopUI();
    }
  } catch {
    // Keep last known state if endpoint is temporarily unavailable.
  }
}

async function refreshBattery() {
  try {
    const battery = await apiGet('/api/v1/battery');
    if (battery?.ok) {
      state.battery = battery.data;
      refreshBatteryDetails();
      refreshHardwareUI();
    }
  } catch {
    // Keep last known value if endpoint is temporarily unavailable.
  }
}

async function refreshBatteryParams() {
  try {
    const out = await apiGet('/api/v1/battery/params');
    const p = out?.data?.params;
    if (!out?.ok || !p) return;
    if (typeof p.voltage_scale === 'number') el('batteryScale').value = p.voltage_scale.toFixed(3);
    if (typeof p.battery_voltage_min === 'number') el('batteryVmin').value = p.battery_voltage_min.toFixed(1);
    if (typeof p.battery_voltage_max === 'number') el('batteryVmax').value = p.battery_voltage_max.toFixed(1);
    if (typeof p.percentage_mode === 'string') el('batteryPctMode').value = p.percentage_mode;
  } catch {
    // Ignore if battery node is not up yet.
  }
}

async function applyBatteryParams() {
  await apiPost('/api/v1/battery/params', {
    voltage_scale: Number(el('batteryScale').value),
    battery_voltage_min: Number(el('batteryVmin').value),
    battery_voltage_max: Number(el('batteryVmax').value),
    percentage_mode: el('batteryPctMode').value,
  });
  await refreshBattery();
}

function handleEvent(ev) {
  let payload;
  try {
    payload = JSON.parse(ev.data);
  } catch {
    payload = { raw: ev.data };
  }

  appendLog('eventsLog', `event: ${ev.type}`, payload);

  if (ev.type === 'snapshot' || ev.type === 'heartbeat') {
    const mission = payload?.payload?.mission_state || payload?.payload?.status;
    if (mission && typeof mission === 'object') {
      for (const key of Object.keys(state.mission)) {
        if (typeof mission[key] === 'boolean') state.mission[key] = mission[key];
      }
      refreshMissionUI();
    }
    const nh = payload?.payload?.node_health;
    if (nh && typeof nh === 'object') {
      state.nodeHealth = nh;
      refreshHardwareUI();
    }
    const b = payload?.payload?.battery;
    if (b && typeof b === 'object') {
      state.battery = b;
      refreshBatteryDetails();
      refreshHardwareUI();
    }
    const es = payload?.payload?.estop;
    if (es && typeof es === 'object') {
      state.estop = es;
      refreshEstopUI();
    }
  }

  if (ev.type === 'mission_state_changed') {
    const missionState = payload?.payload?.mission_state;
    if (missionState && typeof missionState === 'object') {
      for (const key of Object.keys(state.mission)) {
        if (typeof missionState[key] === 'boolean') state.mission[key] = missionState[key];
      }
      refreshMissionUI();
    }
  }

  if (ev.type === 'estop_changed') {
    const es = payload?.payload?.estop;
    if (es && typeof es === 'object') {
      state.estop = es;
      refreshEstopUI();
    }
  }
}

function connectEvents() {
  if (sseReconnectTimer) {
    clearTimeout(sseReconnectTimer);
    sseReconnectTimer = null;
  }
  if (state.events) {
    state.events.close();
    state.events = null;
  }

  const key = apiKey();
  const query = key ? `?api_key=${encodeURIComponent(key)}` : '';
  const url = `${baseUrl()}/api/v1/events${query}`;

  state.events = new EventSource(url);

  state.events.onopen = () => {
    sseReconnectDelay = 1000;
    el('sseState').textContent = 'Events: Connected';
    el('sseState').classList.add('connected');
    appendLog('eventsLog', 'SSE connected', { url });
  };

  state.events.onerror = () => {
    el('sseState').textContent = 'Events: Reconnecting…';
    el('sseState').classList.remove('connected');
    if (state.events) {
      state.events.close();
      state.events = null;
    }
    sseReconnectTimer = setTimeout(connectEvents, sseReconnectDelay);
    sseReconnectDelay = Math.min(sseReconnectDelay * 2, 30000);
  };

  const types = [
    'snapshot',
    'heartbeat',
    'mission_state_changed',
    'estop_changed',
    'map_saved',
    'map_save_failed',
    'route_progress',
    'process_started',
    'process_stopped',
    'process_exited',
  ];
  types.forEach((t) => state.events.addEventListener(t, handleEvent));
}

function disconnectEvents() {
  if (sseReconnectTimer) {
    clearTimeout(sseReconnectTimer);
    sseReconnectTimer = null;
  }
  sseReconnectDelay = 1000;
  if (state.events) {
    state.events.close();
    state.events = null;
  }
  el('sseState').textContent = 'Events: Disconnected';
  el('sseState').classList.remove('connected');
}

function renderChips(containerId, items, onClick) {
  const root = el(containerId);
  root.innerHTML = '';
  items.forEach((item) => {
    const b = document.createElement('button');
    b.className = 'chip';
    b.textContent = item;
    b.onclick = () => onClick(item);
    root.appendChild(b);
  });
}

async function refreshMaps() {
  const out = await apiGet('/api/v1/maps');
  const items = (out?.data?.items || []).map((x) => x.name);
  renderChips('mapsList', items, (name) => {
    el('navMap').value = name;
    el('mapName').value = name;
  });
}

async function refreshRoutes() {
  const out = await apiGet('/api/v1/routes');
  const items = (out?.data?.items || []).map((x) => x.name);
  renderChips('routesList', items, (name) => {
    el('routeFile').value = `${name}.yaml`;
  });
}

async function runPreset(name, fn) {
  if (isEstopEngaged()) {
    toast('E-Stop is engaged. Release E-Stop before starting a preset.', 'error');
    refreshEstopLocks();
    return;
  }
  if (state.presetBusy) {
    appendLog('responseLog', `Preset ${name} skipped`, { reason: 'another preset is running' });
    return;
  }

  state.presetBusy = true;
  setPresetStatus(`${name} running...`);
  try {
    await fn();
    setPresetStatus(`${name} completed`);
  } catch (err) {
    setPresetStatus(`${name} failed`);
    appendLog('responseLog', `Preset ${name} failed`, { error: String(err) });
  } finally {
    state.presetBusy = false;
  }
}

async function presetStartMapping() {
  await apiPostChecked('/api/v1/slam/start', hwArgs());
}

async function presetSaveFixedMap() {
  const mapName = el('presetMapName').value.trim() || el('mapName').value.trim();
  await apiPostChecked('/api/v1/slam/save_map', { map_name: mapName });
  el('mapName').value = mapName;
  el('navMap').value = mapName;
  await refreshMaps();
}

async function presetStartDeliveryMission() {
  const map = el('presetMapName').value.trim() || el('navMap').value.trim();
  const route = el('presetRouteFile').value.trim() || el('routeFile').value.trim();

  await apiPostChecked('/api/v1/navigation/start', {
    map,
    use_yield_requester: el('useYield').value === 'true',
    base_angular_sign: Number(el('baseAngularSign').value),
  });

  await apiPostChecked('/api/v1/route/start', {
    route,
    yaw_mode: el('yawMode').value,
    start_mode: el('startMode').value,
    pause_sec: Number(el('pauseSec').value),
  });

  el('navMap').value = map;
  el('routeFile').value = route;
}

async function presetBringUp() {
  await apiPostChecked('/api/v1/bringup/start', hwArgs());
}

async function presetStopAllMission() {
  const endpoints = [
    '/api/v1/route/stop',
    '/api/v1/waypoints/record/stop',
    '/api/v1/navigation/stop',
    '/api/v1/localization/stop',
    '/api/v1/slam/stop',
    '/api/v1/bringup/stop',
  ];

  for (const path of endpoints) {
    await apiPost(path, {});
  }
}

function actionBtn(id, fn) {
  const btn = el(id);
  if (!btn) return;
  btn.onclick = async () => {
    if (estopLockedButtons.has(id) && isEstopEngaged()) {
      toast('E-Stop is engaged. Release E-Stop before starting.', 'error');
      refreshEstopLocks();
      return;
    }
    btn.disabled = true;
    btn.classList.add('loading');
    try {
      const out = await fn();
      if (out?.ok) {
        btn.classList.add('success');
        setTimeout(() => btn.classList.remove('success'), 1200);
      } else {
        toast(out?.data?.error || 'Operation failed', 'error');
      }
    } catch (err) {
      toast(err.message || 'Operation failed', 'error');
    } finally {
      btn.disabled = estopLockedButtons.has(id) ? isEstopEngaged() : false;
      btn.classList.remove('loading');
    }
  };
}

function hwArgs() {
  return {
    lidar_product: el('slamLidarProduct').value.trim(),
    lidar_baud: Number(el('slamLidarBaud').value),
    lidar_port: el('slamLidarPort').value.trim(),
    base_angular_sign: Number(el('baseAngularSign').value),
  };
}

function bind() {
  el('btnCheckHealth').onclick = async () => {
    const out = await apiGet('/api/v1/health');
    el('apiState').textContent = out?.ok ? 'API: Healthy' : 'API: Error';
    el('apiState').classList.toggle('connected', !!out?.ok);
    await refreshBattery();
  };

  el('btnConnectEvents').onclick = connectEvents;
  el('btnDisconnectEvents').onclick = () => disconnectEvents();

  actionBtn('btnBringUpStart', () => apiPost('/api/v1/bringup/start', hwArgs()));
  actionBtn('btnBringUpStop', () => apiPost('/api/v1/bringup/stop'));

  actionBtn('btnSlamStart', () => apiPost('/api/v1/slam/start', hwArgs()));
  actionBtn('btnSlamStop', () => apiPost('/api/v1/slam/stop'));
  actionBtn('btnSaveMap', () => apiPost('/api/v1/slam/save_map', { map_name: el('mapName').value.trim() }));
  el('btnListMaps').onclick = refreshMaps;

  actionBtn('btnLocStart', () => apiPost('/api/v1/localization/start', {
    map: el('navMap').value.trim(),
    base_angular_sign: Number(el('baseAngularSign').value),
  }));
  actionBtn('btnLocStop', () => apiPost('/api/v1/localization/stop'));

  actionBtn('btnNavStart', () => apiPost('/api/v1/navigation/start', {
    map: el('navMap').value.trim(),
    use_yield_requester: el('useYield').value === 'true',
    base_angular_sign: Number(el('baseAngularSign').value),
  }));
  actionBtn('btnNavStop', () => apiPost('/api/v1/navigation/stop'));

  actionBtn('btnRecordStart', () => apiPost('/api/v1/waypoints/record/start', {
    output_path: el('recordOutput').value.trim(),
  }));
  actionBtn('btnRecordStop', () => apiPost('/api/v1/waypoints/record/stop'));

  actionBtn('btnRouteStart', () => apiPost('/api/v1/route/start', {
    route: el('routeFile').value.trim(),
    yaw_mode: el('yawMode').value,
    start_mode: el('startMode').value,
    pause_sec: Number(el('pauseSec').value),
  }));
  actionBtn('btnRouteStop', () => apiPost('/api/v1/route/stop'));
  el('btnListRoutes').onclick = refreshRoutes;

  actionBtn('btnStatus', () => apiGet('/api/v1/status'));
  actionBtn('btnValidateConfig', () => apiPost('/api/v1/config/validate', { strict: false }));
  actionBtn('btnKillRos', () => apiPost('/api/v1/system/kill_ros'));

  el('btnPresetBringUp').onclick = () => runPreset('Bring Up', presetBringUp);
  el('btnPresetStartMapping').onclick = () => runPreset('Start Mapping', presetStartMapping);
  el('btnPresetSaveMap').onclick = () => runPreset('Save Fixed Map', presetSaveFixedMap);
  el('btnPresetStartDelivery').onclick = () => runPreset('Start Delivery Mission', presetStartDeliveryMission);
  el('btnPresetStopAll').onclick = () => runPreset('Stop All Mission', presetStopAllMission);

  el('btnBatteryReadParams').onclick = refreshBatteryParams;
  el('btnBatteryApplyParams').onclick = applyBatteryParams;
}

bind();
bindLogFoldButtons();
const baseInput = el('baseUrl');
if (baseInput) {
  const current = (baseInput.value || '').trim();
  if (!current || current.includes('127.0.0.1') || current.includes('localhost')) {
    baseInput.value = defaultApiBaseUrl();
  }
}

// Auto-connect SSE and verify API on page load
(async () => {
  try {
    const out = await apiGet('/api/v1/health');
    el('apiState').textContent = out?.ok ? 'API: Healthy' : 'API: Error';
    el('apiState').classList.toggle('connected', !!out?.ok);
  } catch {
    el('apiState').textContent = 'API: Offline';
  }
  await refreshEstop();
  connectEvents();
})();

refreshMaps();
refreshRoutes();
refreshMissionUI();
refreshBattery();
refreshEstop();
refreshBatteryParams();
setInterval(refreshBattery, 5000);
setInterval(refreshEstop, 1000);
refreshEstopLocks();
