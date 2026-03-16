/* global ROSLIB */

const state = {
  ros: null,
  connected: false,
  manualDisconnect: false,
  reconnectTimer: null,
  mode: 'inspect',
  map: null,
  amclPose: null,
  odomPose: null,
  slamPose: null,
  tfPose: null,
  tfClient: null,
  scan: null,
  globalPlan: null,
  localPlan: null,
  canvas: null,
  ctx: null,
  viewport: {
    zoom: 1.0,
    panX: 0,
    panY: 0,
  },
  followRobot: false,
  rotateMap: false,
  dragging: false,
  dragStart: null,
  mapImageData: null,
  clickAnchor: null,
  topics: {},
  _mapCanvas: null,
  lastSeen: {
    map: 0,
    pose: 0,
    scan: 0,
    globalPlan: 0,
    localPlan: 0,
  },
  healthTimer: null,
};

function byId(id) {
  return document.getElementById(id);
}

function defaultRosbridgeUrl() {
  const host = window.location.hostname || '127.0.0.1';
  return `ws://${host}:9090`;
}

function setConnStatus(text, color = '') {
  const node = byId('connStatus');
  node.textContent = text;
  node.style.borderColor = color || '';
  node.style.color = color || '';
}

function setHealthStatus(text, level) {
  const node = byId('healthStatus');
  node.textContent = text;
  node.classList.remove('ok', 'warn', 'danger');
  if (level) node.classList.add(level);
}

function setMode(mode) {
  state.mode = mode;
  byId('modeStatus').textContent = `Mode: ${mode}`;
}

function updateFollowRobotButton() {
  const btn = byId('btnFollowRobot');
  if (!btn) return;
  btn.textContent = `Follow Robot: ${state.followRobot ? 'On' : 'Off'}`;
  if (state.followRobot) {
    btn.classList.add('primary');
  } else {
    btn.classList.remove('primary');
  }
}

function updateRotateMapButton() {
  const btn = byId('btnRotateMap');
  if (!btn) return;
  btn.textContent = `Heading Up: ${state.rotateMap ? 'On' : 'Off'}`;
  if (state.rotateMap) {
    btn.classList.add('primary');
  } else {
    btn.classList.remove('primary');
  }
}

function updateInfo() {
  const lines = [];
  if (state.map) {
    lines.push(`map: ${state.map.info.width}x${state.map.info.height} res=${state.map.info.resolution.toFixed(3)}`);
  } else {
    lines.push('map: n/a');
  }

  const p = state.amclPose || state.odomPose || state.slamPose || state.tfPose;
  if (p) {
    lines.push(`robot: x=${p.x.toFixed(2)} y=${p.y.toFixed(2)} yaw=${(p.yaw * 180 / Math.PI).toFixed(1)} deg`);
  } else {
    lines.push('robot: n/a');
  }

  if (state.scan) {
    lines.push(`scan points: ${state.scan.ranges.length}`);
  } else {
    lines.push('scan: n/a');
  }

  if (state.globalPlan) {
    lines.push(`global plan poses: ${state.globalPlan.length}`);
  }
  if (state.localPlan) {
    lines.push(`local plan poses: ${state.localPlan.length}`);
  }

  lines.push(`zoom: ${state.viewport.zoom.toFixed(2)}`);
  byId('infoBox').textContent = lines.join('\n');
}

function quatToYaw(q) {
  const siny = 2.0 * (q.w * q.z + q.x * q.y);
  const cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny, cosy);
}

function worldToCanvas(wx, wy) {
  if (!state.map) return { x: 0, y: 0 };
  const info = state.map.info;
  const mx = (wx - info.origin.position.x) / info.resolution;
  const my = (wy - info.origin.position.y) / info.resolution;
  const x = mx;
  const y = info.height - my;
  return {
    x: x * state.viewport.zoom + state.viewport.panX,
    y: y * state.viewport.zoom + state.viewport.panY,
  };
}

function canvasToWorld(cx, cy) {
  if (!state.map) return { x: 0, y: 0 };
  const info = state.map.info;
  const mx = (cx - state.viewport.panX) / state.viewport.zoom;
  const my = info.height - (cy - state.viewport.panY) / state.viewport.zoom;
  return {
    x: info.origin.position.x + mx * info.resolution,
    y: info.origin.position.y + my * info.resolution,
  };
}

function ensureCanvasSize() {
  const canvas = state.canvas;
  const rect = canvas.getBoundingClientRect();
  const w = Math.max(320, Math.floor(rect.width));
  const h = Math.max(240, Math.floor(rect.height));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
}

function buildMapImageData() {
  if (!state.map) {
    state.mapImageData = null;
    state._mapCanvas = null;
    return;
  }
  const { width, height } = state.map.info;
  const img = new ImageData(width, height);
  const src = state.map.data;

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const srcIdx = (height - 1 - y) * width + x;
      const value = src[srcIdx];
      let r = 245;
      let g = 245;
      let b = 245;
      if (value === -1) {
        r = 110;
        g = 120;
        b = 130;
      } else {
        const v = 255 - Math.floor((value / 100.0) * 255);
        r = v;
        g = v;
        b = v;
      }
      const idx = (y * width + x) * 4;
      img.data[idx + 0] = r;
      img.data[idx + 1] = g;
      img.data[idx + 2] = b;
      img.data[idx + 3] = 255;
    }
  }

  state.mapImageData = img;

  // Reuse offscreen canvas if dimensions match to avoid flicker
  let offscreen = state._mapCanvas;
  if (!offscreen || offscreen.width !== width || offscreen.height !== height) {
    offscreen = document.createElement('canvas');
    offscreen.width = width;
    offscreen.height = height;
    state._mapCanvas = offscreen;
  }
  offscreen.getContext('2d').putImageData(img, 0, 0);
}

function drawRobot(pose, color) {
  const ctx = state.ctx;
  const p = worldToCanvas(pose.x, pose.y);
  const r = 8;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
  ctx.stroke();

  const hx = p.x + Math.cos(pose.yaw) * 18;
  const hy = p.y - Math.sin(pose.yaw) * 18;
  ctx.beginPath();
  ctx.moveTo(p.x, p.y);
  ctx.lineTo(hx, hy);
  ctx.stroke();
}

function drawPath(path, color) {
  if (!path || path.length < 2) return;
  const ctx = state.ctx;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  const first = worldToCanvas(path[0].x, path[0].y);
  ctx.moveTo(first.x, first.y);
  for (let i = 1; i < path.length; i++) {
    const p = worldToCanvas(path[i].x, path[i].y);
    ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
}

function drawScan() {
  if (!state.scan) return;
  const ctx = state.ctx;
  // Prefer map-frame poses; fall back to odomPose so scan still renders
  // while TF chain is initialising.
  const robot = state.tfPose || state.slamPose || state.amclPose || state.odomPose;
  if (!robot) return;

  // Lidar offset from base_footprint per installed URDF (0.18 m forward)
  const lidarOffsetX = 0.18;
  const lidarX = robot.x + lidarOffsetX * Math.cos(robot.yaw);
  const lidarY = robot.y + lidarOffsetX * Math.sin(robot.yaw);

  ctx.fillStyle = '#ffb84d';
  const { ranges, angle_min, angle_increment, range_min, range_max } = state.scan;
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i];
    if (!isFinite(r) || r < range_min || r > range_max) continue;
    const a = angle_min + i * angle_increment + robot.yaw;
    const wx = lidarX + r * Math.cos(a);
    const wy = lidarY + r * Math.sin(a);
    const p = worldToCanvas(wx, wy);
    ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
  }
}

function drawClickAnchor() {
  if (!state.clickAnchor) return;
  const ctx = state.ctx;
  const p = worldToCanvas(state.clickAnchor.x, state.clickAnchor.y);
  ctx.strokeStyle = '#f15f6f';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(p.x, p.y, 7, 0, Math.PI * 2);
  ctx.stroke();
}

function draw() {
  ensureCanvasSize();
  const ctx = state.ctx;
  const canvas = state.canvas;

  if (state.followRobot) {
    centerOnRobot();
  }

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const showMap = byId('layerMap').checked;
  const showScan = byId('layerScan').checked;
  const showRobot = byId('layerRobot').checked;
  const showGlobal = byId('layerGlobalPlan').checked;
  const showLocal = byId('layerLocalPlan').checked;

  // Draw placeholder when no data
  if (!state.connected) {
    ctx.fillStyle = '#8fb7cb';
    ctx.font = '16px Chakra Petch, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('ROS Disconnected — press Connect', canvas.width / 2, canvas.height / 2);
  } else if (!state.map) {
    ctx.fillStyle = '#8fb7cb';
    ctx.font = '16px Chakra Petch, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Waiting for /map topic...', canvas.width / 2, canvas.height / 2);
  }

  // Heading-up rotation: rotate entire scene so robot heading points up
  const robot = state.tfPose || state.slamPose || state.amclPose || state.odomPose;
  if (state.rotateMap && robot) {
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(robot.yaw - Math.PI / 2);
    ctx.translate(-cx, -cy);
  }

  if (showMap && state._mapCanvas) {
    const info = state.map.info;
    ctx.save();
    ctx.translate(state.viewport.panX, state.viewport.panY);
    ctx.scale(state.viewport.zoom, state.viewport.zoom);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(state._mapCanvas, 0, 0);
    ctx.restore();

    ctx.strokeStyle = '#2a4f60';
    ctx.lineWidth = 1;
    ctx.strokeRect(
      state.viewport.panX,
      state.viewport.panY,
      info.width * state.viewport.zoom,
      info.height * state.viewport.zoom
    );
  }

  if (showGlobal) drawPath(state.globalPlan, '#31bfff');
  if (showLocal) drawPath(state.localPlan, '#6df5a3');
  if (showScan) drawScan();

  if (showRobot) {
    if (state.tfPose) drawRobot(state.tfPose, '#8fb7cb');
    if (state.slamPose) drawRobot(state.slamPose, '#c97dff');
    if (state.odomPose) drawRobot(state.odomPose, '#f5b545');
    if (state.amclPose) drawRobot(state.amclPose, '#24d18f');
  }

  drawClickAnchor();

  // End heading-up rotation
  if (state.rotateMap && robot) {
    ctx.restore();
  }

  updateInfo();
  requestAnimationFrame(draw);
}

function centerView() {
  if (!state.map) return;
  const w = state.map.info.width;
  const h = state.map.info.height;
  const canvas = state.canvas;
  const zoomX = canvas.width / w;
  const zoomY = canvas.height / h;
  state.viewport.zoom = Math.max(0.4, Math.min(2.5, Math.min(zoomX, zoomY) * 0.95));
  state.viewport.panX = (canvas.width - w * state.viewport.zoom) / 2;
  state.viewport.panY = (canvas.height - h * state.viewport.zoom) / 2;
}

function resetView() {
  state.viewport.zoom = 1.0;
  state.viewport.panX = 0;
  state.viewport.panY = 0;
}

function centerOnRobot() {
  const robot = state.amclPose || state.odomPose || state.slamPose || state.tfPose;
  if (!robot) return;
  const canvas = state.canvas;
  const current = worldToCanvas(robot.x, robot.y);
  state.viewport.panX += canvas.width * 0.5 - current.x;
  state.viewport.panY += canvas.height * 0.5 - current.y;
}

function reconnectDelayMs() {
  const raw = Number(byId('reconnectDelayMs').value);
  if (!Number.isFinite(raw)) return 2000;
  return Math.max(500, Math.min(15000, Math.round(raw)));
}

function wantsAutoReconnect() {
  return byId('autoReconnect').checked;
}

function scheduleReconnect(reason) {
  if (state.manualDisconnect || !wantsAutoReconnect()) return;
  if (state.reconnectTimer) return;
  const delay = reconnectDelayMs();
  setConnStatus(`ROS: Reconnecting in ${delay}ms (${reason})`, '#f4ba59');
  state.reconnectTimer = window.setTimeout(() => {
    state.reconnectTimer = null;
    connectRos('auto');
  }, delay);
}

function clearReconnectTimer() {
  if (!state.reconnectTimer) return;
  window.clearTimeout(state.reconnectTimer);
  state.reconnectTimer = null;
}

function markSeen(key) {
  state.lastSeen[key] = Date.now();
}

function updateHealth() {
  const now = Date.now();
  const ages = [];
  if (state.lastSeen.map) ages.push(now - state.lastSeen.map);
  if (state.lastSeen.pose) ages.push(now - state.lastSeen.pose);
  if (state.lastSeen.scan) ages.push(now - state.lastSeen.scan);
  if (state.lastSeen.globalPlan) ages.push(now - state.lastSeen.globalPlan);
  if (state.lastSeen.localPlan) ages.push(now - state.lastSeen.localPlan);

  if (!state.connected) {
    setHealthStatus('Health: Offline', 'danger');
    return;
  }

  if (ages.length === 0) {
    setHealthStatus('Health: Waiting topics', 'warn');
    return;
  }

  const maxAge = Math.max(...ages);
  if (maxAge < 1500) {
    setHealthStatus('Health: Good', 'ok');
  } else if (maxAge < 5000) {
    setHealthStatus(`Health: Delayed ${Math.round(maxAge / 1000)}s`, 'warn');
  } else {
    setHealthStatus(`Health: Stale ${Math.round(maxAge / 1000)}s`, 'danger');
  }
}

function rosSubscribe(topicName, messageType, cb, opts = {}) {
  const topicOpts = {
    ros: state.ros,
    name: topicName,
    messageType,
    queue_size: 1,
    throttle_rate: 100,
  };
  // rosbridge ROS2: pass QoS overrides when needed
  if (opts.qos) {
    topicOpts.qos = opts.qos;
  }
  const topic = new ROSLIB.Topic(topicOpts);
  topic.subscribe(cb);
  state.topics[topicName] = topic;
}

function disconnectRos(manual = true) {
  state.manualDisconnect = manual;
  clearReconnectTimer();
  state._lastMapStamp = null;

  Object.values(state.topics).forEach((t) => {
    try {
      t.unsubscribe();
    } catch (_e) {
      // ignore
    }
  });
  state.topics = {};

  if (state.tfClient) {
    try { state.tfClient.dispose(); } catch (_e) {}
    state.tfClient = null;
  }

  if (state.ros) {
    try {
      state.ros.close();
    } catch (_e) {
      // ignore
    }
  }
  state.ros = null;
  state.connected = false;
  setConnStatus(manual ? 'ROS: Disconnected' : 'ROS: Connection lost', '#f15f6f');

}

function connectRos(origin = 'manual') {
  state.manualDisconnect = origin === 'manual' ? false : state.manualDisconnect;
  disconnectRos(false);
  clearReconnectTimer();

  const wsUrl = byId('wsUrl').value.trim();
  const ros = new ROSLIB.Ros({ url: wsUrl });
  state.ros = ros;
  setConnStatus('ROS: Connecting...', '#f4ba59');

  ros.on('connection', () => {
    state.manualDisconnect = false;
    state.connected = true;
    setConnStatus('ROS: Connected', '#24d18f');
    console.log('[WebRViz] Connected to', wsUrl);

    // TF-based pose: monitor map→base_footprint via TFClient
    if (state.tfClient) { try { state.tfClient.dispose(); } catch(_e) {} }
    state.tfClient = new ROSLIB.TFClient({
      ros: state.ros,
      fixedFrame: 'map',
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
    });
    state.tfClient.subscribe('base_footprint', (tf) => {
      const siny = 2.0 * (tf.rotation.w * tf.rotation.z + tf.rotation.x * tf.rotation.y);
      const cosy = 1.0 - 2.0 * (tf.rotation.y * tf.rotation.y + tf.rotation.z * tf.rotation.z);
      state.tfPose = {
        x: tf.translation.x,
        y: tf.translation.y,
        yaw: Math.atan2(siny, cosy),
      };
      markSeen('pose');
      if (!state._tfLogged) {
        console.log('[WebRViz] TF pose received', state.tfPose.x.toFixed(2), state.tfPose.y.toFixed(2));
        state._tfLogged = true;
      }
    });

    rosSubscribe('/map', 'nav_msgs/msg/OccupancyGrid', (msg) => {
      // Only accept maps newer than the last one — prevents rosbridge
      // re-delivering stale TRANSIENT_LOCAL cached maps.
      const stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
      if (state._lastMapStamp && stamp <= state._lastMapStamp) {
        return;
      }
      state._lastMapStamp = stamp;
      console.log('[WebRViz] /map received', msg.info.width, 'x', msg.info.height);
      const firstMap = !state.map;
      state.map = msg;
      markSeen('map');
      buildMapImageData();
      if (firstMap) centerView();
    }, {
      qos: {
        durability: 1,   // TRANSIENT_LOCAL
        reliability: 1,  // RELIABLE
        depth: 1,
      },
    });

    rosSubscribe('/amcl_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', (msg) => {
      state.amclPose = {
        x: msg.pose.pose.position.x,
        y: msg.pose.pose.position.y,
        yaw: quatToYaw(msg.pose.pose.orientation),
      };
      markSeen('pose');
    });

    rosSubscribe('/pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', (msg) => {
      state.slamPose = {
        x: msg.pose.pose.position.x,
        y: msg.pose.pose.position.y,
        yaw: quatToYaw(msg.pose.pose.orientation),
      };
      markSeen('pose');
      console.log('[WebRViz] /pose received', state.slamPose.x.toFixed(2), state.slamPose.y.toFixed(2));
    });

    rosSubscribe('/odom', 'nav_msgs/msg/Odometry', (msg) => {
      state.odomPose = {
        x: msg.pose.pose.position.x,
        y: msg.pose.pose.position.y,
        yaw: quatToYaw(msg.pose.pose.orientation),
      };
      markSeen('pose');
    });

    rosSubscribe('/scan', 'sensor_msgs/msg/LaserScan', (msg) => {
      state.scan = msg;
      markSeen('scan');
      if (!state._scanLogged) {
        const robot = state.tfPose || state.slamPose || state.amclPose || state.odomPose;
        console.log('[WebRViz] /scan received', msg.ranges.length, 'points, robot=', robot ? 'yes' : 'no');
        state._scanLogged = true;
      }
    });

    rosSubscribe('/plan', 'nav_msgs/msg/Path', (msg) => {
      state.globalPlan = msg.poses.map((p) => ({ x: p.pose.position.x, y: p.pose.position.y }));
      markSeen('globalPlan');
    });

    rosSubscribe('/local_plan', 'nav_msgs/msg/Path', (msg) => {
      state.localPlan = msg.poses.map((p) => ({ x: p.pose.position.x, y: p.pose.position.y }));
      markSeen('localPlan');
    });


  });

  ros.on('error', () => {
    setConnStatus('ROS: Error', '#f15f6f');
    scheduleReconnect('error');
  });

  ros.on('close', () => {
    if (state.connected) {
      state.connected = false;
      setConnStatus('ROS: Closed', '#f4ba59');
    }
    scheduleReconnect('closed');
  });
}

function publishInitialPose(x, y, yaw) {
  if (!state.ros) return;
  const qz = Math.sin(yaw * 0.5);
  const qw = Math.cos(yaw * 0.5);
  const topic = new ROSLIB.Topic({
    ros: state.ros,
    name: '/initialpose',
    messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
  });

  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      pose: {
        position: { x, y, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: qz, w: qw },
      },
      covariance: [
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0.0, 0, 0, 0,
        0, 0, 0, 0.0, 0, 0,
        0, 0, 0, 0, 0.0, 0,
        0, 0, 0, 0, 0, 0.0685,
      ],
    },
  });

  topic.publish(msg);
}

function publishGoal(x, y, yaw) {
  if (!state.ros) return;
  const qz = Math.sin(yaw * 0.5);
  const qw = Math.cos(yaw * 0.5);
  const topic = new ROSLIB.Topic({
    ros: state.ros,
    name: '/goal_pose',
    messageType: 'geometry_msgs/msg/PoseStamped',
  });

  const msg = new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      position: { x, y, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: qz, w: qw },
    },
  });

  topic.publish(msg);
}

function bindUI() {
  byId('btnConnect').onclick = () => {
    state.manualDisconnect = false;
    connectRos('manual');
  };
  byId('btnDisconnect').onclick = () => {
    state.manualDisconnect = true;
    disconnectRos(true);
  };
  byId('btnFitMap').onclick = centerView;
  byId('btnCenterRobot').onclick = centerOnRobot;
  byId('btnFollowRobot').onclick = () => {
    state.followRobot = !state.followRobot;
    if (state.followRobot) {
      centerOnRobot();
    }
    updateFollowRobotButton();
  };
  byId('btnResetView').onclick = resetView;
  byId('btnRotateMap').onclick = () => {
    state.rotateMap = !state.rotateMap;
    updateRotateMapButton();
  };

  byId('btnModeInspect').onclick = () => setMode('inspect');
  byId('btnModeInit').onclick = () => setMode('set_initial_pose');
  byId('btnModeGoal').onclick = () => setMode('set_goal');

  const canvas = byId('rvizCanvas');
  state.canvas = canvas;
  state.ctx = canvas.getContext('2d');

  canvas.addEventListener('mousedown', (e) => {
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    if (state.mode === 'set_initial_pose' || state.mode === 'set_goal') {
      const w = canvasToWorld(cx, cy);
      if (!state.clickAnchor) {
        state.clickAnchor = w;
      } else {
        const yaw = Math.atan2(w.y - state.clickAnchor.y, w.x - state.clickAnchor.x);
        if (state.mode === 'set_initial_pose') {
          publishInitialPose(state.clickAnchor.x, state.clickAnchor.y, yaw);
        } else {
          publishGoal(state.clickAnchor.x, state.clickAnchor.y, yaw);
        }
        state.clickAnchor = null;
        setMode('inspect');
      }
      return;
    }

    state.dragging = true;
    state.dragStart = { x: cx, y: cy, panX: state.viewport.panX, panY: state.viewport.panY };
  });

  canvas.addEventListener('mousemove', (e) => {
    if (!state.dragging || !state.dragStart) return;
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    state.viewport.panX = state.dragStart.panX + (cx - state.dragStart.x);
    state.viewport.panY = state.dragStart.panY + (cy - state.dragStart.y);
  });

  canvas.addEventListener('mouseup', () => {
    state.dragging = false;
    state.dragStart = null;
  });
  canvas.addEventListener('mouseleave', () => {
    state.dragging = false;
    state.dragStart = null;
  });

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;

    const oldZoom = state.viewport.zoom;
    const factor = e.deltaY < 0 ? 1.08 : 0.92;
    const newZoom = Math.max(0.15, Math.min(8.0, oldZoom * factor));
    state.viewport.zoom = newZoom;

    const wx = (cx - state.viewport.panX) / oldZoom;
    const wy = (cy - state.viewport.panY) / oldZoom;
    state.viewport.panX = cx - wx * newZoom;
    state.viewport.panY = cy - wy * newZoom;
  }, { passive: false });

  window.addEventListener('resize', ensureCanvasSize);

  // --- Teleop controls ---
  const linSlider = byId('linSpeed');
  const angSlider = byId('angSpeed');
  const linVal = byId('linSpeedVal');
  const angVal = byId('angSpeedVal');
  linSlider.oninput = () => { linVal.textContent = parseFloat(linSlider.value).toFixed(2); };
  angSlider.oninput = () => { angVal.textContent = parseFloat(angSlider.value).toFixed(2); };


  let teleopInterval = null;
  let teleopCmd = { linear: 0, angular: 0 };

  function publishCmdVel(lx, az) {
    if (!state.ros || !state.connected) return;
    const topic = new ROSLIB.Topic({
      ros: state.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist',
    });
    topic.publish(new ROSLIB.Message({
      linear: { x: lx, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: az },
    }));
  }

  function startTeleop(lx, az) {
    teleopCmd = { linear: lx, angular: az };
    publishCmdVel(lx, az);
    if (!teleopInterval) {
      teleopInterval = setInterval(() => publishCmdVel(teleopCmd.linear, teleopCmd.angular), 100);
    }
  }

  function stopTeleop() {
    if (teleopInterval) { clearInterval(teleopInterval); teleopInterval = null; }
    teleopCmd = { linear: 0, angular: 0 };
    publishCmdVel(0, 0);
  }

  function getLin() { return parseFloat(linSlider.value); }
  function getAng() { return parseFloat(angSlider.value); }

  // Button press/release handlers
  const teleopButtons = {
    btnForward:   () => [getLin(), 0],
    btnBackward:  () => [-getLin(), 0],
    btnLeft:      () => [0, getAng()],
    btnRight:     () => [0, -getAng()],
    btnTurnLeft:  () => [0, getAng()],
    btnTurnRight: () => [0, -getAng()],
    btnStop:      () => [0, 0],
  };

  Object.entries(teleopButtons).forEach(([id, getCmd]) => {
    const btn = byId(id);
    const onDown = (e) => {
      e.preventDefault();
      const [lx, az] = getCmd();
      if (id === 'btnStop') { stopTeleop(); return; }
      btn.classList.add('active');
      startTeleop(lx, az);
    };
    const onUp = (e) => {
      e.preventDefault();
      btn.classList.remove('active');
      stopTeleop();
    };
    btn.addEventListener('mousedown', onDown);
    btn.addEventListener('mouseup', onUp);
    btn.addEventListener('mouseleave', onUp);
    btn.addEventListener('touchstart', onDown, { passive: false });
    btn.addEventListener('touchend', onUp, { passive: false });
    btn.addEventListener('touchcancel', onUp, { passive: false });
  });

  // Keyboard teleop
  const keyMap = { w: 'btnForward', s: 'btnBackward', a: 'btnLeft', d: 'btnRight', q: 'btnTurnLeft', e: 'btnTurnRight', ' ': 'btnStop' };
  const keysDown = new Set();

  window.addEventListener('keydown', (ev) => {
    if (ev.target.tagName === 'INPUT' || ev.target.tagName === 'TEXTAREA') return;
    const k = ev.key.toLowerCase();
    if (!keyMap[k] || keysDown.has(k)) return;
    keysDown.add(k);
    const btnId = keyMap[k];
    const btn = byId(btnId);
    if (btn) btn.classList.add('active');
    if (k === ' ') { stopTeleop(); return; }
    const [lx, az] = teleopButtons[btnId]();
    startTeleop(lx, az);
  });

  window.addEventListener('keyup', (ev) => {
    const k = ev.key.toLowerCase();
    if (!keyMap[k]) return;
    keysDown.delete(k);
    const btn = byId(keyMap[k]);
    if (btn) btn.classList.remove('active');
    if (keysDown.size === 0) stopTeleop();
  });
}

function main() {
  const wsNode = byId('wsUrl');
  if (wsNode) {
    const current = wsNode.value.trim();
    if (!current || current.includes('192.168.1.41')) {
      wsNode.value = defaultRosbridgeUrl();
    }
  }

  bindUI();
  ensureCanvasSize();
  setMode('inspect');
  updateFollowRobotButton();
  updateRotateMapButton();
  setConnStatus('ROS: Disconnected', '#f15f6f');
  setHealthStatus('Health: Offline', 'danger');
  state.healthTimer = window.setInterval(updateHealth, 500);
  draw();

  // Auto-connect on page load
  connectRos('auto');
}

main();
