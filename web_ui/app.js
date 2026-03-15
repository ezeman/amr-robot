const el = (id) => document.getElementById(id);

const state = {
  events: null,
  presetBusy: false,
  mission: {
    slam: false,
    localization: false,
    navigation: false,
    waypoint_record: false,
    route_follow: false,
  },
};

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
    node.classList.toggle('active', !!active);
  }
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
    const mission = payload?.payload?.status || payload?.payload?.mission_state;
    if (mission && typeof mission === 'object') {
      for (const key of Object.keys(state.mission)) {
        if (typeof mission[key] === 'boolean') state.mission[key] = mission[key];
      }
      refreshMissionUI();
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
}

function connectEvents() {
  disconnectEvents();

  const key = apiKey();
  const query = key ? `?api_key=${encodeURIComponent(key)}` : '';
  const url = `${baseUrl()}/api/v1/events${query}`;

  state.events = new EventSource(url);

  state.events.onopen = () => {
    el('sseState').textContent = 'Events: Connected';
    appendLog('eventsLog', 'SSE connected', { url });
  };

  state.events.onerror = () => {
    el('sseState').textContent = 'Events: Error';
  };

  const types = [
    'snapshot',
    'heartbeat',
    'mission_state_changed',
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
  if (state.events) {
    state.events.close();
    state.events = null;
  }
  el('sseState').textContent = 'Events: Disconnected';
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
  await apiPostChecked('/api/v1/slam/start', {
    lidar_product: el('slamLidarProduct').value.trim(),
    lidar_baud: Number(el('slamLidarBaud').value),
    lidar_port: el('slamLidarPort').value.trim(),
    base_angular_sign: Number(el('baseAngularSign').value),
  });
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

async function presetStopAllMission() {
  const endpoints = [
    '/api/v1/route/stop',
    '/api/v1/waypoints/record/stop',
    '/api/v1/navigation/stop',
    '/api/v1/localization/stop',
    '/api/v1/slam/stop',
  ];

  for (const path of endpoints) {
    await apiPost(path, {});
  }
}

function bind() {
  el('btnCheckHealth').onclick = async () => {
    const out = await apiGet('/api/v1/health');
    el('apiState').textContent = out?.ok ? 'API: Healthy' : 'API: Error';
  };

  el('btnConnectEvents').onclick = connectEvents;
  el('btnDisconnectEvents').onclick = disconnectEvents;

  el('btnSlamStart').onclick = () => apiPost('/api/v1/slam/start', {
    lidar_product: el('slamLidarProduct').value.trim(),
    lidar_baud: Number(el('slamLidarBaud').value),
    lidar_port: el('slamLidarPort').value.trim(),
    base_angular_sign: Number(el('baseAngularSign').value),
  });
  el('btnSlamStop').onclick = () => apiPost('/api/v1/slam/stop', {});
  el('btnSaveMap').onclick = () => apiPost('/api/v1/slam/save_map', {
    map_name: el('mapName').value.trim(),
  });
  el('btnListMaps').onclick = refreshMaps;

  el('btnLocStart').onclick = () => apiPost('/api/v1/localization/start', {
    map: el('navMap').value.trim(),
    base_angular_sign: Number(el('baseAngularSign').value),
  });
  el('btnLocStop').onclick = () => apiPost('/api/v1/localization/stop', {});

  el('btnNavStart').onclick = () => apiPost('/api/v1/navigation/start', {
    map: el('navMap').value.trim(),
    use_yield_requester: el('useYield').value === 'true',
    base_angular_sign: Number(el('baseAngularSign').value),
  });
  el('btnNavStop').onclick = () => apiPost('/api/v1/navigation/stop', {});

  el('btnRecordStart').onclick = () => apiPost('/api/v1/waypoints/record/start', {
    output_path: el('recordOutput').value.trim(),
  });
  el('btnRecordStop').onclick = () => apiPost('/api/v1/waypoints/record/stop', {});

  el('btnRouteStart').onclick = () => apiPost('/api/v1/route/start', {
    route: el('routeFile').value.trim(),
    yaw_mode: el('yawMode').value,
    start_mode: el('startMode').value,
    pause_sec: Number(el('pauseSec').value),
  });
  el('btnRouteStop').onclick = () => apiPost('/api/v1/route/stop', {});
  el('btnListRoutes').onclick = refreshRoutes;

  el('btnStatus').onclick = () => apiGet('/api/v1/status');
  el('btnValidateConfig').onclick = () => apiPost('/api/v1/config/validate', { strict: false });
  el('btnKillRos').onclick = () => apiPost('/api/v1/system/kill_ros', {});

  el('btnPresetStartMapping').onclick = () => runPreset('Start Mapping', presetStartMapping);
  el('btnPresetSaveMap').onclick = () => runPreset('Save Fixed Map', presetSaveFixedMap);
  el('btnPresetStartDelivery').onclick = () => runPreset('Start Delivery Mission', presetStartDeliveryMission);
  el('btnPresetStopAll').onclick = () => runPreset('Stop All Mission', presetStopAllMission);
}

bind();
refreshMaps();
refreshRoutes();
refreshMissionUI();
