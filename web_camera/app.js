/* global ROSLIB */
'use strict';

/* ─── State ───────────────────────────────────────────────────────────── */
const S = {
  ros: null,
  connected: false,
  subs: [],
  /* stats */
  frameCount: 0,
  detectionCount: 0,
  fpsFrames: [],
  objectCounts: {},
  recording: false,
  recordedFrames: [],
  /* image */
  imgWidth: 0,
  imgHeight: 0,
  currentImg: null,
  currentDetections: [],
  /* depth */
  depthData: null,
  depthW: 0,
  depthH: 0,
  /* timing */
  lastRgbTime: 0,
  lastDetTime: 0,
  lastDepthTime: 0,
};

/* ─── Helpers ─────────────────────────────────────────────────────────── */
const byId = (id) => document.getElementById(id);

function defaultWsUrl() {
  const host = window.location.hostname || '127.0.0.1';
  return 'ws://' + host + ':9090';
}

function chipClass(el, level) {
  el.classList.remove('ok', 'warn', 'danger');
  if (level) el.classList.add(level);
}

/* deterministic colour per class label */
const CLASS_COLORS = {};
function colorForClass(label) {
  if (CLASS_COLORS[label]) return CLASS_COLORS[label];
  const PALETTE = [
    '#43c5ff', '#24d18f', '#f4ba59', '#f15f6f', '#c97dff',
    '#ff8a5c', '#5ce1e6', '#ffd166', '#ff6b6b', '#48dbfb',
    '#1dd1a1', '#feca57', '#ff9ff3', '#54a0ff', '#00d2d3',
    '#ee5a24', '#a29bfe', '#fd79a8', '#e17055', '#00cec9',
  ];
  const idx = Object.keys(CLASS_COLORS).length % PALETTE.length;
  CLASS_COLORS[label] = PALETTE[idx];
  return CLASS_COLORS[label];
}

/* ─── UI Updaters ─────────────────────────────────────────────────────── */
function setConn(text, level) {
  const el = byId('connStatus');
  el.textContent = text;
  chipClass(el, level);
}

function setCam(text, level) {
  const el = byId('camStatus');
  el.textContent = text;
  chipClass(el, level);
}

function setDet(text, level) {
  const el = byId('detStatus');
  el.textContent = text;
  chipClass(el, level);
}

function sliderVal(sliderId, valId, fmt) {
  const s = byId(sliderId);
  const v = byId(valId);
  s.oninput = () => { v.textContent = fmt(s.value); };
}

function getRefreshMs() {
  const fps = parseFloat(byId('refreshRate').value) || 2;
  return 1000 / fps;
}

function updateStats() {
  byId('statFrames').textContent = S.frameCount;
  byId('statDetections').textContent = S.detectionCount;
  /* fps from last 30 frames */
  const now = Date.now();
  S.fpsFrames.push(now);
  while (S.fpsFrames.length > 30) S.fpsFrames.shift();
  if (S.fpsFrames.length > 1) {
    const dt = (S.fpsFrames[S.fpsFrames.length - 1] - S.fpsFrames[0]) / 1000;
    byId('statFps').textContent = dt > 0 ? ((S.fpsFrames.length - 1) / dt).toFixed(1) : '0.0';
  }
}

function updateObjectCounts() {
  const el = byId('objectCounts');
  const entries = Object.entries(S.objectCounts).sort((a, b) => b[1] - a[1]);
  el.innerHTML = entries.map(([label, count]) => {
    const c = colorForClass(label);
    return '<span class="obj-tag" style="border-color:' + c + '">' +
      label + ' <span class="count" style="background:' + c + '">' + count + '</span></span>';
  }).join('');
}

function addDetLog(detections) {
  const el = byId('detLog');
  const t = new Date().toLocaleTimeString();
  for (const d of detections) {
    const div = document.createElement('div');
    div.className = 'entry';
    div.innerHTML = '<span class="time">' + t + '</span> ' +
      '<span class="label">' + escHtml(d.label) + '</span> ' +
      '<span class="conf">' + (d.score * 100).toFixed(0) + '%</span>';
    el.prepend(div);
  }
  /* keep max 200 entries */
  while (el.children.length > 200) el.removeChild(el.lastChild);
}

function escHtml(s) {
  const d = document.createElement('div');
  d.textContent = s;
  return d.innerHTML;
}

/* ─── Canvas Drawing ──────────────────────────────────────────────────── */
function drawFrame() {
  const canvas = byId('camCanvas');
  const ctx = canvas.getContext('2d');
  if (!S.currentImg || !S.currentImg.complete || S.currentImg.naturalWidth === 0) return;

  const w = S.currentImg.naturalWidth;
  const h = S.currentImg.naturalHeight;
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  S.imgWidth = w;
  S.imgHeight = h;

  ctx.clearRect(0, 0, w, h);
  ctx.drawImage(S.currentImg, 0, 0);

  /* depth overlay */
  const showDepth = byId('showDepth').checked;
  if (showDepth && S.depthData && S.depthW > 0 && S.depthH > 0) {
    drawDepthOverlay(ctx, w, h);
  }

  /* draw detections */
  const showBoxes = byId('showBoxes').checked;
  const showLabels = byId('showLabels').checked;
  const showConf = byId('showConf').checked;
  const confThresh = parseFloat(byId('confThreshold').value);
  const opacity = parseFloat(byId('boxOpacity').value);

  for (const det of S.currentDetections) {
    if (det.score < confThresh) continue;

    const color = colorForClass(det.label);
    const x = det.x;
    const y = det.y;
    const bw = det.w;
    const bh = det.h;

    if (showBoxes) {
      /* filled box */
      ctx.fillStyle = color + Math.round(opacity * 255).toString(16).padStart(2, '0');
      ctx.fillRect(x, y, bw, bh);
      /* border */
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.strokeRect(x, y, bw, bh);
    }

    if (showLabels || showConf) {
      let text = '';
      if (showLabels) text += det.label;
      if (showLabels && showConf) text += ' ';
      if (showConf) text += (det.score * 100).toFixed(0) + '%';

      ctx.font = 'bold 14px "Chakra Petch", sans-serif';
      const tw = ctx.measureText(text).width;
      const lh = 20;
      const tx = x;
      const ty = y > lh ? y - 4 : y + bh + lh;

      ctx.fillStyle = color;
      ctx.fillRect(tx - 1, ty - lh + 2, tw + 8, lh);
      ctx.fillStyle = '#000';
      ctx.fillText(text, tx + 3, ty - 3);
    }
  }

  /* update resolution display */
  byId('resolution').textContent = w + '×' + h;
  byId('timestamp').textContent = new Date().toLocaleTimeString();
}

/* ─── Depth Overlay ───────────────────────────────────────────────────── */
function drawDepthOverlay(ctx, canvasW, canvasH) {
  /* S.depthData is a Uint16Array (mm) at S.depthW x S.depthH */
  const dw = S.depthW;
  const dh = S.depthH;
  const data = S.depthData;

  /* Scale depth to canvas via offscreen canvas */
  let offCv, offCtx;
  if (typeof OffscreenCanvas !== 'undefined') {
    offCv = new OffscreenCanvas(dw, dh);
    offCtx = offCv.getContext('2d');
  } else {
    offCv = document.createElement('canvas');
    offCv.width = dw;
    offCv.height = dh;
    offCtx = offCv.getContext('2d');
  }
  const imgData = offCtx.createImageData(dw, dh);
  const px = imgData.data;

  for (let i = 0; i < dw * dh; i++) {
    const d = data[i];
    let r = 0, g = 0, b = 0, a = 0;
    if (d > 0 && d < 15000) {
      const t = Math.min(d / 8000, 1.0);
      if (t < 0.25) {
        b = 255; g = Math.round(255 * (t / 0.25));
      } else if (t < 0.5) {
        g = 255; b = Math.round(255 * (1 - (t - 0.25) / 0.25));
      } else if (t < 0.75) {
        g = 255; r = Math.round(255 * ((t - 0.5) / 0.25));
      } else {
        r = 255; g = Math.round(255 * (1 - (t - 0.75) / 0.25));
      }
      a = 120;
    }
    const idx = i * 4;
    px[idx] = r;
    px[idx + 1] = g;
    px[idx + 2] = b;
    px[idx + 3] = a;
  }
  offCtx.putImageData(imgData, 0, 0);
  ctx.drawImage(offCv, 0, 0, canvasW, canvasH);
}

/* ─── ROS Connection ──────────────────────────────────────────────────── */
function disconnect() {
  for (const sub of S.subs) {
    try { sub.unsubscribe(); } catch (_) { /* ignore */ }
  }
  S.subs = [];
  if (S.ros) {
    try { S.ros.close(); } catch (_) { /* ignore */ }
  }
  S.ros = null;
  S.connected = false;
  S.depthData = null;
  S.depthW = 0;
  S.depthH = 0;
  setConn('ROS: Disconnected', '');
  setCam('Camera: OFF', '');
  setDet('Detection: OFF', '');
  byId('overlay').classList.remove('hidden');
  byId('overlayText').textContent = 'No camera feed';
}

function connect() {
  disconnect();
  const url = byId('wsUrl').value.trim();
  if (!url) return;

  setConn('ROS: Connecting…', 'warn');
  const ros = new ROSLIB.Ros({ url: url });
  S.ros = ros;

  ros.on('connection', () => {
    S.connected = true;
    setConn('ROS: Connected', 'ok');
    subscribeTopics(ros);
  });

  ros.on('error', () => {
    setConn('ROS: Error', 'danger');
  });

  ros.on('close', () => {
    S.connected = false;
    setConn('ROS: Closed', 'warn');
    setCam('Camera: OFF', '');
    setDet('Detection: OFF', '');
  });
}

/* ─── Subscriptions ───────────────────────────────────────────────────── */
function subscribeTopics(ros) {
  /* Compute throttle_rate from slider (ms between deliveries, server-side) */
  const throttleMs = Math.max(Math.round(getRefreshMs()), 100);

  /* --- RGB Compressed --- */
  const rgbTopic = byId('rgbTopic').value.trim();
  if (rgbTopic) {
    const sub = new ROSLIB.Topic({
      ros: ros,
      name: rgbTopic,
      messageType: 'sensor_msgs/msg/CompressedImage',
      throttle_rate: throttleMs,
      queue_length: 1,
    });
    sub.subscribe((msg) => {
      const now = Date.now();
      if (now - S.lastRgbTime < getRefreshMs()) return;
      S.lastRgbTime = now;

      if (!msg.data) return;
      const fmt = (msg.format || '').toLowerCase();
      const mime = fmt.includes('png') ? 'image/png' : 'image/jpeg';
      const img = new Image();
      img.onload = () => {
        S.currentImg = img;
        S.frameCount++;
        updateStats();
        drawFrame();
        setCam('Camera: LIVE', 'ok');
        byId('overlay').classList.add('hidden');

        if (S.recording) {
          S.recordedFrames.push({ ts: now, src: img.src });
        }
      };
      img.src = 'data:' + mime + ';base64,' + msg.data;
    });
    S.subs.push(sub);
  }

  /* --- Detection2DArray (vision_msgs) --- */
  const detTopic = byId('detTopic').value.trim();
  if (detTopic) {
    const sub = new ROSLIB.Topic({
      ros: ros,
      name: detTopic,
      messageType: 'vision_msgs/msg/Detection2DArray',
      throttle_rate: throttleMs,
      queue_length: 1,
    });
    sub.subscribe((msg) => {
      const now = Date.now();
      if (now - S.lastDetTime < getRefreshMs()) return;
      S.lastDetTime = now;

      const dets = parseDetections(msg);
      S.currentDetections = dets;
      if (dets.length > 0) {
        S.detectionCount += dets.length;
        setDet('Detection: ' + dets.length + ' objects', 'ok');
        addDetLog(dets);
        for (const d of dets) {
          S.objectCounts[d.label] = (S.objectCounts[d.label] || 0) + 1;
        }
        updateObjectCounts();
      } else {
        setDet('Detection: 0 objects', 'warn');
      }
      drawFrame();
    });
    S.subs.push(sub);

    /* Also try SpatialDetectionArray (depthai_ros_msgs) */
    const sub2 = new ROSLIB.Topic({
      ros: ros,
      name: detTopic,
      messageType: 'depthai_ros_msgs/msg/SpatialDetectionArray',
      throttle_rate: throttleMs,
      queue_length: 1,
    });
    sub2.subscribe((msg) => {
      const now = Date.now();
      if (now - S.lastDetTime < getRefreshMs()) return;
      S.lastDetTime = now;

      const dets = parseSpatialDetections(msg);
      S.currentDetections = dets;
      if (dets.length > 0) {
        S.detectionCount += dets.length;
        setDet('Detection: ' + dets.length + ' objects', 'ok');
        addDetLog(dets);
        for (const d of dets) {
          S.objectCounts[d.label] = (S.objectCounts[d.label] || 0) + 1;
        }
        updateObjectCounts();
      }
      drawFrame();
    });
    S.subs.push(sub2);
  }

  /* --- Depth (raw 16UC1) --- */
  const depthTopic = byId('depthTopic').value.trim();
  if (depthTopic) {
    /* Throttle depth more aggressively (raw 16UC1 is ~1.8 MB/frame) */
    const depthThrottle = Math.max(throttleMs, 500);
    const sub = new ROSLIB.Topic({
      ros: ros,
      name: depthTopic,
      messageType: 'sensor_msgs/msg/Image',
      throttle_rate: depthThrottle,
      queue_length: 1,
    });
    sub.subscribe((msg) => {
      const now = Date.now();
      if (now - S.lastDepthTime < Math.max(getRefreshMs(), 500)) return;
      S.lastDepthTime = now;

      if (!msg.data || !msg.width || !msg.height) return;
      const w = msg.width;
      const h = msg.height;
      try {
        const raw = atob(msg.data);
        const buf = new Uint16Array(w * h);
        for (let i = 0; i < w * h; i++) {
          buf[i] = raw.charCodeAt(i * 2) | (raw.charCodeAt(i * 2 + 1) << 8);
        }
        S.depthData = buf;
        S.depthW = w;
        S.depthH = h;
        /* redraw if depth overlay is on */
        if (byId('showDepth').checked) drawFrame();
      } catch (_) { /* ignore decode error */ }
    });
    S.subs.push(sub);
  }
}

/* ─── Detection Parsers ───────────────────────────────────────────────── */
const COCO_LABELS = [
  'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
  'boat', 'traffic light', 'fire hydrant', '', 'stop sign', 'parking meter', 'bench',
  'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
  'giraffe', '', 'backpack', 'umbrella', '', '', 'handbag', 'tie', 'suitcase',
  'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
  'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', '',
  'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
  'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
  'chair', 'couch', 'potted plant', 'bed', '', 'dining table', '', '', 'toilet',
  '', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
  'oven', 'toaster', 'sink', 'refrigerator', '', 'book', 'clock', 'vase',
  'scissors', 'teddy bear', 'hair drier', 'toothbrush',
];

function labelFromId(id) {
  if (id >= 0 && id < COCO_LABELS.length && COCO_LABELS[id]) return COCO_LABELS[id];
  return 'class_' + id;
}

function parseDetections(msg) {
  /* vision_msgs/Detection2DArray */
  const results = [];
  if (!msg.detections) return results;
  for (const det of msg.detections) {
    const bbox = det.bbox || {};
    const cx = bbox.center ? (bbox.center.position ? bbox.center.position.x : (bbox.center.x || 0)) : 0;
    const cy = bbox.center ? (bbox.center.position ? bbox.center.position.y : (bbox.center.y || 0)) : 0;
    const w = bbox.size_x || 0;
    const h = bbox.size_y || 0;
    let label = 'unknown';
    let score = 0;
    if (det.results && det.results.length > 0) {
      const r = det.results[0];
      score = r.hypothesis ? r.hypothesis.score : (r.score || 0);
      const cls = r.hypothesis ? r.hypothesis.class_id : (r.id || '');
      label = isNaN(cls) ? cls : labelFromId(parseInt(cls));
    }
    results.push({
      label: label,
      score: score,
      x: cx - w / 2,
      y: cy - h / 2,
      w: w,
      h: h,
    });
  }
  return results;
}

function parseSpatialDetections(msg) {
  /* depthai_ros_msgs/SpatialDetectionArray */
  const results = [];
  if (!msg.detections) return results;
  for (const det of msg.detections) {
    const bbox = det.bbox || {};
    const cx = bbox.center ? (bbox.center.position ? bbox.center.position.x : (bbox.center.x || 0)) : 0;
    const cy = bbox.center ? (bbox.center.position ? bbox.center.position.y : (bbox.center.y || 0)) : 0;
    const w = bbox.size_x || 0;
    const h = bbox.size_y || 0;
    let label = 'unknown';
    let score = 0;
    if (det.results && det.results.length > 0) {
      const r = det.results[0];
      score = r.hypothesis ? r.hypothesis.score : (r.score || 0);
      const cls = r.hypothesis ? r.hypothesis.class_id : (r.id || '');
      label = isNaN(cls) ? cls : labelFromId(parseInt(cls));
    }
    /* spatial position (meters) */
    let dist = '';
    if (det.position) {
      const z = det.position.z || det.position.x || 0;
      if (z > 0) dist = ' (' + z.toFixed(2) + 'm)';
    }
    results.push({
      label: label + dist,
      score: score,
      x: cx - w / 2,
      y: cy - h / 2,
      w: w,
      h: h,
    });
  }
  return results;
}

/* ─── Snapshot ────────────────────────────────────────────────────────── */
function takeSnapshot() {
  const canvas = byId('camCanvas');
  if (!canvas || S.imgWidth === 0) return;
  const link = document.createElement('a');
  link.download = 'agv_snapshot_' + Date.now() + '.png';
  link.href = canvas.toDataURL('image/png');
  link.click();
}

/* ─── Record (saves frames to memory, then exports as ZIP-like download) */
function toggleRecord() {
  const btn = byId('btnRecord');
  if (!S.recording) {
    S.recording = true;
    S.recordedFrames = [];
    btn.classList.add('active');
    btn.innerHTML = '&#9632; Stop Rec';
  } else {
    S.recording = false;
    btn.classList.remove('active');
    btn.innerHTML = '&#9679; Record';
    /* export last recorded frames as text file with base64 */
    if (S.recordedFrames.length > 0) {
      const content = S.recordedFrames.map((f) => f.ts + ',' + f.src).join('\n');
      const blob = new Blob([content], { type: 'text/plain' });
      const link = document.createElement('a');
      link.download = 'agv_recording_' + Date.now() + '.csv';
      link.href = URL.createObjectURL(blob);
      link.click();
      URL.revokeObjectURL(link.href);
    }
  }
}

/* ─── Fullscreen ──────────────────────────────────────────────────────── */
function toggleFullscreen() {
  const el = byId('viewerPanel');
  if (!document.fullscreenElement) {
    el.requestFullscreen().catch(() => {});
  } else {
    document.exitFullscreen();
  }
}

/* ─── Init ────────────────────────────────────────────────────────────── */
document.addEventListener('DOMContentLoaded', () => {
  byId('wsUrl').value = defaultWsUrl();

  /* Slider bindings */
  sliderVal('refreshRate', 'refreshVal', (v) => parseFloat(v).toFixed(1));
  sliderVal('jpegQuality', 'qualityVal', (v) => parseInt(v, 10));
  sliderVal('confThreshold', 'confVal', (v) => parseFloat(v).toFixed(2));
  sliderVal('boxOpacity', 'opacityVal', (v) => parseFloat(v).toFixed(2));

  /* Button bindings */
  byId('btnConnect').onclick = connect;
  byId('btnDisconnect').onclick = disconnect;
  byId('btnSnapshot').onclick = takeSnapshot;
  byId('btnFullscreen').onclick = toggleFullscreen;
  byId('btnRecord').onclick = toggleRecord;
  byId('btnClearStats').onclick = () => {
    S.frameCount = 0;
    S.detectionCount = 0;
    S.fpsFrames = [];
    S.objectCounts = {};
    byId('statFrames').textContent = '0';
    byId('statDetections').textContent = '0';
    byId('statFps').textContent = '0.0';
    byId('statLatency').textContent = '—';
    byId('objectCounts').innerHTML = '';
    byId('detLog').innerHTML = '';
  };

  /* Redraw when display settings change */
  const checkboxes = ['showBoxes', 'showLabels', 'showConf', 'showDepth'];
  for (const id of checkboxes) {
    byId(id).onchange = drawFrame;
  }
  byId('confThreshold').oninput = drawFrame;
  byId('boxOpacity').oninput = drawFrame;

  /* Keyboard shortcut: S = snapshot, F = fullscreen, Esc = exit fullscreen */
  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT') return;
    if (e.key === 's' || e.key === 'S') takeSnapshot();
    if (e.key === 'f' || e.key === 'F') toggleFullscreen();
  });
});
