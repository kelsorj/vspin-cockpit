// vspin-cockpit · Access2 + VSpin — front-end logic.
//
// What lives here:
//   1. three.js scene + URDFLoader that reads /urdf-info and mounts the
//      combined vspin+loader URDF. Joints are driven live from two sources:
//        a) SSE events from /events (kinematic_state — smoothest)
//        b) read_status polling every 500ms — a fallback that also fills
//           the KPI strip and status LEDs.
//   2. Per-slot connection UI (Sim/Connect/Disconnect) that POSTs to
//      /devices/{slot}/connect and reflects state on the header pill.
//   3. Command dispatch: any button with data-cmd sends
//      {kind, params} to /devices/{slot}/execute. The params dict is
//      built from the DOM by buildParamsForCmd().
//   4. Jog buttons and teachpoint buttons — one shared handler each.
//   5. A shared activity log at the bottom with colour-coded lines. Both
//      slots share the log so complete_cycle traces read top-to-bottom.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import URDFLoader from 'urdf-loader';

// ---------------------------------------------------------------------------
// Log helper
// ---------------------------------------------------------------------------

const logEl = document.getElementById('log');
const logWrap = document.getElementById('log-wrap');
const verboseEl = document.getElementById('verbose');

function log(line, cls) {
  const span = document.createElement('span');
  if (cls) span.className = cls;
  const t = new Date().toISOString().substring(11, 19);
  span.textContent = `[${t}] ${line}\n`;
  logEl.appendChild(span);
  logEl.scrollTop = logEl.scrollHeight;
}

document.getElementById('log-header').addEventListener('click', (e) => {
  // Ignore clicks on the "clear" link and the verbose checkbox — they own
  // their own behaviour and shouldn't toggle the drawer.
  if (e.target.closest('a, label, input')) return;
  logWrap.classList.toggle('collapsed');
});
document.getElementById('log-clear').addEventListener('click', (e) => {
  e.preventDefault(); e.stopPropagation();
  while (logEl.firstChild) logEl.removeChild(logEl.firstChild);
});

// ---------------------------------------------------------------------------
// Fetch wrapper
// ---------------------------------------------------------------------------

async function jsonPOST(url, body) {
  const r = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  });
  let out;
  try { out = await r.json(); } catch (_e) { out = {}; }
  if (!r.ok) out.ok = false;
  return out;
}

async function jsonGET(url) {
  const r = await fetch(url);
  return r.json();
}

// ---------------------------------------------------------------------------
// Per-slot connection UI
// ---------------------------------------------------------------------------

const slots = ['access2', 'vspin'];

function setConn(slot, on, sim) {
  const pill = document.querySelector(`.conn-pill[data-slot="${slot}"]`);
  const dot = pill.querySelector('[data-conn-dot]');
  dot.classList.toggle('on', !!on);
  dot.classList.toggle('err', on === false);
  const connectBtn = pill.querySelector('[data-cmd="connect"]');
  const disconnectBtn = pill.querySelector('[data-cmd="disconnect"]');
  connectBtn.textContent = on ? 'Reconnect' : 'Connect';
  disconnectBtn.disabled = !on;
  if (typeof sim === 'boolean') pill.querySelector('[data-sim]').checked = sim;
}

async function refreshHealth() {
  try {
    const h = await jsonGET('/health');
    for (const s of slots) {
      const info = h[s] || {};
      setConn(s, !!info.connected, !!info.simulated);
      // Only overwrite the host/port fields on first paint, otherwise the
      // operator's edits would be clobbered every poll.
      const pill = document.querySelector(`.conn-pill[data-slot="${s}"]`);
      if (!pill.dataset.primed) {
        pill.dataset.primed = '1';
        if (s === 'access2') {
          const c = info.connection || {};
          pill.querySelector('[data-host]').value = c.host || '192.168.0.66';
          pill.querySelector('[data-port]').value = c.port || 7612;
        } else {
          pill.querySelector('[data-serial]').value =
            (info.connection || {}).port || '';
        }
      }
    }
  } catch (e) {
    log('health poll failed: ' + e.message, 'log-err');
  }
}

// Wire up Connect / Disconnect on each pill.
document.querySelectorAll('.conn-pill').forEach(pill => {
  const slot = pill.getAttribute('data-slot');
  pill.querySelector('[data-cmd="connect"]').addEventListener('click', async () => {
    const body = { simulated: pill.querySelector('[data-sim]').checked };
    if (slot === 'access2') {
      body.host = pill.querySelector('[data-host]').value;
      body.port = parseInt(pill.querySelector('[data-port]').value, 10);
    } else {
      body.port = pill.querySelector('[data-serial]').value;
    }
    log(`→ ${slot} connect (sim=${body.simulated})`, 'log-cmd');
    const r = await jsonPOST(`/devices/${slot}/connect`, body);
    if (r.ok) {
      log(`  ✓ ${slot} connected`, 'log-ok');
      setConn(slot, true, !!r.simulated);
    } else {
      log(`  ✗ ${slot} connect: ${r.error || r.detail || 'failed'}`, 'log-err');
      setConn(slot, false);
    }
  });
  pill.querySelector('[data-cmd="disconnect"]').addEventListener('click', async () => {
    log(`→ ${slot} disconnect`, 'log-cmd');
    const r = await jsonPOST(`/devices/${slot}/disconnect`);
    if (r.ok) { log(`  ✓ ${slot} disconnected`, 'log-ok'); setConn(slot, false); }
    else { log(`  ✗ ${slot} disconnect failed`, 'log-err'); }
  });
});

// ---------------------------------------------------------------------------
// Command dispatch (per dev-panel)
// ---------------------------------------------------------------------------

async function execute(slot, kind, params) {
  const verbose = verboseEl.checked;
  if (verbose) log(`→ ${slot}.${kind} ${JSON.stringify(params || {})}`, 'log-cmd');
  else log(`→ ${slot}.${kind}`, 'log-cmd');
  const r = await jsonPOST(`/devices/${slot}/execute`, { kind, params: params || {} });
  if (!r.ok) {
    log(`  ✗ ${slot}.${kind}: ${r.error || r.detail || JSON.stringify(r.payload || {})}`, 'log-err');
    return r;
  }
  if (verbose) log(`  ✓ ${JSON.stringify(r.payload || {})}`, 'log-ok');
  else log('  ✓', 'log-ok');
  return r;
}

// Any button with data-cmd inside a .dev-panel routes to that slot.
document.querySelectorAll('.dev-panel button[data-cmd]').forEach(btn => {
  const slot = btn.closest('.dev-panel').getAttribute('data-slot');
  const cmd = btn.getAttribute('data-cmd');
  btn.addEventListener('click', () => execute(slot, cmd, buildParamsForCmd(slot, cmd, btn)));
});

// Jog buttons — Access2 only.
document.querySelectorAll('.dev-panel[data-slot="access2"] button[data-jog]').forEach(btn => {
  btn.addEventListener('click', () => {
    const axis = btn.getAttribute('data-jog');
    const sign = btn.getAttribute('data-dir') === '-' ? -1 : 1;
    const stepId = axis === 'y' ? 'jog-y' : axis === 'z' ? 'jog-z' : 'jog-g';
    const dist = parseFloat(document.getElementById(stepId).value) || 0;
    const speed = parseInt(document.getElementById('access2-speed').value, 10);
    execute('access2', 'move_axis_relative', { axis, delta_mm: sign * dist, speed });
  });
});

// Teachpoint buttons — Access2 only.
document.querySelectorAll('.dev-panel[data-slot="access2"] button[data-teach]').forEach(btn => {
  btn.addEventListener('click', () => {
    const name = btn.getAttribute('data-teach');
    const speed = parseInt(document.getElementById('access2-speed').value, 10);
    execute('access2', 'goto_teachpoint', { name, speed });
  });
});

// Build the per-command params dict. This is the ONE place the DOM ↔ driver
// mapping lives; adding a new command means adding one branch here.
function buildParamsForCmd(slot, cmd, btn) {
  if (slot === 'access2') {
    if (cmd === 'load_plate' || cmd === 'unload_plate') {
      return {
        bucket: parseInt(document.getElementById('cycle-bucket').value, 10),
        speed: parseInt(document.getElementById('access2-speed').value, 10),
      };
    }
    if (cmd === 'complete_cycle') {
      return {
        bucket: parseInt(document.getElementById('cycle-bucket').value, 10),
        rcf: parseFloat(document.getElementById('cycle-rcf').value),
        duration_s: parseFloat(document.getElementById('cycle-time').value),
        speed: parseInt(document.getElementById('access2-speed').value, 10),
      };
    }
    if (cmd === 'reset_estop' || cmd === 'home' || cmd === 'park'
        || cmd === 'open_gripper' || cmd === 'close_gripper' || cmd === 'read_status') {
      return {};
    }
  }
  if (slot === 'vspin') {
    if (cmd === 'spin') {
      // Prefer the field the operator most recently edited: if RPM differs
      // from the derived value, send rpm; else send rcf. Keep both — the
      // driver picks the one that's set. (RCF wins if both are present.)
      return {
        rcf: parseFloat(document.getElementById('spin-rcf').value),
        rpm: parseFloat(document.getElementById('spin-rpm').value),
        duration_s: parseFloat(document.getElementById('spin-time').value),
        accel_pct: parseInt(document.getElementById('spin-accel').value, 10),
        decel_pct: parseInt(document.getElementById('spin-decel').value, 10),
        time_mode: document.getElementById('spin-mode').value,
        rotor_radius_mm: parseFloat(document.getElementById('spin-radius').value),
      };
    }
    if (cmd === 'go_to_bucket' || cmd === 'teach_bucket') {
      return { n: parseInt(btn.getAttribute('data-bucket'), 10) };
    }
  }
  return {};
}

// Keep RCF <-> RPM synced in the VSpin panel. This is a nicety, not a
// requirement — the driver accepts either.
function _rpmFromRcf(rcf, r_mm) { return Math.sqrt(rcf / (0.00001118 * (r_mm / 10.0))); }
function _rcfFromRpm(rpm, r_mm) { return 0.00001118 * (r_mm / 10.0) * rpm * rpm; }
const rcfEl = document.getElementById('spin-rcf');
const rpmEl = document.getElementById('spin-rpm');
const radEl = document.getElementById('spin-radius');
rcfEl.addEventListener('input', () => {
  const r = parseFloat(radEl.value) || 100;
  rpmEl.value = Math.max(1, Math.round(_rpmFromRcf(parseFloat(rcfEl.value) || 0, r)));
});
rpmEl.addEventListener('input', () => {
  const r = parseFloat(radEl.value) || 100;
  rcfEl.value = Math.max(1, Math.round(_rcfFromRpm(parseFloat(rpmEl.value) || 0, r) * 10) / 10);
});

// ---------------------------------------------------------------------------
// URDF viewer
// ---------------------------------------------------------------------------

const viewerStatus = document.getElementById('viewer-status');
const viewerCanvas = document.getElementById('viewer-canvas');
const viewerJoints = document.getElementById('viewer-joints');

function setViewerStatus(txt, err) {
  viewerStatus.textContent = txt;
  viewerStatus.classList.toggle('err', !!err);
}

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.shadowMap.enabled = true;
renderer.setSize(viewerCanvas.clientWidth, viewerCanvas.clientHeight);
viewerCanvas.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color('#0b1220');
const camera = new THREE.PerspectiveCamera(
  50, viewerCanvas.clientWidth / viewerCanvas.clientHeight, 0.01, 50);
camera.position.set(1.4, 1.2, 1.4);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.3, 0);
controls.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const dir = new THREE.DirectionalLight(0xffffff, 0.9);
dir.position.set(3, 5, 3);
scene.add(dir);
scene.add(new THREE.GridHelper(4, 8, 0x334155, 0x1e293b));
scene.add(new THREE.AxesHelper(0.2));

let robot = null;

async function loadUrdf() {
  setViewerStatus('Fetching URDF metadata…');
  const info = await jsonGET('/urdf-info');
  const urdfUrl = info.urdf_url;
  const deviceUrl = info.device_url;
  const loader = new URDFLoader();
  loader.packages = (_pkg) => deviceUrl;
  loader.parseCollision = false;
  const gltfLoader = new GLTFLoader();
  const stlLoader = new STLLoader();
  const daeLoader = new ColladaLoader();
  loader.loadMeshCb = (path, _mgr, done) => {
    const lower = path.toLowerCase();
    const onErr = (err) => { console.warn('mesh load failed', path, err); done(null, err); };
    if (lower.endsWith('.gltf') || lower.endsWith('.glb'))
      gltfLoader.load(path, (g) => done(g.scene), undefined, onErr);
    else if (lower.endsWith('.stl'))
      stlLoader.load(path, (geom) => {
        const mat = new THREE.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.2, roughness: 0.6 });
        done(new THREE.Mesh(geom, mat));
      }, undefined, onErr);
    else if (lower.endsWith('.dae'))
      daeLoader.load(path, (c) => done(c.scene), undefined, onErr);
    else onErr(new Error('unsupported mesh: ' + path));
  };
  setViewerStatus('Fetching URDF …');
  const xml = await fetch(urdfUrl).then(r => r.text());
  setViewerStatus('Parsing URDF …');
  robot = loader.parse(xml);
  // URDF is Z-up; three.js is Y-up. -π/2 around X puts the ground plane down.
  robot.rotation.x = -Math.PI / 2;
  scene.add(robot);

  // One frame after add: frame the camera on the bbox now that meshes have
  // materialized, and report joint counts to the status line.
  setTimeout(() => {
    const box = new THREE.Box3().setFromObject(robot);
    if (!isFinite(box.min.x)) {
      setViewerStatus('URDF loaded but no meshes resolved — check /urdf mount.', true);
      return;
    }
    const size = box.getSize(new THREE.Vector3()).length();
    const center = box.getCenter(new THREE.Vector3());
    camera.position.copy(center).add(new THREE.Vector3(size * 0.8, size * 0.6, size * 0.8));
    controls.target.copy(center);
    controls.update();

    const names = Object.keys(robot.joints || {});
    const movable = names.filter(n => robot.joints[n].jointType !== 'fixed');
    viewerJoints.textContent = `${names.length} joints (${movable.length} movable)`;
    setViewerStatus(`Loaded ${info.joints.join(', ')}`, false);
    setTimeout(() => viewerStatus.style.display = 'none', 2000);
  }, 100);
}

function setJoint(name, value) {
  if (!robot || !robot.joints || !robot.joints[name]) return;
  try { robot.joints[name].setJointValue(value); } catch (_e) {}
}

function tick() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

window.addEventListener('resize', () => {
  const w = viewerCanvas.clientWidth, h = viewerCanvas.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
});

// ---------------------------------------------------------------------------
// SSE — subscribe to kinematic_state events for smooth joint animation
// ---------------------------------------------------------------------------

function openEventStream() {
  const es = new EventSource('/events');
  es.addEventListener('kinematic_state', (msg) => {
    let ev;
    try { ev = JSON.parse(msg.data); } catch (_e) { return; }
    const joints = (ev.payload || {}).joints || {};
    for (const [name, value] of Object.entries(joints)) {
      setJoint(name, value);
    }
  });
  es.onerror = () => {
    // Browser will auto-reconnect; log once so we notice a real problem.
    if (!openEventStream._warned) {
      log('SSE stream dropped; browser will retry', 'log-info');
      openEventStream._warned = true;
    }
  };
  es.onopen = () => { openEventStream._warned = false; };
}

// ---------------------------------------------------------------------------
// Status polling — fills KPI strip + status LEDs, backup for URDF joints
// ---------------------------------------------------------------------------

function setKPI(key, value, decimals = 2) {
  const el = document.querySelector(`[data-kpi="${key}"]`);
  if (!el) return;
  if (value == null || Number.isNaN(value)) el.textContent = '--.--';
  else if (typeof value === 'number') el.textContent = value.toFixed(decimals);
  else el.textContent = String(value);
}
function setFlag(key, on, mode) {
  const el = document.querySelector(`[data-flag="${key}"]`);
  if (!el) return;
  el.classList.toggle('on', !!on);
  if (mode === 'bad') el.classList.add('bad');
}

async function pollAccess2() {
  const r = await jsonPOST('/devices/access2/execute', { kind: 'read_status', params: {} })
    .catch(() => ({ ok: false }));
  if (!r.ok) return;
  const p = r.payload || {};
  const ap = p.axis_positions || {};
  setKPI('access2.y', ap.y_mm);
  setKPI('access2.z', ap.z_mm);
  setKPI('access2.g', ap.gripper_mm);
  setFlag('access2.homed', p.homed);
  setFlag('access2.optical_plate_sensor', p.optical_plate_sensor);
  setFlag('access2.servos_enabled', p.servos_enabled);
  setFlag('access2.estop_tripped', p.estop_tripped, 'bad');
  // Fallback URDF update — SSE gives ~10 Hz during motion, but nothing at
  // idle. This keeps the model in sync after a manual jog completes.
  if (typeof ap.y_mm === 'number' && typeof ap.z_mm === 'number'
      && typeof ap.gripper_mm === 'number') {
    setJoint('dof_y_axis', ap.y_mm / 1000.0);
    setJoint('dof_z_axis', ap.z_mm / 1000.0);
    const gNorm = Math.max(0, Math.min(1, (ap.gripper_mm - 14) / (32 - 14)));
    setJoint('dof_left_finger',  0.014 + gNorm * (0.032 - 0.014));
    setJoint('dof_right_finger', 0.030 - gNorm * (0.030 - 0.016));
  }
}

async function pollVspin() {
  const r = await jsonPOST('/devices/vspin/execute', { kind: 'read_status', params: {} })
    .catch(() => ({ ok: false }));
  if (!r.ok) return;
  const p = r.payload || {};
  setKPI('vspin.rpm', p.rpm, 0);
  setKPI('vspin.bucket', p.current_bucket);
  setFlag('vspin.door_open', p.door_open);
  setFlag('vspin.door_locked', p.door_locked);
  setFlag('vspin.bucket_locked', p.bucket_locked);
  setFlag('vspin.homed', p.homed);
  setFlag('vspin.spinning', p.spinning || p.in_motion);
  if (typeof p.rotor_angle_deg === 'number') {
    setJoint('dof_rotor', p.rotor_angle_deg * Math.PI / 180);
  }
}

// Kick everything off.
(async function boot() {
  try {
    await loadUrdf();
  } catch (e) {
    setViewerStatus('URDF load failed: ' + e.message, true);
    log('URDF load failed: ' + e.message, 'log-err');
  }
  refreshHealth();
  setInterval(refreshHealth, 3000);
  setInterval(pollAccess2, 500);
  setInterval(pollVspin, 500);
  openEventStream();
  log('cockpit ready', 'log-info');
})();
