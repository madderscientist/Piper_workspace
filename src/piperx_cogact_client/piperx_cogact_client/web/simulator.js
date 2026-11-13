import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";
import { createIcons, RefreshCw, RotateCcw, SkipBack, Play, Pause, SkipForward, Focus, Maximize, Square } from "lucide";

createIcons({ icons: { RefreshCw, RotateCcw, SkipBack, Play, Pause, SkipForward, Focus, Maximize, Square } });
THREE.Cache.enabled = true;
const $ = (id) => document.getElementById(id);
const stage = $("scene");
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
stage.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf1f4f5);
scene.add(new THREE.HemisphereLight(0xffffff, 0x829293, 2.5));
const light = new THREE.DirectionalLight(0xffffff, 2.5);
light.position.set(2, -3, 4);
scene.add(light);
const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 50);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
const grid = new THREE.GridHelper(2.4, 24, 0xb9c9c6, 0xd8e0df);
grid.rotation.x = Math.PI / 2;
scene.add(grid, new THREE.AxesHelper(0.18));
const models = new Map();
let manifest;
let state;
let frames = [];
let preview = null;
let previewRequest = null;
let cursor = 0;
let playing = false;
let startedAt = 0;
let startedCursor = 0;
let polling = false;

async function get(path, options = {}) {
  const response = await fetch(path, { cache: "no-store", ...options });
  const data = await response.json();
  if (!response.ok || data.ok === false) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function source() {
  return document.querySelector('input[name="source"]:checked').value;
}

function useBatch() {
  document.querySelector('input[name="source"][value="batch"]').checked = true;
}

function feedback(arm) {
  const live = state?.arms.find((item) => item.id === arm.id);
  const home = state?.calibration.home_poses?.[arm.id];
  return {
    joints_rad: live?.joints_rad || home?.joints_rad || [0, 0, 0, 0, 0, 0],
    gripper_m: live?.gripper_m ?? home?.gripper_m ?? 0,
  };
}

function setJoints(model, pose) {
  const joints = pose.joint_positions_rad || pose.joints_rad;
  for (let index = 0; index < 6; index += 1) {
    model.robot.setJointValue(`joint${index + 1}`, joints[index] + model.arm.angle_offsets_rad[index]);
  }
  const width = pose.gripper_opening_m ?? pose.gripper_m ?? 0;
  model.robot.setJointValue("joint7", width / 2);
  model.robot.setJointValue("joint8", -width / 2);
}

function framePose(arm, frame) {
  return frame?.[arm.role] || feedback(arm);
}

function paint() {
  const lower = Math.floor(cursor);
  const upper = Math.min(lower + 1, frames.length - 1);
  const weight = cursor - lower;
  for (const model of models.values()) {
    let pose = feedback(model.arm);
    if (source() === "batch" && frames.length) {
      const first = framePose(model.arm, frames[lower]);
      const second = framePose(model.arm, frames[upper]);
      const firstJoints = first.joint_positions_rad || first.joints_rad;
      const secondJoints = second.joint_positions_rad || second.joints_rad;
      pose = {
        joints_rad: firstJoints.map((value, index) => value + weight * (secondJoints[index] - value)),
        gripper_m: (1 - weight) * (first.gripper_opening_m ?? first.gripper_m ?? 0)
          + weight * (second.gripper_opening_m ?? second.gripper_m ?? 0),
      };
    }
    setJoints(model, pose);
    if (model.path) model.path.visible = $("sim-path").checked;
  }
  const total = Math.max(0, frames.length - 1);
  $("sim-timeline").max = total;
  $("sim-timeline").value = Math.floor(cursor);
  $("sim-timeline").disabled = total === 0 || previewRequest !== null;
  $("sim-progress").textContent = `${Math.floor(cursor)} / ${total}`;
  $("sim-time").textContent = `${(cursor / (preview?.execute_hz || 30)).toFixed(2)} s`;
  for (const id of ["sim-play", "sim-prev", "sim-next", "sim-reset"]) $(id).disabled = total === 0 || previewRequest !== null;
}

function pause() {
  playing = false;
  $("play-icon").toggleAttribute("hidden", false);
  $("pause-icon").toggleAttribute("hidden", true);
  $("sim-play").title = "播放";
  $("sim-play").setAttribute("aria-label", "播放");
}

function buildFrames() {
  const chunk = preview?.chunk;
  cursor = 0;
  frames = [];
  if (chunk?.steps.length && chunk.steps.every((step) => manifest.arms.every((arm) => step[arm.role].joint_positions_rad))) {
    const start = Object.fromEntries(manifest.arms.map((arm) => [arm.role,
      preview.initial_arms[arm.role] || feedback(arm)]));
    frames = [start, ...chunk.steps];
  }
  $("sim-batch").textContent = chunk ? `批次 ${chunk.request_id.slice(0, 8)}` : "尚未请求预览";
  for (const model of models.values()) {
    if (model.path) {
      scene.remove(model.path);
      model.path.geometry.dispose();
      model.path.material.dispose();
      model.path = null;
    }
    if (!frames.length) continue;
    const points = frames.map((frame) => {
      setJoints(model, framePose(model.arm, frame));
      model.root.updateMatrixWorld(true);
      return model.tcp.getWorldPosition(new THREE.Vector3());
    });
    model.path = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({ color: model.color }));
    scene.add(model.path);
  }
  paint();
}

function fitView() {
  scene.updateMatrixWorld(true);
  const box = new THREE.Box3();
  for (const model of models.values()) box.expandByObject(model.root);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const radius = Math.max(box.getBoundingSphere(new THREE.Sphere()).radius, 0.25);
  const halfFov = Math.atan(Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * Math.min(camera.aspect, 1));
  const distance = radius / Math.sin(halfFov) * 1.12;
  camera.position.copy(center).add(new THREE.Vector3(1.3, -1.7, 1.15).normalize().multiplyScalar(distance));
  controls.target.copy(center);
  controls.update();
}

function applyManifest() {
  grid.position.z = manifest.board_z_m;
  for (const arm of manifest.arms) {
    const model = models.get(arm.id);
    if (!model) continue;
    model.arm = arm;
    model.root.matrix.set(...arm.world_from_base.flat());
    model.root.matrixWorldNeedsUpdate = true;
    // 模型原样遵循 URDF；仅将 TCP 标记从驱动 Link6 换算到模型 Link6。
    model.tcp.matrix.set(...arm.model_link6_from_driver_link6.flat())
      .multiply(new THREE.Matrix4().set(...arm.flange_from_tcp.flat()));
    model.tcp.matrixWorldNeedsUpdate = true;
  }
}

async function loadModels() {
  const manager = new THREE.LoadingManager();
  const failures = [];
  const meshesReady = new Promise((resolve) => { manager.onLoad = resolve; });
  manager.onError = (url) => failures.push(url);
  manager.onProgress = (_url, loaded, total) => { $("sim-status").textContent = `加载模型 ${loaded} / ${total}`; };
  const loader = new URDFLoader(manager);
  loader.packages = { piper_description: "/model-assets" };
  await Promise.all(manifest.arms.map(async (arm) => {
    const robot = await loader.loadAsync(arm.urdf_url);
    // 可视化忠实回放输入，不让加载库额外裁剪关节角或夹爪开度。
    for (const joint of Object.values(robot.joints)) joint.ignoreLimits = true;
    const root = new THREE.Group();
    root.matrixAutoUpdate = false;
    root.add(robot);
    scene.add(root);
    const color = arm.role === "left" ? 0x087f71 : 0xb96c2c;
    const tcp = new THREE.Group();
    tcp.matrixAutoUpdate = false;
    tcp.add(new THREE.AxesHelper(0.07), new THREE.Mesh(new THREE.SphereGeometry(0.009, 12, 8),
      new THREE.MeshBasicMaterial({ color })));
    robot.links.link6.add(tcp);
    models.set(arm.id, { arm, robot, root, tcp, color, path: null });
  }));
  await meshesReady;
  if (failures.length) throw new Error(`mesh 加载失败：${failures.join(", ")}`);
  applyManifest();
  buildFrames();
  fitView();
  $("sim-status").textContent = `${models.size} 臂 · URDF / mesh 已加载`;
}

async function poll() {
  if (polling) return;
  polling = true;
  try {
    state = await get("/api/state");
    if (state.calibration.calibration_id !== manifest.calibration_id) {
      manifest = await get("/api/simulator");
      applyManifest();
    }
    paint();
  } catch (error) {
    $("sim-status").textContent = error.message;
  } finally {
    polling = false;
  }
}

function startPlayback() {
  useBatch();
  if (cursor >= frames.length - 1) cursor = 0;
  startedAt = performance.now();
  startedCursor = cursor;
  playing = true;
  $("play-icon").toggleAttribute("hidden", true);
  $("pause-icon").toggleAttribute("hidden", false);
  $("sim-play").title = "暂停";
  $("sim-play").setAttribute("aria-label", "暂停");
}

function renderPreviewRequest() {
  const busy = previewRequest !== null;
  $("sim-load").disabled = busy;
  $("sim-load").classList.toggle("is-busy", busy);
  $("sim-load").setAttribute("aria-busy", String(busy));
  $("sim-request-label").textContent = busy ? "请求中..." : "请求预览";
}

async function requestPreview() {
  if (previewRequest) return;
  pause();
  const controller = new AbortController();
  previewRequest = controller;
  renderPreviewRequest();
  paint();
  try {
    const result = await get("/api/simulator/preview", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ instruction: state.settings.instruction }), signal: controller.signal,
    });
    if (previewRequest !== controller) return;
    preview = result;
    useBatch();
    buildFrames();
    startPlayback();
  } catch (error) {
    if (error.name !== "AbortError") $("sim-status").textContent = error.message;
  } finally {
    if (previewRequest === controller) {
      previewRequest = null;
      renderPreviewRequest();
      paint();
    }
  }
}

$("sim-play").onclick = () => {
  if (playing) pause();
  else if (cursor > 0 && cursor < frames.length - 1) startPlayback();
  else requestPreview();
};
function seek(position) {
  pause();
  useBatch();
  cursor = Math.max(0, Math.min(position, frames.length - 1));
  paint();
}
$("sim-prev").onclick = () => seek(Math.floor(cursor) - 1);
$("sim-next").onclick = () => seek(Math.floor(cursor) + 1);
$("sim-reset").onclick = () => seek(0);
$("sim-timeline").oninput = (event) => seek(Number(event.target.value));
$("sim-load").onclick = requestPreview;
$("sim-stop").onclick = () => {
  pause();
  previewRequest?.abort();
  previewRequest = null;
  renderPreviewRequest();
  paint();
};
$("sim-fit").onclick = fitView;
$("sim-fullscreen").onclick = () => stage.requestFullscreen().catch((error) => { $("sim-status").textContent = error.message; });
$("sim-speed").onchange = () => { startedAt = performance.now(); startedCursor = cursor; };
$("sim-path").onchange = paint;
document.querySelectorAll('input[name="source"]').forEach((input) => { input.onchange = () => { pause(); paint(); }; });
new ResizeObserver(() => {
  renderer.setSize(stage.clientWidth, stage.clientHeight);
  camera.aspect = stage.clientWidth / stage.clientHeight;
  camera.updateProjectionMatrix();
  if (models.size) fitView();
}).observe(stage);

function animate(now) {
  requestAnimationFrame(animate);
  if (playing) {
    cursor = Math.min(frames.length - 1,
      startedCursor + (now - startedAt) / 1000 * (preview.execute_hz || 30) * Number($("sim-speed").value));
    if (cursor >= frames.length - 1) pause();
    paint();
  }
  controls.update();
  renderer.render(scene, camera);
}
requestAnimationFrame(animate);
try {
  [manifest, state] = await Promise.all([get("/api/simulator"), get("/api/state")]);
  await loadModels();
  setInterval(poll, 500);
} catch (error) {
  $("sim-status").textContent = `模型加载失败：${error.message}`;
}