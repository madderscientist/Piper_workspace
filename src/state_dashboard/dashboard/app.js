"use strict";

const CONNECTIONS = {live: "ROS 在线", partial: "部分数据", stale: "数据过期", waiting: "等待遥测", offline: "服务断开"};
const ARM_STATES = ["正常", "急停", "无解", "奇异点", "目标超限", "关节通信异常", "抱闸未打开", "碰撞", "示教超速", "关节异常", "其他异常", "示教记录", "示教执行", "示教暂停", "主控过温", "电阻过温"];
const CONTROL_MODES = ["待机", "CAN 控制", "拖动示教", "以太网控制", "Wi-Fi 控制", "遥控器", "联动示教", "离线轨迹"];
const STREAM_NAMES = {joints: "关节反馈", status: "状态反馈", pose: "末端位姿"};
const CAN_SEGMENT_NAMES = [
  "J1/J2 (0x2A5)", "J3/J4 (0x2A6)", "J5/J6 (0x2A7)", "夹爪 (0x2A8)", "整臂状态 (0x2A1)",
  "驱动1 (0x261)", "驱动2 (0x262)", "驱动3 (0x263)", "驱动4 (0x264)", "驱动5 (0x265)", "驱动6 (0x266)",
];
const STUDIO_LAYER_RATE_DEFS = [
  {key: "can_rx_hz", label: "CAN 硬件接收 (0x2A5..2A8)"},
  {key: "loop_hz", label: "驱动主循环 (watchdog)"},
  {key: "comp_hz", label: "重力/摩擦/六维力补偿解算"},
  {key: "cmd_tx_hz", label: "CAN 运动命令下发 (MIT/J/P/L)"},
  {key: "grip_tx_hz", label: "CAN 夹爪命令下发 (0x159)"},
  {key: "ros_cmd_hz", label: "上层 ROS 命令输入 (command/*)"},
  {key: "joints_pub_hz", label: "关节反馈 (joint_states_feedback)"},
  {key: "wrench_pub_hz", label: "末端六维力 (end_wrench_stamped)"},
];
const cards = [];
const histories = Array.from({length: 4}, () => Array.from({length: 6}, () => []));
const wrenchHistories = Array.from({length: 4}, () => Array.from({length: 6}, () => []));
const WRENCH_AXES = [
  {label: "Fx · N", title: "Fx (N)", scale: 30, color: "#0d766e", kind: "force"},
  {label: "Fy · N", title: "Fy (N)", scale: 30, color: "#0f8a5f", kind: "force"},
  {label: "Fz · N", title: "Fz (N)", scale: 30, color: "#096f8a", kind: "force"},
  {label: "Mx · N·m", title: "Mx (N·m)", scale: 5, color: "#2563b9", kind: "torque"},
  {label: "My · N·m", title: "My (N·m)", scale: 5, color: "#7840b0", kind: "torque"},
  {label: "Mz · N·m", title: "Mz (N·m)", scale: 5, color: "#b52668", kind: "torque"},
];
const powerRequests = Object.fromEntries(["enable", "disable"].map(action => [
  action, Array.from({length: 4}, () => ({pending: false, error: "", requestBefore: null, startedAt: 0})),
]));
let lastSnapshot = null;
let lastSuccess = 0;
let studioIndex = 0;
let studioViewer = null;
let studioGhostDeg = null;
let studioTargetPose = null;
let studioInitializedArm = -1;
let studioConfigSig = "";
let studioLastSeenError = "";

const toRadians = deg => deg * Math.PI / 180;
const toDegrees = rad => rad * 180 / Math.PI;
const number = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : "—";
const codeLabel = (labels, value) => labels[value] ?? `未知 (${value})`;
const hasFault = status => status && (
  status.err_code !== 0 || status.angle_limits.some(Boolean) ||
  status.communication_errors.some(Boolean) ||
  ![0, 11, 12, 13].includes(status.arm_status)
);

function createWrenchBox(axis, axisIdx, studio = false) {
  const prefix = studio ? "studio-wrench" : "outer-wrench";
  const barH = studio ? 6 : 5;
  const box = document.createElement("div");
  box.className = studio
    ? `studio-wrench-box wrench-axis-${axisIdx}`
    : `wrench-value ${axis.kind} wrench-axis-${axisIdx}`;
  box.innerHTML = `<div class="${prefix}-head"><span>${studio ? axis.title : axis.label}</span><strong>—</strong></div>` +
    `<svg class="${prefix}-bar" viewBox="0 0 100 ${barH}" preserveAspectRatio="none" aria-hidden="true"><rect class="wrench-bar-fill" x="50" y="0" width="0" height="${barH}"></rect><line x1="50" y1="0" x2="50" y2="${barH}"></line></svg>` +
    `<svg class="${prefix}-spark" viewBox="0 0 100 22" preserveAspectRatio="none" aria-label="${axis.title} 趋势"><line x1="0" y1="11" x2="100" y2="11"></line><path></path></svg>`;
  return box;
}

function renderWrenchBox(box, val, meta, history, digits) {
  box.querySelector("strong").textContent = number(val, digits);
  const fill = box.querySelector(".wrench-bar-fill");
  if (fill) {
    if (Number.isFinite(val)) {
      const ratio = Math.max(-1, Math.min(1, val / meta.scale));
      const pct = Math.abs(ratio) * 50;
      fill.setAttribute("x", (ratio >= 0 ? 50 : 50 - pct).toFixed(2));
      fill.setAttribute("width", pct.toFixed(2));
    } else {
      fill.setAttribute("x", "50");
      fill.setAttribute("width", "0");
    }
  }
  box.querySelector("path")?.setAttribute("d", sparkline(history));
}

function setStudioJointTargetInputs(j, deg) {
  const jRow = document.querySelectorAll(".studio-joint-row")[j];
  if (jRow) {
    jRow.querySelector('input[type="range"]').value = String(deg);
    jRow.querySelector('input[type="number"]').value = deg.toFixed(1);
  }
  const mRow = document.querySelectorAll("#studio-mit-tbody tr")[j];
  if (mRow) {
    const s = mRow.querySelector(".mit-row-slider");
    const p = mRow.querySelector(".mit-row-pos");
    if (s) s.value = String(deg);
    if (p) p.value = deg.toFixed(1);
  }
}

function createCards() {
  const template = document.querySelector("#arm-template");
  for (let index = 0; index < 4; index++) {
    const card = template.content.firstElementChild.cloneNode(true);
    card.querySelector(".arm-index").textContent = String(index + 1).padStart(2, "0");
    card.querySelector("h2").textContent = `机械臂 ${String(index + 1).padStart(2, "0")}`;
    card.querySelector(".namespace").textContent = `/arm_${index + 1}`;
    window.PiperControls.bind(card, index, () => ({
      snapshot: lastSnapshot,
      online: Boolean(lastSnapshot && performance.now() - lastSuccess <= 2500),
    }));
    card.querySelector(".power-button").addEventListener("click", () => {
      const arm = lastSnapshot?.arms[index];
      if (arm) setArmPower(index, nextPowerAction(arm, index, performance.now() - lastSuccess <= 2500));
    });
    card.querySelector(".expand-arm-btn")?.addEventListener("click", () => openStudio(index));
    const tbody = card.querySelector("tbody");
    for (let joint = 0; joint < 6; joint++) {
      const row = document.createElement("tr");
      row.className = `joint-color-${joint}`;
      row.innerHTML = `<td><span class="joint-dot"></span>J${joint + 1}</td><td class="position">—</td><td class="velocity">—</td><td class="effort">—</td><td><svg class="sparkline" viewBox="0 0 100 22" role="img" aria-label="J${joint + 1} 角度趋势"><line x1="0" y1="11" x2="100" y2="11"></line><path></path></svg></td>`;
      tbody.append(row);
    }
    for (const label of ["X · mm", "Y · mm", "Z · mm", "R · °", "P · °", "Y · °"]) {
      const value = document.createElement("div");
      value.className = "pose-value";
      const title = document.createElement("span");
      title.textContent = label;
      const output = document.createElement("strong");
      output.textContent = "—";
      value.append(title, output);
      card.querySelector(".pose-values").append(value);
    }
    WRENCH_AXES.forEach((axis, axisIdx) => {
      card.querySelector(".wrench-values").append(createWrenchBox(axis, axisIdx, false));
    });
    document.querySelector("#arms").append(card);
    cards.push(card);
  }
}

function actualPowerState(arm, backendOnline) {
  return backendOnline && arm.streams.status.fresh ? arm.status?.power_state || "unknown" : "unknown";
}

function nextPowerAction(arm, index, backendOnline) {
  const enablePending = powerRequests.enable[index].pending || arm.enable?.pending;
  return !enablePending && actualPowerState(arm, backendOnline) === "disabled" ? "enable" : "disable";
}

function renderPower(arm, index, backendOnline) {
  const card = cards[index];
  for (const operation of ["enable", "disable"]) {
    const control = arm[operation];
    const local = powerRequests[operation][index];
    if (local.error && control?.requested_at !== local.requestBefore &&
        ["succeeded", "failed"].includes(control?.outcome)) {
      local.error = "";
    }
  }
  const action = nextPowerAction(arm, index, backendOnline);
  const label = action === "enable" ? "使能" : "失能";
  const control = arm[action];
  const disablePending = powerRequests.disable[index].pending || arm.disable?.pending;
  const reason = !backendOnline ? "服务断开，禁止发送指令"
    : !control ? "当前服务不支持使能/失能"
    : action === "enable" && arm.control?.calibration_active ? "末端标定进行中，禁止使能"
    : control.blocked_reason || (!control.available ? "使能/失能服务未就绪" : "");
  const button = card.querySelector(".power-button");
  button.dataset.action = action;
  button.disabled = Boolean(reason || disablePending || !lastSnapshot?.control_token);
  button.textContent = disablePending ? "失能请求处理中…" : `${label}此机械臂`;
  button.setAttribute("aria-label", `${label}机械臂 ${index + 1}`);
  button.title = reason || (action === "enable"
    ? "使能六关节及夹爪；保持测得的夹爪开度，不切换运动模式"
    : arm.status?.teaching_active
      ? "示教中失能任一臂会触发四臂及夹爪全停；请先提供可靠支撑。这不是急停。"
      : "点击失能本机械臂及夹爪；请先确保机械臂有可靠支撑。这不是急停。");
  const state = actualPowerState(arm, backendOnline);
  const stateLabel = {enabled: "六关节已使能", disabled: "六关节已失能", mixed: "部分关节使能", unknown: "电机状态未知"};
  const output = card.querySelector(".power-state");
  output.dataset.state = state;
  output.textContent = stateLabel[state];
  output.title = backendOnline && arm.streams.status.fresh &&
      Array.isArray(arm.status?.motor_enabled) && Array.isArray(arm.status?.motor_enable_valid)
    ? arm.status.motor_enabled.map((enabled, joint) =>
      `J${joint + 1}: ${arm.status.motor_enable_valid[joint] ? enabled ? "使能" : "失能" : "未知"}`).join(" · ")
    : "未收到新鲜的电机使能反馈";
  const operations = ["enable", "disable"];
  const latest = operations.reduce((previous, candidate) => {
    const timestamp = operation => Math.max(
      powerRequests[operation][index].startedAt, (arm[operation]?.requested_at || 0) * 1000);
    return timestamp(candidate) >= timestamp(previous) ? candidate : previous;
  });
  const pending = operations.find(operation => powerRequests[operation][index].pending || arm[operation]?.pending);
  card.dataset.powerPending = String(Boolean(pending || disablePending));
  const feedbackAction = disablePending ? "disable" : pending || latest;
  const local = powerRequests[feedbackAction][index];
  const feedback = arm[feedbackAction];
  const result = card.querySelector(".power-result");
  result.dataset.outcome = local.pending ? "pending" : local.error ? "unknown" : feedback?.outcome || "idle";
  result.textContent = local.pending
    ? `正在等待驱动确认${feedbackAction === "enable" ? "使能" : "失能"}，请勿重复操作…`
    : local.error || (feedback?.outcome !== "idle" ? feedback?.message : "")
      || reason || "按钮跟随六关节实际反馈；未知或部分使能时优先失能。";
  card.querySelector(".enable-service").textContent = arm.enable?.service || `${arm.namespace}/enable_srv`;
}

async function setArmPower(index, action) {
  const arm = lastSnapshot?.arms[index];
  const local = powerRequests[action][index];
  const label = action === "enable" ? "使能" : "失能";
  if (!arm || local.pending || cards[index].querySelector(".power-button").disabled) return;
  local.pending = true;
  local.error = "";
  local.startedAt = Date.now();
  local.requestBefore = arm[action]?.requested_at;
  renderPower(arm, index, true);
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 11000);
  try {
    const response = await fetch(`/api/arms/${arm.id}/${action}`, {
      method: "POST",
      headers: {"Content-Type": "application/json", "X-Piper-Control-Token": lastSnapshot.control_token},
      body: "{}",
      signal: controller.signal,
    });
    const result = await response.json();
    if (!response.ok || result.success !== true) {
      local.error = result.message || `${label}未成功（HTTP ${response.status}），请检查驱动。`;
    } else {
      const currentArm = lastSnapshot?.arms[index] || arm;
      currentArm[action] = {...currentArm[action], pending: false, outcome: "succeeded", message: result.message};
    }
  } catch (error) {
    local.error = `未能确认${label}结果，指令可能已送达；请检查设备，勿盲目重试。${error.name === "AbortError" ? "（请求超时）" : `（${error.message}）`}`;
  } finally {
    clearTimeout(timeout);
    local.pending = false;
    renderPower(lastSnapshot.arms[index], index, performance.now() - lastSuccess <= 2500);
    window.PiperControls.render(lastSnapshot.arms[index], index, performance.now() - lastSuccess <= 2500);
  }
}

function sparkline(values) {
  const valid = values.filter(Number.isFinite);
  if (valid.length < 2) return "";
  const low = Math.min(...valid) - 1;
  const span = Math.max(2, Math.max(...valid) - low + 1);
  let drawing = false;
  return values.map((value, index) => {
    if (!Number.isFinite(value)) {
      drawing = false;
      return "";
    }
    const command = drawing ? "L" : "M";
    drawing = true;
    return `${command}${(index * 100 / Math.max(1, values.length - 1)).toFixed(1)},${(20 - (value - low) / span * 18).toFixed(1)}`;
  }).join(" ");
}

function renderArm(arm, index, backendOnline, appendHistory) {
  const card = cards[index];
  card.dataset.connection = backendOnline ? arm.connection : "offline";
  card.querySelector(".namespace").textContent = arm.namespace;
  card.querySelector(".connection-badge b").textContent = CONNECTIONS[card.dataset.connection];
  renderPower(arm, index, backendOnline);
  const statusFresh = backendOnline && arm.streams.status.fresh;
  const jointsFresh = backendOnline && arm.streams.joints.fresh;
  const poseFresh = backendOnline && arm.streams.pose.fresh;
  const status = arm.status;
  card.querySelector(".arm-state").textContent = status ? codeLabel(ARM_STATES, status.arm_status) : "—";
  card.querySelector(".control-mode").textContent = status ? codeLabel(CONTROL_MODES, status.ctrl_mode) : "—";
  card.querySelector(".motion-state").textContent = status ? ({0: "已到位", 1: "未到位"}[status.motion_status] ?? `未知 (${status.motion_status})`) : "—";
  for (const selector of [".arm-state", ".control-mode", ".motion-state"]) {
    card.querySelector(selector).dataset.stale = String(!statusFresh);
  }
  card.querySelector(".gripper").textContent = number(arm.joints?.gripper_m == null ? null : arm.joints.gripper_m * 1000, 1);
  card.querySelector(".gripper").dataset.stale = String(!jointsFresh);
  card.querySelector(".joint-section").dataset.stale = String(!jointsFresh);
  card.querySelector(".pose-section").dataset.stale = String(!poseFresh);
  const rates = (backendOnline && arm.control_fresh && arm.control?.rates) ? arm.control.rates : null;
  const jointHz = Number.isFinite(rates?.joints_pub_hz) && rates.joints_pub_hz > arm.streams.joints.hz
    ? rates.joints_pub_hz : arm.streams.joints.hz;
  card.querySelector(".joint-rate").textContent = `${number(jointHz, 1)} Hz · ${arm.streams.joints.messages.toLocaleString()} msgs`;
  const setRateCell = (sel, val) => {
    const el = card.querySelector(sel);
    if (el) el.textContent = Number.isFinite(val) ? `${val.toFixed(0)} Hz` : "—";
  };
  setRateCell(".rate-can-rx", rates ? rates.can_rx_hz : (jointsFresh ? jointHz : null));
  setRateCell(".rate-joints", jointsFresh ? jointHz : null);
  setRateCell(".rate-comp", rates ? rates.comp_hz : null);
  setRateCell(".rate-cmd-tx", rates ? rates.cmd_tx_hz : null);
  setRateCell(".rate-ros-cmd", rates ? rates.ros_cmd_hz : null);
  card.querySelectorAll("tbody tr").forEach((row, joint) => {
    const position = arm.joints?.position_deg[joint];
    row.querySelector(".position").textContent = number(position);
    row.querySelector(".velocity").textContent = number(arm.joints?.velocity_rad_s[joint], 3);
    row.querySelector(".effort").textContent = number(arm.joints?.effort_nm[joint], 3);
    if (appendHistory) {
      histories[index][joint].push(jointsFresh && Number.isFinite(position) ? position : null);
      if (histories[index][joint].length > 60) histories[index][joint].shift();
    }
    row.querySelector("path").setAttribute("d", sparkline(histories[index][joint]));
  });
  const coordinates = arm.pose ? [...arm.pose.position_mm, ...arm.pose.rpy_deg] : [];
  card.querySelectorAll(".pose-value strong").forEach((element, axis) => {
    element.textContent = number(coordinates[axis], axis < 3 ? 1 : 2);
  });
  card.querySelector(".pose-frame").textContent = arm.pose?.frame_id || "frame 未指定";
  const controlFresh = Boolean(backendOnline && arm.control_fresh && arm.control?.mode === "mit" && arm.control?.mit_ready);
  const wrench = (controlFresh && Array.isArray(arm.control?.end_wrench)) ? arm.control.end_wrench : [];
  const wrenchSection = card.querySelector(".wrench-section");
  if (wrenchSection) {
    wrenchSection.dataset.stale = String(!controlFresh);
    card.querySelectorAll(".wrench-value").forEach((box, axis) => {
      const meta = WRENCH_AXES[axis];
      const val = wrench[axis];
      if (appendHistory) {
        wrenchHistories[index][axis].push(controlFresh && Number.isFinite(val) ? val : null);
        if (wrenchHistories[index][axis].length > 100) wrenchHistories[index][axis].shift();
      }
      renderWrenchBox(box, val, meta, wrenchHistories[index][axis], axis < 3 ? 1 : 2);
    });
    const fMag = wrench.length >= 3 && wrench.slice(0, 3).every(Number.isFinite)
      ? Math.hypot(wrench[0], wrench[1], wrench[2]) : null;
    const mMag = wrench.length >= 6 && wrench.slice(3, 6).every(Number.isFinite)
      ? Math.hypot(wrench[3], wrench[4], wrench[5]) : null;
    const summaryEl = card.querySelector(".wrench-summary");
    if (summaryEl) summaryEl.textContent = (fMag !== null && mMag !== null)
      ? `|F| ${fMag.toFixed(1)} N · |M| ${mMag.toFixed(2)} N·m`
      : "|F| — N · |M| — N·m";
  }
  const alert = card.querySelector(".alert-line");
  alert.classList.remove("ok", "fault");
  const flags = [];
  if (statusFresh) {
    status.angle_limits.forEach((active, joint) => { if (active) flags.push(`J${joint + 1} 超限`); });
    status.communication_errors.forEach((active, joint) => { if (active) flags.push(`J${joint + 1} 通信异常`); });
    if (![0, 11, 12, 13].includes(status.arm_status)) flags.unshift(codeLabel(ARM_STATES, status.arm_status));
    if (status.err_code !== 0 && flags.length === 0) flags.push("驱动报告故障，检查原始故障码");
    alert.classList.add(hasFault(status) ? "fault" : "ok");
    card.querySelector(".alert-text").textContent = flags.length ? flags.join(" · ") : "当前反馈未报告故障";
    card.querySelector(".alert-icon").textContent = flags.length ? "!" : "✓";
  } else {
    card.querySelector(".alert-text").textContent = status ? "状态反馈已失效，以下为最后一次接收值" : "等待状态反馈，尚无法判断故障";
    card.querySelector(".alert-icon").textContent = "○";
  }
  card.querySelector(".error-code").textContent = status ? `0x${status.err_code.toString(16).toUpperCase().padStart(4, "0")}` : "—";
  const ages = Object.values(arm.streams).map(stream => stream.age_ms).filter(Number.isFinite);
  card.querySelector(".last-seen").textContent = ages.length ? `最近反馈 ${Math.round(Math.min(...ages))} ms 前` : "尚未收到反馈";
  const list = card.querySelector(".stream-list");
  list.replaceChildren();
  for (const [name, stream] of Object.entries(arm.streams)) {
    const row = document.createElement("div");
    row.className = "stream-row";
    const topic = document.createElement("code");
    topic.textContent = stream.topic;
    const detail = document.createElement("span");
    const pubKey = {joints: "joints_pub_hz", status: "status_pub_hz", pose: "pose_pub_hz"}[name];
    const dispHz = (rates && Number.isFinite(rates[pubKey]) && rates[pubKey] > stream.hz) ? rates[pubKey] : stream.hz;
    detail.textContent = `${STREAM_NAMES[name]} · ${backendOnline && stream.fresh ? "有效" : "未收到 / 过期"} · ${number(dispHz, 1)} Hz · ${stream.age_ms == null ? "无数据" : `${stream.age_ms} ms 前`}`;
    row.append(topic, detail);
    if (stream.error) {
      const error = document.createElement("span");
      error.className = "stream-error";
      error.textContent = stream.error;
      row.append(error);
    }
    list.append(row);
  }
  if (rates) {
    for (const extra of [
      {topic: `${arm.namespace}/end_wrench_stamped`, desc: `末端六维力 · ${number(rates.wrench_pub_hz, 1)} Hz`},
      {topic: `${arm.namespace}/control_state`, desc: `控制状态 · ${number(rates.control_pub_hz, 1)} Hz`},
      {topic: "CAN 补偿 / 命令下发", desc: `补偿 ${number(rates.comp_hz, 1)} Hz · MIT下发 ${number(rates.mit_tx_hz, 1)} Hz · 夹爪 ${number(rates.grip_tx_hz, 1)} Hz · 上层输入 ${number(rates.ros_cmd_hz, 1)} Hz`},
    ]) {
      const row = document.createElement("div");
      row.className = "stream-row";
      const topic = document.createElement("code");
      topic.textContent = extra.topic;
      const detail = document.createElement("span");
      detail.textContent = extra.desc;
      row.append(topic, detail);
      list.append(row);
    }
  }
}

function render(snapshot, backendOnline, appendHistory = true) {
  snapshot.arms.forEach((arm, index) => renderArm(arm, index, backendOnline, appendHistory));
  snapshot.arms.forEach((arm, index) => window.PiperControls.render(arm, index, backendOnline));
  renderTeaching(snapshot, backendOnline);
  renderStudio(snapshot, backendOnline);
  document.querySelector("#online-count").textContent = backendOnline ? snapshot.arms.filter(arm => arm.connection === "live").length : 0;
  const statuses = backendOnline ? snapshot.arms.filter(arm => arm.streams.status.fresh) : [];
  document.querySelector("#fault-count").textContent = statuses.length ? statuses.filter(arm => hasFault(arm.status)).length : "—";
  const jointStreams = backendOnline ? snapshot.arms.filter(arm => arm.streams.joints.fresh) : [];
  const armJointRate = arm => {
    const r = arm.control_fresh ? arm.control?.rates?.joints_pub_hz : null;
    return Number.isFinite(r) && r > arm.streams.joints.hz ? r : arm.streams.joints.hz;
  };
  document.querySelector("#total-rate").textContent = jointStreams.length
    ? number(jointStreams.reduce((total, arm) => total + armJointRate(arm), 0), 1) : "—";
  const summaryEl = document.querySelector("#layer-rate-summary");
  if (summaryEl) {
    if (jointStreams.length) {
      const avgJ = jointStreams.reduce((s, a) => s + armJointRate(a), 0) / jointStreams.length;
      const avgComp = jointStreams.reduce((s, a) => s + (a.control?.rates?.comp_hz ?? 0), 0) / jointStreams.length;
      const avgTx = jointStreams.reduce((s, a) => s + (a.control?.rates?.cmd_tx_hz ?? 0), 0) / jointStreams.length;
      const avgIn = jointStreams.reduce((s, a) => s + (a.control?.rates?.ros_cmd_hz ?? 0), 0) / jointStreams.length;
      summaryEl.textContent = `单臂 ${avgJ.toFixed(0)} Hz · 补偿 ${avgComp.toFixed(0)} Hz · 下发 ${avgTx.toFixed(0)} Hz · 输入 ${avgIn.toFixed(0)} Hz`;
    } else {
      summaryEl.textContent = "单臂 — Hz · 补偿 — Hz · 下发 — Hz";
    }
  }
}

async function requestTeaching(action, body = {}) {
  if (!lastSnapshot?.control_token) throw new Error("服务未连接");
  const response = await fetch(`/api/teaching/${action}`, {
    method: "POST",
    headers: {"Content-Type": "application/json", "X-Piper-Control-Token": lastSnapshot.control_token},
    body: JSON.stringify(body),
  });
  const result = await response.json();
  if (!response.ok || result.success !== true) throw new Error(result.message || `HTTP ${response.status}`);
  return result;
}

function setTeachingFeedback(text, isError = false) {
  const el = document.querySelector("#teaching-feedback");
  if (!el) return;
  el.textContent = text;
  el.classList.toggle("error", isError);
}

function renderTeaching(snapshot, backendOnline) {
  const badge = document.querySelector("#teaching-phase");
  const select = document.querySelector("#teaching-mode-select");
  if (!badge || !select) return;
  const t = snapshot.teaching;
  const fresh = backendOnline && snapshot.teaching_fresh && t;
  const active = Boolean(fresh && ["enabling", "preloading", "starting", "engaging", "active"].includes(t.phase));
  badge.dataset.active = String(active);
  select.disabled = active;
  if (!fresh) {
    badge.textContent = "示教：节点未运行";
    badge.title = "未收到 /bilateral_teaching/state 新鲜状态";
    return;
  }
  const errStr = Number.isFinite(t.engage_max_error_deg) ? ` · 误差 ${t.engage_max_error_deg.toFixed(1)}°` : "";
  badge.textContent = `示教：${t.phase}${t.dry_run ? " (预览)" : ""}${errStr}`;
  badge.title = t.reason || "";
  if (document.activeElement !== select && t.mode) {
    select.value = t.mode;
  }
}

function setStudioStatus(text, isError = false) {
  const el = document.querySelector("#studio-action-status");
  if (!el) return;
  el.textContent = text;
  el.classList.toggle("error", isError);
}

function syncStudioInputsFromArm(arm) {
  if (!arm) return;
  studioGhostDeg = null;
  studioTargetPose = null;
  const lowRad = arm.control?.joint_lower;
  const highRad = arm.control?.joint_upper;
  if (arm.joints?.position_deg?.every(Number.isFinite)) {
    const degs = arm.joints.position_deg.map(v => Number(v.toFixed(2)));
    document.querySelectorAll(".studio-joint-row").forEach((row, j) => {
      const range = row.querySelector('input[type="range"]');
      if (Array.isArray(lowRad) && Number.isFinite(lowRad[j])) range.min = String(toDegrees(lowRad[j]));
      if (Array.isArray(highRad) && Number.isFinite(highRad[j])) range.max = String(toDegrees(highRad[j]));
      setStudioJointTargetInputs(j, degs[j]);
    });
  }
  if (arm.pose) {
    const coords = [...arm.pose.position_mm, ...arm.pose.rpy_deg];
    document.querySelectorAll("#studio-cart-inputs input").forEach((inp, idx) => {
      if (Number.isFinite(coords[idx])) inp.value = coords[idx].toFixed(idx < 3 ? 1 : 2);
    });
  }
  if (Number.isFinite(arm.joints?.gripper_m)) {
    const mm = Math.max(0, Math.min(100, arm.joints.gripper_m * 1000));
    document.querySelector("#studio-grip-width-range").value = String(mm);
    document.querySelector("#studio-grip-width").value = mm.toFixed(1);
  }
  const cfg = arm.control?.configuration;
  document.querySelectorAll("#studio-mit-tbody tr").forEach((tr, j) => {
    const slider = tr.querySelector(".mit-row-slider");
    const velInput = tr.querySelector(".mit-row-vel");
    const kpInput = tr.querySelector(".mit-row-kp");
    const kdInput = tr.querySelector(".mit-row-kd");
    const tauInput = tr.querySelector(".mit-row-tau");
    if (slider) {
      if (Array.isArray(lowRad) && Number.isFinite(lowRad[j])) slider.min = String(toDegrees(lowRad[j]));
      if (Array.isArray(highRad) && Number.isFinite(highRad[j])) slider.max = String(toDegrees(highRad[j]));
    }
    const deg = arm.joints?.position_deg?.[j];
    if (Number.isFinite(deg)) setStudioJointTargetInputs(j, deg);
    if (velInput) velInput.value = "0.0";
    if (kpInput) kpInput.value = String(cfg?.kp?.[j] ?? cfg?.debug_kp?.[j] ?? 15.0);
    if (kdInput) kdInput.value = String(cfg?.kd?.[j] ?? cfg?.debug_kd?.[j] ?? 0.5);
    if (tauInput) tauInput.value = "0.0";
  });
  studioViewer?.update({ghostJoints: null, targetPose: null});
}

function readStudioMitPacket() {
  const position = [], velocity = [], kp = [], kd = [], torque = [];
  document.querySelectorAll("#studio-mit-tbody tr").forEach(tr => {
    position.push(toRadians(Number(tr.querySelector(".mit-row-pos").value)));
    velocity.push(Number(tr.querySelector(".mit-row-vel").value));
    kp.push(Number(tr.querySelector(".mit-row-kp").value));
    kd.push(Number(tr.querySelector(".mit-row-kd").value));
    torque.push(Number(tr.querySelector(".mit-row-tau").value));
  });
  return {position, velocity, kp, kd, torque};
}

function syncJointTargetUI(j, valDeg, pushIfTracking = true) {
  setStudioJointTargetInputs(j, valDeg);
  if (!studioGhostDeg) {
    const current = lastSnapshot?.arms[studioIndex]?.joints?.position_deg;
    studioGhostDeg = Array.isArray(current) && current.every(Number.isFinite) ? [...current] : [0, 0, 0, 0, 0, 0];
  }
  studioGhostDeg[j] = valDeg;
  studioViewer?.update({ghostJoints: studioGhostDeg.map(toRadians)});
  window.PiperControls.setDesiredJoint(studioIndex, j, toRadians(valDeg), pushIfTracking);
}

function openStudio(index) {
  studioIndex = index;
  studioInitializedArm = -1;
  const dialog = document.querySelector("#arm-studio");
  if (!dialog.open) dialog.showModal();
  if (!studioViewer && window.PiperMesh) {
    studioViewer = window.PiperMesh.createViewer(
      document.querySelector("#studio-mesh-canvas"),
      {interactive: true, distance: 1.05},
    );
  }
  if (lastSnapshot) {
    syncStudioInputsFromArm(lastSnapshot.arms[index]);
    studioInitializedArm = index;
    renderStudio(lastSnapshot, performance.now() - lastSuccess <= 2500);
  }
}

function initStudio() {
  const dialog = document.querySelector("#arm-studio");
  if (!dialog) return;
  const tabsWrap = document.querySelector("#studio-arm-tabs");
  for (let i = 0; i < 4; i++) {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "studio-arm-tab";
    btn.textContent = `0${i + 1} 机械臂 ${i + 1}`;
    btn.addEventListener("click", () => openStudio(i));
    tabsWrap.append(btn);
  }
  document.querySelector("#studio-close-btn").addEventListener("click", () => dialog.close());
  document.querySelector("#studio-power-btn").addEventListener("click", () => {
    const arm = lastSnapshot?.arms[studioIndex];
    if (arm) setArmPower(studioIndex, nextPowerAction(arm, studioIndex, performance.now() - lastSuccess <= 2500));
  });

  document.querySelectorAll(".studio-tab-btn").forEach(btn => {
    btn.addEventListener("click", () => {
      const target = btn.dataset.studioTab;
      document.querySelectorAll(".studio-tab-btn").forEach(b => b.classList.toggle("active", b === btn));
      document.querySelectorAll(".studio-panel").forEach(p => p.classList.toggle("active", p.dataset.studioPanel === target));
    });
  });

  document.querySelectorAll(".studio-cam-bar button[data-cam]").forEach(btn => {
    btn.addEventListener("click", () => studioViewer?.setPreset(btn.dataset.cam));
  });
  document.querySelector("#studio-wireframe-btn").addEventListener("click", e => {
    if (!studioViewer) return;
    studioViewer.wireframe = !studioViewer.wireframe;
    e.currentTarget.classList.toggle("active", studioViewer.wireframe);
    studioViewer.draw();
  });
  document.querySelector("#studio-ghost-btn").addEventListener("click", e => {
    if (!studioViewer) return;
    studioViewer.showGhost = !studioViewer.showGhost;
    e.currentTarget.classList.toggle("active", studioViewer.showGhost);
    studioViewer.draw();
  });
  document.querySelector("#studio-camera-btn")?.addEventListener("click", e => {
    if (!studioViewer) return;
    studioViewer.showCamera = !studioViewer.showCamera;
    e.currentTarget.classList.toggle("active", studioViewer.showCamera);
    studioViewer.draw();
  });

  const jointGrid = document.querySelector("#studio-joint-inputs");
  const defaultLow = [-150, 0, -170, -89, -89, -170];
  const defaultHigh = [150, 180, 0, 89, 89, 170];
  for (let j = 0; j < 6; j++) {
    const row = document.createElement("div");
    row.className = "studio-joint-row";
    row.innerHTML = `<span class="joint-color-${j}">J${j + 1}</span>` +
      `<em class="j-meas" title="当前实测角">—</em>` +
      `<input type="range" min="${defaultLow[j]}" max="${defaultHigh[j]}" step="0.5" value="0" aria-label="J${j + 1} 目标角度滑块">` +
      `<input type="number" min="-716" max="716" step="0.5" value="0.0" aria-label="J${j + 1} 目标角度数值">` +
      `<em class="j-delta" title="目标与实测偏差">Δ0.0°</em>`;
    const range = row.querySelector('input[type="range"]');
    const num = row.querySelector('input[type="number"]');
    range.addEventListener("input", () => {
      const val = Number(range.value);
      if (Number.isFinite(val)) syncJointTargetUI(j, val, true);
    });
    num.addEventListener("input", () => {
      const val = Number(num.value);
      if (Number.isFinite(val)) syncJointTargetUI(j, val, true);
    });
    jointGrid.append(row);
  }

  document.querySelectorAll("#studio-cart-inputs input").forEach(inp => {
    inp.addEventListener("input", () => {
      const vals = Array.from(document.querySelectorAll("#studio-cart-inputs input")).map(i => Number(i.value));
      if (vals.every(Number.isFinite)) {
        studioTargetPose = [vals[0] / 1000, vals[1] / 1000, vals[2] / 1000, toRadians(vals[3]), toRadians(vals[4]), toRadians(vals[5])];
        studioViewer?.update({targetPose: studioTargetPose});
      }
    });
  });

  const mitTbody = document.querySelector("#studio-mit-tbody");
  for (let j = 0; j < 6; j++) {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td class="joint-color-${j}">J${j + 1}</td>` +
      `<td class="mit-meas-cell">—</td>` +
      `<td class="mit-target-col"><div class="mit-target-cell">` +
        `<input type="range" class="mit-row-slider" min="${defaultLow[j]}" max="${defaultHigh[j]}" step="0.5" value="0" aria-label="J${j + 1} MIT 目标角滑块">` +
        `<input type="number" class="mit-row-pos" step="0.5" value="0.0" aria-label="J${j + 1} MIT 目标角度">` +
      `</div></td>` +
      `<td><input type="number" class="mit-row-vel" step="0.1" value="0.0"></td>` +
      `<td><input type="number" class="mit-row-kp" min="0" max="500" step="0.5" value="15.0"></td>` +
      `<td><input type="number" class="mit-row-kd" min="0" max="100" step="0.1" value="0.5"></td>` +
      `<td><input type="number" class="mit-row-tau" step="0.1" value="0.0"></td>` +
      `<td class="mit-g-cell">—</td>` +
      `<td class="mit-ext-cell">—</td>`;
    const rSlider = tr.querySelector(".mit-row-slider");
    const rPos = tr.querySelector(".mit-row-pos");
    rSlider.addEventListener("input", () => {
      const val = Number(rSlider.value);
      if (Number.isFinite(val)) syncJointTargetUI(j, val, true);
    });
    rPos.addEventListener("input", () => {
      const val = Number(rPos.value);
      if (Number.isFinite(val)) syncJointTargetUI(j, val, true);
    });
    mitTbody.append(tr);
  }

  const wrenchCards = document.querySelector("#studio-wrench-cards");
  if (wrenchCards) {
    WRENCH_AXES.forEach((meta, i) => wrenchCards.append(createWrenchBox(meta, i, true)));
  }
  const wrenchJointTbody = document.querySelector("#studio-wrench-joint-tbody");
  if (wrenchJointTbody) {
    for (let j = 0; j < 6; j++) {
      const tr = document.createElement("tr");
      tr.innerHTML = `<td class="joint-color-${j}">J${j + 1}</td>` +
        `<td class="wj-pos">—</td><td class="wj-ext">—</td><td class="wj-iner">—</td>` +
        `<td class="wj-grav">—</td><td class="wj-fric">—</td><td class="wj-out">—</td>`;
      wrenchJointTbody.append(tr);
    }
  }

  const diagTbody = document.querySelector("#studio-diag-tbody");
  for (let j = 0; j < 7; j++) {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${j < 6 ? `J${j + 1}` : "夹爪"}</td>` +
      `<td class="d-en">—</td><td class="d-pos">—</td><td class="d-vel">—</td>` +
      `<td class="d-eff">—</td><td class="d-lim">—</td><td class="d-com">—</td>`;
    diagTbody.append(tr);
  }

  const canAges = document.querySelector("#studio-can-ages");
  for (const name of CAN_SEGMENT_NAMES) {
    const item = document.createElement("div");
    item.className = "can-age-item";
    item.innerHTML = `<span>${name}</span><strong>—</strong>`;
    canAges.append(item);
  }
  const studioLayerRates = document.querySelector("#studio-layer-rates");
  if (studioLayerRates) {
    for (const def of STUDIO_LAYER_RATE_DEFS) {
      const item = document.createElement("div");
      item.className = "can-age-item";
      item.dataset.rateKey = def.key;
      item.innerHTML = `<span>${def.label}</span><strong>—</strong>`;
      studioLayerRates.append(item);
    }
  }

  document.querySelectorAll("#studio-mode-buttons button").forEach(btn => {
    btn.addEventListener("click", async () => {
      try {
        const res = await window.PiperControls.requestArm(studioIndex, "mode", {mode: btn.dataset.mode});
        setStudioStatus(`模式请求已发送 (${btn.dataset.mode})：${res.message}`);
      } catch (err) {
        setStudioStatus(`模式切换失败：${err.message}`, true);
      }
    });
  });

  const wRange = document.querySelector("#studio-grip-width-range");
  const wNum = document.querySelector("#studio-grip-width");
  const eRange = document.querySelector("#studio-grip-effort-range");
  const eNum = document.querySelector("#studio-grip-effort");
  for (const [rEl, nEl] of [[wRange, wNum], [eRange, eNum]]) {
    rEl.addEventListener("input", () => { nEl.value = Number(rEl.value).toFixed(1); });
    nEl.addEventListener("input", () => { rEl.value = nEl.value; });
  }

  const sendGripper = async (widthM, effort) => {
    try {
      const res = await window.PiperControls.requestArm(studioIndex, "gripper", {width: widthM, effort});
      setStudioStatus(res.message);
    } catch (err) {
      setStudioStatus(`夹爪指令失败：${err.message}`, true);
    }
  };
  document.querySelectorAll("button[data-grip-preset]").forEach(btn => {
    btn.addEventListener("click", () => {
      const w = Number(btn.dataset.gripPreset);
      wRange.value = String(w * 1000);
      wNum.value = (w * 1000).toFixed(1);
      sendGripper(w, Number(eNum.value));
    });
  });
  document.querySelector("#studio-grip-release-btn").addEventListener("click", () => sendGripper(Number(wNum.value) / 1000, 0.0));
  document.querySelector("#studio-grip-send-btn").addEventListener("click", () => sendGripper(Number(wNum.value) / 1000, Number(eNum.value)));

  document.querySelector("#studio-joint-sync-btn").addEventListener("click", () => {
    syncStudioInputsFromArm(lastSnapshot?.arms[studioIndex]);
    setStudioStatus("已同步当前六轴实测角度到目标输入框与 3D 虚影。");
  });
  document.querySelector("#studio-joint-send-btn").addEventListener("click", async () => {
    try {
      const position = Array.from(document.querySelectorAll(".studio-joint-row input[type='number']")).map(i => toRadians(Number(i.value)));
      const speed = Math.round(Number(document.querySelector("#studio-joint-speed").value));
      const res = await window.PiperControls.requestArm(studioIndex, "joints", {position, speed_percent: speed});
      setStudioStatus(res.message);
    } catch (err) {
      setStudioStatus(`关节目标发送失败：${err.message}`, true);
    }
  });

  document.querySelector("#studio-cart-sync-btn").addEventListener("click", () => {
    syncStudioInputsFromArm(lastSnapshot?.arms[studioIndex]);
    setStudioStatus("已同步当前法兰位姿到笛卡尔目标输入框。");
  });
  document.querySelector("#studio-cart-send-btn").addEventListener("click", async () => {
    try {
      const vals = Array.from(document.querySelectorAll("#studio-cart-inputs input")).map(i => Number(i.value));
      const pose = [vals[0] / 1000, vals[1] / 1000, vals[2] / 1000, toRadians(vals[3]), toRadians(vals[4]), toRadians(vals[5])];
      const speed = Math.round(Number(document.querySelector("#studio-cart-speed").value));
      const res = await window.PiperControls.requestArm(studioIndex, "cartesian", {pose, speed_percent: speed});
      setStudioStatus(res.message);
    } catch (err) {
      setStudioStatus(`笛卡尔目标发送失败：${err.message}`, true);
    }
  });

  const studioScale = document.querySelector("#studio-gravity-scale");
  studioScale.addEventListener("input", () => {
    document.querySelector("#studio-gravity-output").textContent = `${Math.round(Number(studioScale.value) * 100)}%`;
  });
  studioScale.addEventListener("change", async () => {
    const arm = lastSnapshot?.arms[studioIndex];
    if (!arm?.control_fresh) return;
    try {
      const res = await window.PiperControls.requestArm(studioIndex, "configure", {
        end_effector: arm.control.configuration.end_effector,
        gravity_scale: Number(studioScale.value),
      });
      setStudioStatus(res.message);
    } catch (err) {
      setStudioStatus(`重力系数保存失败：${err.message}`, true);
    }
  });
  document.querySelector("#studio-save-effector-btn").addEventListener("click", async () => {
    const arm = lastSnapshot?.arms[studioIndex];
    if (!arm?.control_fresh) return;
    try {
      const res = await window.PiperControls.requestArm(studioIndex, "configure", {
        end_effector: document.querySelector("#studio-effector-select").value,
        gravity_scale: arm.control.configuration.gravity_scale,
      });
      setStudioStatus(res.message);
    } catch (err) {
      setStudioStatus(`末端保存失败：${err.message}`, true);
    }
  });
  const calCheck = document.querySelector("#studio-cal-confirm-check");
  const calBtn = document.querySelector("#studio-calibrate-btn");
  calCheck.addEventListener("change", () => {
    const reason = window.PiperControls.getCalibrationBlock(studioIndex);
    calBtn.disabled = Boolean(reason || !calCheck.checked);
  });
  calBtn.addEventListener("click", async () => {
    const reason = window.PiperControls.getCalibrationBlock(studioIndex);
    if (reason || !calCheck.checked) {
      setStudioStatus(reason || "请先勾选确认末端无夹物且已完全闭合", true);
      return;
    }
    calBtn.disabled = true;
    setStudioStatus("正在等待设备标零确认与新鲜零位回读……");
    try {
      const res = await window.PiperControls.requestArm(studioIndex, "calibrate", {confirm_closed: true});
      calCheck.checked = false;
      setStudioStatus(`标定已确认：${res.message}`);
    } catch (err) {
      setStudioStatus(`标定失败或未确认：${err.message}`, true);
    }
  });

  document.querySelector("#studio-mit-sync-btn").addEventListener("click", () => {
    syncStudioInputsFromArm(lastSnapshot?.arms[studioIndex]);
    setStudioStatus("已填入当前实测角与默认 MIT 增益。");
  });
  const activateFreeMove = async () => {
    try {
      await window.PiperControls.stopTracking(studioIndex);
      document.querySelectorAll(".mit-free-btn").forEach(btn => btn.classList.add("active"));
      document.querySelectorAll(".mit-track-btn").forEach(btn => btn.classList.remove("active"));
      syncStudioInputsFromArm(lastSnapshot?.arms[studioIndex]);
      setStudioStatus("已切换为自由移动（零重力浮动）：可手掰机械臂，滑条自动跟随实测角。");
    } catch (err) {
      setStudioStatus(`切换自由移动失败：${err.message}`, true);
    }
  };
  const activateSliderTracking = async () => {
    try {
      if (!window.PiperControls.isMitTracking(studioIndex) && !studioGhostDeg) {
        syncStudioInputsFromArm(lastSnapshot?.arms[studioIndex]);
      }
      const posRad = await window.PiperControls.startTracking(studioIndex);
      if (Array.isArray(posRad) && posRad.length === 6) {
        posRad.forEach((rad, j) => setStudioJointTargetInputs(j, toDegrees(rad)));
      }
      document.querySelectorAll(".mit-free-btn").forEach(btn => btn.classList.remove("active"));
      document.querySelectorAll(".mit-track-btn").forEach(btn => btn.classList.add("active"));
      setStudioStatus("已切换为跟踪滑条（锁定位置）：持续发送 MIT 目标，拖动滑条实时跟随，手推夹爪可直接测六维力。");
    } catch (err) {
      document.querySelectorAll(".mit-free-btn").forEach(btn => btn.classList.add("active"));
      document.querySelectorAll(".mit-track-btn").forEach(btn => btn.classList.remove("active"));
      setStudioStatus(`启动跟踪滑条失败：${err.message}`, true);
    }
  };
  document.querySelectorAll(".mit-free-btn").forEach(btn => btn.addEventListener("click", activateFreeMove));
  document.querySelectorAll(".mit-track-btn").forEach(btn => btn.addEventListener("click", activateSliderTracking));
  document.querySelector("#studio-mit-zero-kp-btn").addEventListener("click", () => {
    document.querySelectorAll("#studio-mit-tbody tr").forEach(tr => {
      tr.querySelector(".mit-row-kp").value = "0.0";
    });
  });
  document.querySelector("#studio-mit-send-btn").addEventListener("click", async () => {
    try {
      const pkt = readStudioMitPacket();
      const res = await window.PiperControls.requestArm(studioIndex, "mit/command", pkt);
      setStudioStatus(res.message);
    } catch (err) {
      setStudioStatus(`MIT 指令失败：${err.message}`, true);
    }
  });

  document.querySelector("#teaching-start-btn")?.addEventListener("click", async () => {
    try {
      const res = await requestTeaching("start");
      setTeachingFeedback(res.message);
      setStudioStatus(res.message);
    } catch (err) {
      setTeachingFeedback(`启动失败：${err.message}`, true);
    }
  });
  document.querySelector("#teaching-stop-btn")?.addEventListener("click", async () => {
    try {
      const res = await requestTeaching("stop");
      setTeachingFeedback(res.message);
      setStudioStatus(res.message);
    } catch (err) {
      setTeachingFeedback(`停止失败：${err.message}`, true);
    }
  });
  document.querySelector("#teaching-mode-select")?.addEventListener("change", async e => {
    try {
      const res = await requestTeaching("mode", {mode: e.target.value});
      setTeachingFeedback(res.message);
    } catch (err) {
      setTeachingFeedback(`模式切换失败：${err.message}`, true);
    }
  });
}

function renderStudio(snapshot, backendOnline) {
  const dialog = document.querySelector("#arm-studio");
  if (!dialog?.open) return;
  const arm = snapshot.arms[studioIndex];
  if (!arm) return;
  if (studioInitializedArm !== studioIndex && arm.joints) {
    syncStudioInputsFromArm(arm);
    studioInitializedArm = studioIndex;
  }
  document.querySelectorAll(".studio-arm-tab").forEach((btn, idx) => {
    const cfgName = snapshot.arms[idx]?.control?.configuration?.name || `机械臂 ${idx + 1}`;
    btn.textContent = `0${idx + 1} ${cfgName}`;
    btn.classList.toggle("active", idx === studioIndex);
  });
  const cfg = arm.control?.configuration;
  const status = arm.status;
  const pState = actualPowerState(arm, backendOnline);
  const pLabels = {enabled: "六轴已使能", disabled: "六轴已失能", mixed: "部分使能", unknown: "使能未知"};
  document.querySelector("#studio-conn-pill").textContent = CONNECTIONS[backendOnline ? arm.connection : "offline"];
  document.querySelector("#studio-power-pill").textContent = pLabels[pState];
  document.querySelector("#studio-mode-pill").textContent = `驱动模式：${arm.control?.mode || "—"} | 固件：${status ? codeLabel(CONTROL_MODES, status.ctrl_mode) : "—"}`;
  document.querySelector("#studio-gripper-pill").textContent = `开度：${number(arm.joints?.gripper_m == null ? null : arm.joints.gripper_m * 1000, 1)} mm`;

  const action = nextPowerAction(arm, studioIndex, backendOnline);
  const pBtn = document.querySelector("#studio-power-btn");
  pBtn.dataset.action = action;
  pBtn.textContent = `${action === "enable" ? "使能" : "失能"}机械臂 ${studioIndex + 1}`;
  pBtn.disabled = cards[studioIndex]?.querySelector(".power-button")?.disabled ?? true;

  document.querySelector("#studio-hud-title").textContent = cfg
    ? `${cfg.name} (${arm.namespace} · ${cfg.can_port})`
    : `机械臂 0${studioIndex + 1} (${arm.namespace})`;
  document.querySelector("#studio-hud-pose").textContent = arm.pose
    ? `XYZ: [${arm.pose.position_mm.map(v => v.toFixed(1)).join(", ")}] mm · RPY: [${arm.pose.rpy_deg.map(v => v.toFixed(1)).join(", ")}]°`
    : "法兰位姿：等待 end_pose_stamped";
  const studioWrenchReady = Boolean(backendOnline && arm.control_fresh && arm.control?.mode === "mit" && arm.control?.mit_ready);
  const wrenchArr = (studioWrenchReady && Array.isArray(arm.control?.end_wrench)) ? arm.control.end_wrench : null;
  const fNorm = wrenchArr && wrenchArr.length >= 3 && wrenchArr.slice(0, 3).every(Number.isFinite)
    ? Math.hypot(wrenchArr[0], wrenchArr[1], wrenchArr[2]) : null;
  const mNorm = wrenchArr && wrenchArr.length >= 6 && wrenchArr.slice(3, 6).every(Number.isFinite)
    ? Math.hypot(wrenchArr[3], wrenchArr[4], wrenchArr[5]) : null;
  const gForce = Number.isFinite(arm.control?.gripper_force) ? arm.control.gripper_force : null;
  const wrenchPill = document.querySelector("#studio-wrench-pill");
  if (wrenchPill) {
    wrenchPill.textContent = (fNorm !== null && mNorm !== null)
      ? `末端 |F| ${fNorm.toFixed(1)} N · |M| ${mNorm.toFixed(2)} N·m`
      : "末端 |F| — N · |M| — N·m";
  }
  const studioRates = (backendOnline && arm.control_fresh && arm.control?.rates) ? arm.control.rates : null;
  const ratesPill = document.querySelector("#studio-rates-pill");
  if (ratesPill) {
    ratesPill.textContent = studioRates
      ? `主环 ${number(studioRates.loop_hz, 0)} Hz · 补偿 ${number(studioRates.comp_hz, 0)} Hz · 下发 ${number(studioRates.cmd_tx_hz, 0)} Hz · 输入 ${number(studioRates.ros_cmd_hz, 0)} Hz`
      : "主环 — Hz · 补偿 — Hz · 下发 — Hz · 输入 — Hz";
  }
  const targetRateEl = document.querySelector("#studio-target-rate");
  if (targetRateEl) {
    targetRateEl.textContent = studioRates?.target_hz
      ? `驱动目标 ${studioRates.target_hz} Hz · 示教 200 Hz · 上层 50 Hz`
      : "驱动 200 Hz · 示教 200 Hz · 上层 50 Hz";
  }
  document.querySelectorAll("#studio-layer-rates .can-age-item").forEach(item => {
    const key = item.dataset.rateKey;
    const val = studioRates?.[key];
    const active = Number.isFinite(val) && val > 0;
    item.classList.toggle("ok", active);
    item.querySelector("strong").textContent = Number.isFinite(val) ? `${val.toFixed(1)} Hz` : "—";
  });
  const hudWrench = document.querySelector("#studio-hud-wrench");
  if (hudWrench) {
    hudWrench.textContent = (wrenchArr && fNorm !== null && mNorm !== null)
      ? `末端力 F: [${wrenchArr.slice(0, 3).map(v => v.toFixed(1)).join(", ")}] N (|F|=${fNorm.toFixed(1)} N) · 力矩 M: [${wrenchArr.slice(3, 6).map(v => v.toFixed(2)).join(", ")}] N·m`
      : "末端受载 F / M：等待 MIT 状态";
  }
  document.querySelector("#studio-hud-effector").textContent = cfg
    ? `末端：${cfg.effector.label} (${cfg.effector.mass_kg} kg) · 重力系数 ${(arm.control.gravity_scale_applied * 100).toFixed(0)}%`
    : "末端配置：等待驱动配置";
  const camCalib = arm.camera_calibration || null;
  const hudCam = document.querySelector("#studio-hud-camera");
  if (hudCam) {
    if (camCalib && Array.isArray(camCalib.translation) && Array.isArray(camCalib.rpy)) {
      const tMm = camCalib.translation.map(v => (v * 1000).toFixed(1));
      const rDeg = camCalib.rpy.map(v => toDegrees(v).toFixed(1));
      const intr0 = Array.isArray(camCalib.intrinsics) && camCalib.intrinsics.length ? camCalib.intrinsics[0] : null;
      const intrText = intr0 ? ` · 内参 ${intr0.width}×${intr0.height} (RMS=${intr0.rms}px, FOV=${intr0.fov_deg?.join("°×")}°)` : "";
      hudCam.textContent = `腕部相机 ${camCalib.camera} 相对 ${camCalib.parent}：XYZ=[${tMm.join(", ")}] mm · RPY=[${rDeg.join(", ")}]°${intrText}`;
    } else {
      hudCam.textContent = "腕部相机：未配置或尚未保存外参";
    }
  }
  const camStampEl = document.querySelector("#studio-cam-calib-stamp");
  const camBodyEl = document.querySelector("#studio-cam-calib-body");
  if (camStampEl && camBodyEl) {
    if (camCalib && Array.isArray(camCalib.translation) && Array.isArray(camCalib.rpy)) {
      const tMm = camCalib.translation.map(v => (v * 1000).toFixed(2));
      const rDeg = camCalib.rpy.map(v => toDegrees(v).toFixed(2));
      const origins = camCalib.mesh_origins || {};
      const wf = origins.gripper_base_with_flange ? origins.gripper_base_with_flange.map(v => (v * 1000).toFixed(2)).join(", ") : "—";
      const wof = origins.gripper_base_without_flange ? origins.gripper_base_without_flange.map(v => (v * 1000).toFixed(2)).join(", ") : "—";
      const l6w = origins.link6_with_wrench ? origins.link6_with_wrench.map(v => (v * 1000).toFixed(2)).join(", ") : "—";
      const intr0 = Array.isArray(camCalib.intrinsics) && camCalib.intrinsics.length ? camCalib.intrinsics[0] : null;
      const k = intr0?.camera_matrix || [];
      camStampEl.textContent = `${camCalib.camera} · ${camCalib.method} (${camCalib.samples} 组, 残差 ${camCalib.residual_mm ?? "—"} mm) · ${camCalib.stamp || ""}`;
      camBodyEl.innerHTML =
        `<div><b>外参 (${camCalib.parent} → ${camCalib.camera})</b>：XYZ = [${tMm.join(", ")}] mm · RPY = [${rDeg.join(", ")}]° · Quat = [${(camCalib.rotation || []).map(v => v.toFixed(4)).join(", ")}]</div>` +
        `<div><b>折算至各原点</b>：gripper_base_without_flange.STL (+52.5mm) = [${wof}] mm · link6 (+92.8mm 含力传感器) = [${l6w}] mm · gripper_base.STL (+63.0mm) = [${wf}] mm</div>` +
        (intr0 ? `<div><b>内参 (${intr0.width}×${intr0.height})</b>：fx=${Number(k[0]).toFixed(2)}, fy=${Number(k[4]).toFixed(2)}, cx=${Number(k[2]).toFixed(2)}, cy=${Number(k[5]).toFixed(2)} · RMS=${intr0.rms} px · FOV=${(intr0.fov_deg || []).join("°×")}°</div>` : "");
    } else {
      camStampEl.textContent = "未标定";
      camBodyEl.textContent = "当前机械臂暂无已保存的腕部相机标定结果。";
    }
  }

  if (studioViewer) {
    const qRad = arm.joints?.position_deg?.map(v => Number.isFinite(v) ? toRadians(v) : 0) || [0, 0, 0, 0, 0, 0];
    const trackedRad = window.PiperControls.trackedPosition(studioIndex);
    studioViewer.update({
      joints: qRad,
      ghostJoints: studioGhostDeg ? studioGhostDeg.map(toRadians) : trackedRad,
      targetPose: studioTargetPose,
      wrench: wrenchArr,
      gripper: arm.joints?.gripper_m ?? 0.02,
      effector: cfg?.end_effector || (studioIndex < 2 ? "teaching_handle" : "gripper_with_wrench"),
      cameraCalib: camCalib,
    });
  }
  const trackingActive = window.PiperControls.isMitTracking(studioIndex);
  document.querySelectorAll(".mit-free-btn").forEach(btn => btn.classList.toggle("active", !trackingActive));
  document.querySelectorAll(".mit-track-btn").forEach(btn => btn.classList.toggle("active", trackingActive));
  if (!trackingActive && !studioGhostDeg && arm.joints?.position_deg?.every(Number.isFinite)) {
    const activeEl = document.activeElement;
    const editingTarget = activeEl && (activeEl.classList.contains("mit-row-pos") || activeEl.classList.contains("mit-row-slider") || activeEl.closest(".studio-joint-row"));
    if (!editingTarget) {
      arm.joints.position_deg.forEach((deg, j) => setStudioJointTargetInputs(j, deg));
    }
  }

  const drvMode = arm.control?.mode || "standby";
  document.querySelector("#studio-card-joints").dataset.recommended = String(["joint", "mit"].includes(drvMode));
  document.querySelector("#studio-card-cartesian").dataset.recommended = String(["point", "linear"].includes(drvMode));

  document.querySelectorAll(".studio-joint-row").forEach((row, j) => {
    const meas = arm.joints?.position_deg?.[j];
    const targetVal = Number(row.querySelector('input[type="number"]').value);
    row.querySelector(".j-meas").textContent = Number.isFinite(meas) ? `${meas.toFixed(1)}°` : "—";
    const deltaEl = row.querySelector(".j-delta");
    if (Number.isFinite(meas) && Number.isFinite(targetVal)) {
      const diff = targetVal - meas;
      deltaEl.textContent = `Δ${diff >= 0 ? "+" : ""}${diff.toFixed(1)}°`;
      deltaEl.classList.toggle("changed", Math.abs(diff) >= 0.2);
    } else {
      deltaEl.textContent = "Δ—";
      deltaEl.classList.remove("changed");
    }
  });

  if (arm.pose) {
    const measPose = [...arm.pose.position_mm, ...arm.pose.rpy_deg];
    document.querySelectorAll("#studio-cart-inputs input").forEach((inp, idx) => {
      const diff = Number(inp.value) - measPose[idx];
      const em = document.querySelector(`em[data-cart-delta="${idx}"]`);
      if (em && Number.isFinite(diff)) {
        em.textContent = `Δ${diff >= 0 ? "+" : ""}${diff.toFixed(1)}${idx < 3 ? "" : "°"}`;
      }
    });
  }

  document.querySelector("#studio-mode-detail").textContent = status
    ? `ctrl_mode=${status.ctrl_mode} · mode_feedback=${status.mode_feedback} · teach_status=${status.teach_status}`
    : "等待状态反馈";
  document.querySelectorAll("#studio-mode-buttons button").forEach(btn => {
    btn.classList.toggle("active", arm.control?.mode === btn.dataset.mode);
  });

  const calReason = window.PiperControls.getCalibrationBlock(studioIndex);
  const calCheck = document.querySelector("#studio-cal-confirm-check");
  if (calReason) calCheck.checked = false;
  document.querySelector("#studio-calibrate-btn").disabled = Boolean(calReason || !calCheck.checked);
  document.querySelector("#studio-cal-reason").textContent = calReason
    ? `当前不可标零：${calReason}`
    : "满足标零条件：确认末端完全闭合后勾选上方复选框即可写入零点";

  if (cfg) {
    const sel = document.querySelector("#studio-effector-select");
    const sig = `${studioIndex}:${JSON.stringify(cfg)}`;
    if (studioConfigSig !== sig) {
      studioConfigSig = sig;
      sel.replaceChildren(...Object.entries(cfg.effectors).map(([k, prof]) => {
        const opt = document.createElement("option");
        opt.value = k;
        opt.textContent = prof.label;
        return opt;
      }));
      sel.value = cfg.end_effector;
      const scaleInput = document.querySelector("#studio-gravity-scale");
      scaleInput.value = String(cfg.gravity_scale);
      document.querySelector("#studio-gravity-output").textContent = `${Math.round(cfg.gravity_scale * 100)}%`;
    }
    document.querySelector("#studio-gravity-summary").textContent =
      `生效系数 ${arm.control.gravity_scale_applied.toFixed(2)} · 补偿力矩 [${arm.control.gravity_torque.map(v => v.toFixed(2)).join(", ")}] Nm`;
  }

  document.querySelectorAll("#studio-mit-tbody tr").forEach((tr, j) => {
    const meas = arm.joints?.position_deg?.[j];
    tr.querySelector(".mit-meas-cell").textContent = Number.isFinite(meas) ? `${meas.toFixed(1)}°` : "—";
    const gVal = arm.control?.gravity_torque?.[j];
    tr.querySelector(".mit-g-cell").textContent = Number.isFinite(gVal) ? gVal.toFixed(3) : "—";
    const extVal = studioWrenchReady ? arm.control?.external_torque?.[j] : null;
    const extCell = tr.querySelector(".mit-ext-cell");
    if (extCell) extCell.textContent = Number.isFinite(extVal) ? extVal.toFixed(3) : "—";
  });

  const wrenchSummaryEl = document.querySelector("#studio-wrench-summary");
  if (wrenchSummaryEl) {
    wrenchSummaryEl.textContent = (fNorm !== null && mNorm !== null)
      ? `合力 |F| = ${fNorm.toFixed(2)} N · 合力矩 |M| = ${mNorm.toFixed(3)} N·m · 夹爪力 ${gForce !== null ? gForce.toFixed(2) : "—"}`
      : "等待 MIT 周期力螺旋数据";
  }
  document.querySelectorAll("#studio-wrench-cards .studio-wrench-box").forEach((box, idx) => {
    renderWrenchBox(box, wrenchArr?.[idx], WRENCH_AXES[idx], wrenchHistories[studioIndex][idx], idx < 3 ? 2 : 3);
  });
  document.querySelectorAll("#studio-wrench-joint-tbody tr").forEach((tr, j) => {
    const deg = arm.joints?.position_deg?.[j];
    tr.querySelector(".wj-pos").textContent = Number.isFinite(deg) ? `${deg.toFixed(1)}°` : "—";
    tr.querySelector(".wj-ext").textContent = studioWrenchReady ? number(arm.control?.external_torque?.[j], 3) : "—";
    tr.querySelector(".wj-iner").textContent = studioWrenchReady ? number(arm.control?.inertia_torque?.[j], 3) : "—";
    tr.querySelector(".wj-grav").textContent = studioWrenchReady ? number(arm.control?.gravity_torque?.[j], 3) : "—";
    tr.querySelector(".wj-fric").textContent = studioWrenchReady ? number(arm.control?.friction_torque?.[j], 3) : "—";
    tr.querySelector(".wj-out").textContent = studioWrenchReady ? number(arm.control?.output_torque?.[j], 3) : "—";
  });

  document.querySelector("#studio-err-code").textContent = status
    ? `故障字：0x${status.err_code.toString(16).toUpperCase().padStart(4, "0")}${status.last_error ? ` · ${status.last_error}` : ""}`
    : "故障字：—";

  document.querySelectorAll("#studio-diag-tbody tr").forEach((tr, j) => {
    if (j < 6) {
      const en = status?.motor_enable_valid?.[j] ? (status.motor_enabled[j] ? "已使能" : "已失能") : "未知";
      const deg = arm.joints?.position_deg?.[j];
      const rad = Number.isFinite(deg) ? toRadians(deg) : null;
      tr.querySelector(".d-en").textContent = en;
      tr.querySelector(".d-pos").textContent = Number.isFinite(deg) ? `${deg.toFixed(2)}° (${rad.toFixed(3)})` : "—";
      tr.querySelector(".d-vel").textContent = number(arm.joints?.velocity_rad_s?.[j], 3);
      tr.querySelector(".d-eff").textContent = number(arm.joints?.effort_nm?.[j], 3);
      tr.querySelector(".d-lim").textContent = status ? (status.angle_limits[j] ? "⚠ 超限" : "正常") : "—";
      tr.querySelector(".d-com").textContent = status ? (status.communication_errors[j] ? "⚠ 异常" : "正常") : "—";
    } else {
      const en = status?.gripper_valid ? (status.gripper_enabled ? "已使能" : "零力/失能") : "未知";
      const mm = arm.joints?.gripper_m == null ? null : arm.joints.gripper_m * 1000;
      tr.querySelector(".d-en").textContent = en;
      tr.querySelector(".d-pos").textContent = Number.isFinite(mm) ? `${mm.toFixed(2)} mm` : "—";
      tr.querySelector(".d-vel").textContent = "—";
      tr.querySelector(".d-eff").textContent = "—";
      tr.querySelector(".d-lim").textContent = "—";
      tr.querySelector(".d-com").textContent = status?.gripper_valid ? "正常" : "未知";
    }
  });

  const ageItems = document.querySelectorAll("#studio-can-ages .can-age-item");
  ageItems.forEach((item, idx) => {
    const age = status?.feedback_age?.[idx];
    const valid = Number.isFinite(age) && age >= 0 && age <= 1.0;
    item.classList.toggle("ok", Boolean(valid));
    item.classList.toggle("stale", status != null && !valid);
    item.querySelector("strong").textContent = Number.isFinite(age) && age >= 0 ? `${(age * 1000).toFixed(1)} ms` : "缺失";
  });

  const drvErr = arm.control?.last_error || status?.last_error || "";
  const errBadge = document.querySelector("#studio-last-error-badge");
  if (errBadge) {
    errBadge.textContent = drvErr ? `最近报错：${drvErr}` : "驱动无报错";
  }
  if (drvErr && drvErr !== studioLastSeenError) {
    studioLastSeenError = drvErr;
    setStudioStatus(`驱动报错：${drvErr}`, true);
  } else if (!drvErr) {
    studioLastSeenError = "";
  }

  const logList = document.querySelector("#studio-log-list");
  if (logList) {
    const logs = Array.isArray(snapshot.logs) ? snapshot.logs : [];
    const lines = logs.slice(-25).reverse();
    if (drvErr && !lines.some(l => l.msg.includes(drvErr))) {
      lines.unshift({time: "最新", level: "ERROR", node: `${arm.namespace}.driver`, msg: drvErr});
    }
    logList.replaceChildren(...(lines.length ? lines.map(entry => {
      const div = document.createElement("div");
      div.className = `log-entry ${entry.level}`;
      div.textContent = `[${entry.time}] [${entry.level}] [${entry.node}]: ${entry.msg}`;
      return div;
    }) : [Object.assign(document.createElement("div"), {className: "log-entry", textContent: "暂无 ROS 节点日志输出"})]));
  }
}

function reportConnection(online, error = "") {
  document.querySelector("#transport-state").textContent = online ? "已连接" : "连接中断";
  document.querySelector("#transport-dot").className = `transport-dot ${online ? "online" : "offline"}`;
  const notice = document.querySelector("#notice");
  notice.classList.toggle("error", !online);
  if (!online) {
    for (const card of cards) {
      card.dataset.connection = "offline";
      card.querySelector(".connection-badge b").textContent = CONNECTIONS.offline;
      card.querySelector(".power-button").disabled = true;
    }
    notice.hidden = false;
    notice.textContent = `监控服务连接中断，正在自动重连。所有数值均为历史数据，不代表当前状态。${error ? `（${error}）` : ""}`;
  } else {
    const live = lastSnapshot.arms.filter(arm => arm.connection === "live").length;
    notice.hidden = live === 4;
    notice.textContent = live === 0
      ? "尚无完整 Piper 遥测。请检查 CAN 已激活，并启动 /arm_1～/arm_4 下的 ROS 驱动。仅连接硬件不会自动产生 ROS 话题；缺失值显示为 —，不按零值处理。"
      : `当前 ${live} / 4 台 ROS 遥测完整，其余设备存在缺失或过期话题。展开设备行内的“话题与时效”可查看详情。`;
  }
}

async function refresh() {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2500);
  try {
    const response = await fetch("/api/status", {signal: controller.signal, cache: "no-store"});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const snapshot = await response.json();
    if (!Array.isArray(snapshot.arms) || snapshot.arms.length !== 4) throw new Error("遥测响应格式无效");
    lastSnapshot = snapshot;
    lastSuccess = performance.now();
    render(snapshot, true);
    reportConnection(true);
  } catch (error) {
    if (lastSnapshot) render(lastSnapshot, false, false);
    reportConnection(false, error.name === "AbortError" ? "请求超时" : error.message);
  } finally {
    clearTimeout(timeout);
    setTimeout(refresh, 80);
  }
}

createCards();
initStudio();
setInterval(() => {
  document.querySelector("#clock").textContent = new Date().toLocaleTimeString("zh-CN", {hour12: false});
  if (lastSnapshot && performance.now() - lastSuccess > 2500) {
    render(lastSnapshot, false, false);
    reportConnection(false, "超过 2.5 秒未收到服务响应");
  }
}, 500);
refresh();
