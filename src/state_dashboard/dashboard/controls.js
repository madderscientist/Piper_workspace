"use strict";

window.PiperControls = (() => {
  const states = [];

  async function request(state, action, body) {
    const {snapshot, online} = state.context();
    if (!online || !snapshot?.control_token) throw new Error("服务断开，禁止发送指令");
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 3000);
    try {
      const response = await fetch(`/api/arms/${state.index + 1}/${action}`, {
        method: "POST",
        headers: {"Content-Type": "application/json", "X-Piper-Control-Token": snapshot.control_token},
        body: JSON.stringify(body), signal: controller.signal,
      });
      const result = await response.json();
      if (!response.ok || result.success !== true) throw new Error(result.message || `HTTP ${response.status}`);
      return result;
    } finally {
      clearTimeout(timer);
    }
  }

  function calibrationBlock(state) {
    const {snapshot, online} = state.context();
    const arm = snapshot?.arms[state.index];
    if (!online || !arm?.control_fresh || !arm.streams.joints.fresh ||
        !Number.isFinite(arm.joints?.gripper_m)) return "控制状态或末端反馈缺失、过期";
    if (!Object.hasOwn(arm.control, "calibration_blocked_reason")) return "驱动尚未提供标定接口，请更新并重启驱动";
    if (arm.enable?.pending || arm.disable?.pending) return "请等待使能/失能请求结束";
    return arm.control.calibration_blocked_reason;
  }

  function bind(card, index, context) {
    const root = card.querySelector(".configuration-controls");
    let savedTracking = false;
    try { savedTracking = sessionStorage.getItem(`piper_mit_tracking_${index}`) === "1"; } catch (_) {}
    const state = {
      root, index, context,
      tracking: savedTracking,
      desired: [0, 0, 0, 0, 0, 0],
      sending: false, pending: false,
    };
    states.push(state);
  }

  function mitPacket(position) {
    return {
      position,
      velocity: [0, 0, 0, 0, 0, 0],
      kp: [-1, -1, -1, -1, -1, -1],
      kd: [-1, -1, -1, -1, -1, -1],
      torque: [0, 0, 0, 0, 0, 0],
    };
  }

  function render(arm, index, online) {
    const state = states[index];
    const root = state.root;
    const fresh = online && arm.control_fresh;
    const control = arm.control;
    const config = control?.configuration;
    if (state.tracking && fresh && control.mode !== "mit" && !state.pending && !state.sending) {
      state.tracking = false;
      try { sessionStorage.removeItem(`piper_mit_tracking_${index}`); } catch (_) {}
    }
    if (!state.tracking && fresh && Array.isArray(control?.mit_target) && control.mit_target.length === 6) {
      state.desired = [...control.mit_target];
    } else if (!state.tracking && !state.pending && arm.joints?.position_deg?.every(Number.isFinite)) {
      state.desired = arm.joints.position_deg.map(v => v * Math.PI / 180);
    }
    if (!config) return;
    cardTitle(arm, index, config);
    const extEl = root.querySelector(".outer-ext-torques");
    if (extEl) {
      extEl.textContent = fresh && control.mode === "mit" && control.mit_ready && Array.isArray(control.external_torque)
        ? `关节受载外力矩 Nm：[${control.external_torque.map(v => v.toFixed(2)).join(", ")}]`
        : "关节受载外力矩 Nm：等待 MIT 模式就绪";
    }
    const gEl = root.querySelector(".gravity-torques");
    if (gEl) {
      gEl.textContent = fresh && control.mode === "mit"
        ? `重力系数 ${control.gravity_scale_applied.toFixed(2)} · 重力补偿 Nm：[${control.gravity_torque.map(v => v.toFixed(2)).join(", ")}]`
        : "非 MIT 模式：不计算、不输出软件重力补偿";
    }
    const errEl = root.querySelector(".driver-error");
    if (errEl) errEl.textContent = control.last_error ? `驱动最近错误：${control.last_error}` : "";
  }

  function cardTitle(arm, index, config) {
    const card = states[index].root.closest(".arm-row");
    card.querySelector("h2").textContent = config.name;
    card.querySelector(".namespace").textContent = `${arm.namespace} · ${config.can_port} · ${config.effector.label}`;
  }

  setInterval(() => {
    for (const state of states) {
      if (!state.tracking || state.sending) continue;
      state.sending = true;
      request(state, "mit/command", mitPacket(state.desired))
        .catch(() => {})
        .finally(() => { state.sending = false; });
    }
  }, 60);

  async function startTracking(index) {
    const state = states[index];
    if (!state || state.pending) return null;
    if (state.tracking) return state.desired;
    state.pending = true;
    try {
      const arm = state.context().snapshot?.arms[index];
      if (arm?.joints?.position_deg?.every(Number.isFinite)) {
        state.desired = arm.joints.position_deg.map(v => v * Math.PI / 180);
      }
      await request(state, "mit/command", mitPacket(state.desired));
      if (arm?.control?.mode !== "mit") {
        await request(state, "mode", {mode: "mit"});
      }
      state.tracking = true;
      try { sessionStorage.setItem(`piper_mit_tracking_${index}`, "1"); } catch (_) {}
      return [...state.desired];
    } finally {
      state.pending = false;
    }
  }

  async function stopTracking(index) {
    const state = states[index];
    if (!state) return;
    state.tracking = false;
    try { sessionStorage.removeItem(`piper_mit_tracking_${index}`); } catch (_) {}
    const arm = state.context().snapshot?.arms[index];
    if (arm?.control?.mode === "mit") {
      await request(state, "mode", {mode: "mit"});
    }
  }

  function setDesiredJoint(index, joint, rad, push = false) {
    const state = states[index];
    if (!state) return;
    state.desired[joint] = rad;
    if (push && state.tracking && !state.sending) {
      state.sending = true;
      request(state, "mit/command", mitPacket(state.desired))
        .catch(() => {})
        .finally(() => { state.sending = false; });
    }
  }

  function isMitTracking(index) {
    const state = states[index];
    return Boolean(state?.tracking);
  }

  function trackedPosition(index) {
    const state = states[index];
    const arm = state?.context().snapshot?.arms[index];
    if (Array.isArray(arm?.control?.mit_target) && arm.control.mit_target.length === 6) {
      return arm.control.mit_target;
    }
    return state?.tracking ? state.desired : null;
  }

  async function requestArm(index, action, body) {
    return request(states[index], action, body);
  }

  function getCalibrationBlock(index) {
    return states[index] ? calibrationBlock(states[index]) : "未初始化";
  }

  return {
    bind, render, requestArm, getCalibrationBlock,
    startTracking, stopTracking, setDesiredJoint, isMitTracking, trackedPosition,
  };
})();
