(() => {
  const $ = (id) => document.getElementById(id);
  const MODE_LABELS = {
    idle: "空闲",
    ready: "已获取待执行",
    requesting: "云端推理中",
    batch_running: "整批按频执行中",
    auto_running: "自动连续闭环中",
    async_running: "异步重叠执行中",
    homing: "回起始位中",
    error: "异常",
  };

  let settingsInitialized = false;
  let tcpInputsInitialized = false;
  let lastOverlayKey = "";
  let modeRevision = 0;
  let switchingMode = false;
  let activeMode = "step";
  let pendingAction = null;
  let backendMode = "idle";
  const camCards = new Map();
  const EXECUTION_MODES = {
    step: { label: "执行一步", path: "/api/execute/step" },
    batch: { label: "执行整批", path: "/api/execute/batch" },
    auto: { label: "启动串行闭环", path: "/api/execute/auto" },
    async: { label: "启动异步重叠", path: "/api/execute/async" },
  };

  function selectedMode() {
    return document.querySelector('input[name="execution-mode"]:checked').value;
  }

  function renderExecutionMode() {
    const mode = selectedMode();
    document.querySelectorAll("[data-for-modes]").forEach((el) => {
      el.hidden = !el.dataset.forModes.split(" ").includes(mode);
    });
    renderActionState();
  }

  function renderActionState() {
    const running = ["requesting", "batch_running", "auto_running", "async_running", "homing"].includes(backendMode);
    const busy = switchingMode || pendingAction !== null || running;
    for (const id of ["btn-request", "btn-execute"]) {
      $(id).disabled = busy;
      $(id).classList.toggle("is-busy", busy);
      $(id).setAttribute("aria-busy", String(busy));
    }
    $("btn-request").textContent = pendingAction?.kind === "request" ? "请求中..." : "请求";
    $("btn-execute").textContent = pendingAction?.kind === "execute" || running
      ? "执行中..." : EXECUTION_MODES[selectedMode()].label;
  }

  async function switchExecutionMode() {
    modeRevision += 1;
    pendingAction = null;
    renderExecutionMode();
    renderChunk(null, 0);
    $("step-progress").textContent = "0 / 0";
    $("auto-cycles-tag").textContent = "0 轮";
    $("async-tag").textContent = "待命";
    $("inp-use-cached").checked = false;
    $("log-list").replaceChildren();
    if (switchingMode) return;
    switchingMode = true;
    renderActionState();
    $("mode-badge").textContent = "切换中";
    $("status-msg").textContent = "正在停止当前执行并清空结果...";
    try {
      await api("/api/stop", { release_mit: true, clear_results: true });
      activeMode = selectedMode();
    } catch (err) {
      document.querySelector(`input[name="execution-mode"][value="${activeMode}"]`).checked = true;
      renderExecutionMode();
      alert("模式切换失败：" + err.message);
    } finally {
      switchingMode = false;
      backendMode = "idle";
      await poll();
      renderActionState();
    }
  }

  async function api(path, body = null) {
    const opts = body === null
      ? { method: "GET", cache: "no-store" }
      : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
    const res = await fetch(path, opts);
    const data = await res.json().catch(() => ({ ok: false, error: `HTTP ${res.status}` }));
    if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
    return data;
  }

  function parseVec3(str) {
    const nums = str.split(",").map((s) => Number(s.trim()));
    if (nums.length !== 3 || nums.some((n) => !Number.isFinite(n))) {
      throw new Error("请输入 3 个逗号分隔的数字，例如 0, 0, 0.12");
    }
    return nums;
  }

  function renderCalibration(cal) {
    $("calib-file").textContent = cal.file || "—";
    $("calib-id").textContent = cal.calibration_id || "—";
    $("calib-time").textContent = cal.updated_at || "尚未保存（使用默认初值）";
    const badge = $("calib-badge");
    badge.textContent = cal.loaded_from_disk ? `已加载落盘默认 (${cal.calibration_id})` : "默认初值（未落盘）";
    badge.className = "badge " + (cal.loaded_from_disk ? "ok" : "");

    if (!tcpInputsInitialized) {
      $("world-origin").value = cal.world_origin || "dual_base_board";
      $("base-mid-x").value = cal.base_mid_x_m ?? 0.06;
      $("board-z").value = cal.board_z_m ?? 0.75;
      $("left-tcp-xyz").value = (cal.flange_from_tcp?.left?.xyz_m || [0, 0, 0]).join(", ");
      $("left-tcp-rpy").value = (cal.flange_from_tcp?.left?.rpy_deg || [0, 0, 0]).join(", ");
      $("right-tcp-xyz").value = (cal.flange_from_tcp?.right?.xyz_m || [0, 0, 0]).join(", ");
      $("right-tcp-rpy").value = (cal.flange_from_tcp?.right?.rpy_deg || [0, 0, 0]).join(", ");
      tcpInputsInitialized = true;
    }

    const lines = [];
    for (const side of ["left", "right"]) {
      const wb = cal.world_from_base?.[side];
      const obs = cal.board_observations?.[side];
      lines.push(`${side.toUpperCase()} world<-base: xyz=[${(wb?.xyz_m || []).join(", ")}] m, rpy=[${(wb?.rpy_deg || []).join(", ")}] deg` +
        (obs ? `  (角点 ${obs.corner_count}, RMS ${obs.reproj_rms_px}px)` : ""));
    }
    if (cal.world_from_head) {
      const obsH = cal.board_observations?.head;
      lines.push(`HEAD world<-cam:  xyz=[${cal.world_from_head.xyz_m.join(", ")}] m, rpy=[${cal.world_from_head.rpy_deg.join(", ")}] deg` +
        (obsH ? `  (角点 ${obsH.corner_count}, RMS ${obsH.reproj_rms_px}px)` : ""));
    } else {
      lines.push("HEAD world<-cam:  未标定 (null)");
    }
    $("calib-transforms").textContent = lines.join("\n");

    const overlayKey = (cal.has_overlays || []).join(",") + "@" + (cal.updated_at || "");
    if (overlayKey !== lastOverlayKey) {
      lastOverlayKey = overlayKey;
      const strip = $("calib-overlays");
      strip.innerHTML = (cal.has_overlays || []).map(
        (k) => `<img src="/api/calibration/overlay/${k}.jpg?t=${Date.now()}" alt="${k} ChArUco">`,
      ).join("");
    }

    const homes = cal.home_poses || {};
    const homeKeys = Object.keys(homes);
    $("home-pose-box").textContent = homeKeys.length === 0
      ? "尚未落盘起始关节位置"
      : homeKeys.map((aid) => {
          const h = homes[aid];
          return `${aid} (${h.updated_at}): joints=[${h.joints_rad.map((v) => v.toFixed(4)).join(", ")}] rad | grip=${h.gripper_m.toFixed(4)} m`;
        }).join("\n");
  }

  function renderCameras(cameras) {
    const grid = $("camera-grid");
    for (const cam of cameras) {
      let card = camCards.get(cam.id);
      if (!card) {
        const el = document.createElement("div");
        el.className = "cam-card";
        el.innerHTML = `
          <div class="cam-head"><strong></strong><span class="badge"></span></div>
          <div class="cam-view"><img alt=""></div>
          <div class="cam-foot"><span class="dim"></span><span class="age"></span></div>`;
        grid.appendChild(el);
        card = {
          el,
          title: el.querySelector("strong"),
          badge: el.querySelector(".badge"),
          img: el.querySelector("img"),
          dim: el.querySelector(".dim"),
          age: el.querySelector(".age"),
          mode: "",
        };
        camCards.set(cam.id, card);
      }
      card.title.textContent = `${cam.label} (${cam.role})`;
      card.badge.textContent = cam.online ? (cam.calibrated_intrinsics ? "实时 · 内参已加载" : "实时 · 无内参") : "等待画面";
      card.badge.className = "badge " + (cam.online ? "ok" : "err");
      card.dim.textContent = cam.width ? `${cam.width}×${cam.height} · #${cam.sequence}` : (cam.error || "未连接");
      card.age.textContent = cam.age_ms !== null ? `${cam.age_ms.toFixed(0)} ms` : "—";

      if (cam.online && card.mode !== "live") {
        card.img.src = `/api/cameras/${cam.id}/stream.mjpg?t=${Date.now()}`;
        card.mode = "live";
      }
    }
  }

  function renderArms(arms) {
    const grid = $("arm-grid");
    grid.innerHTML = arms.map((arm) => {
      const joints = arm.joints_rad
        ? arm.joints_rad.map((q, i) => `<div class="joint-pill"><small>J${i + 1}</small>${q.toFixed(3)}<small>${(q * 180 / Math.PI).toFixed(1)}°</small></div>`).join("")
        : `<div class="muted">等待 ${arm.topic} ...</div>`;
      const grip = arm.gripper_m !== null && arm.gripper_m !== undefined
        ? `<div class="joint-pill"><small>夹爪(m)</small>${arm.gripper_m.toFixed(4)}<small>${(arm.gripper_m * 1000).toFixed(1)}mm</small></div>`
        : "";
      const sent = arm.last_sent
        ? `最近 MIT 下发 (#${arm.last_sent.tx_count}): [${arm.last_sent.joints_rad.map((v) => v.toFixed(3)).join(", ")}]`
        : "尚未下发 MIT 指令";
      return `
        <div class="arm-card">
          <div class="cam-head">
            <strong>${arm.label} · <code>${arm.mit_topic}</code></strong>
            <span class="badge ${arm.online ? "ok" : "err"}">${arm.online ? (arm.holding ? "在线 · MIT维持中" : "在线") : "离线"}</span>
          </div>
          <div class="joints-row">${joints}${grip}</div>
          <div class="cam-foot"><span>${sent}</span><span>${arm.age_ms !== null ? `反馈龄 ${arm.age_ms} ms` : ""}</span></div>
        </div>`;
    }).join("");
  }

  function renderChunk(chunk, cursor) {
    const wrap = $("chunk-table-wrap");
    if (!chunk) {
      $("chunk-meta").textContent = "";
      wrap.textContent = "暂无预测数据";
      return;
    }
    const ik = chunk.ik_summary;
    const ikText = ik ? ` · C++ IK ${ik.solve_ms}ms (最大残差 ${ik.max_pos_err_mm}mm)` : "";
    $("chunk-meta").textContent = `(${chunk.action_kind} · 云端 ${chunk.latency_ms}ms${ikText} · ID ${chunk.request_id.slice(0, 8)})`;
    const rows = chunk.steps.map((s, idx) => {
      const active = idx === cursor ? ' class="active-step"' : "";
      const fmtSide = (t) => {
        if (t.tcp_pose_world_xyzw && t.joint_positions_rad) {
          const tcp = t.tcp_pose_world_xyzw.slice(0, 3).map((v) => v.toFixed(3)).join(",");
          const q = t.joint_positions_rad.map((v) => v.toFixed(2)).join(",");
          const err = t.ik_pos_err_mm !== null && t.ik_pos_err_mm !== undefined ? ` (${t.ik_pos_err_mm.toFixed(2)}mm)` : "";
          return `xyz=[${tcp}] → q=[${q}] g=${t.gripper_opening_m ?? "—"}${err}`;
        }
        if (t.joint_positions_rad) return `q=[${t.joint_positions_rad.map((v) => v.toFixed(3)).join(",")}] g=${t.gripper_opening_m ?? "—"}`;
        if (t.tcp_pose_world_xyzw) return `xyz=[${t.tcp_pose_world_xyzw.slice(0, 3).map((v) => v.toFixed(3)).join(",")}] g=${t.gripper_opening_m ?? "—"}`;
        return "—";
      };
      return `<tr${active}><td>#${s.step_index}</td><td>${s.target_offset_s.toFixed(2)}s</td><td>${fmtSide(s.left)}</td><td>${fmtSide(s.right)}</td></tr>`;
    }).join("");
    wrap.innerHTML = `<table><thead><tr><th>步</th><th>t</th><th>Left</th><th>Right</th></tr></thead><tbody>${rows}</tbody></table>`;
  }

  async function poll() {
    if (switchingMode) return;
    const revision = modeRevision;
    try {
      const s = await api("/api/state");
      if (switchingMode || revision !== modeRevision) return;
      const badge = $("mode-badge");
      badge.dataset.mode = s.mode;
      badge.textContent = MODE_LABELS[s.mode] || s.mode;
      $("status-msg").textContent = s.message || "";
      backendMode = s.mode;
      renderActionState();

      if (!settingsInitialized) {
        $("inp-instruction").value = s.settings.instruction || "";
        $("inp-url").value = s.settings.server_url || "";
        $("inp-exec-hz").value = s.settings.execute_hz;
        $("inp-settle-s").value = s.settings.settle_delay_s;
        $("inp-hold-hz").value = s.settings.hold_hz;
        $("inp-gripper").checked = s.settings.enable_gripper;
        settingsInitialized = true;
      }
      $("step-progress").textContent = `${s.step_cursor} / ${s.total_steps}`;
      $("batch-rate-tag").textContent = `${s.settings.execute_hz} Hz`;
      $("auto-cycles-tag").textContent = `${s.auto_cycles} 轮`;
      if (s.async_stats && s.async_stats.chunks > 0) {
        const a = s.async_stats;
        $("async-tag").textContent = `t=${a.latency_ms}ms · #${a.start_index + 1}切入 · 混合${a.overlap_steps}步`;
      } else {
        $("async-tag").textContent = "待命";
      }

      renderCalibration(s.calibration);
      renderCameras(s.cameras);
      renderArms(s.arms);
      renderChunk(s.chunk, s.step_cursor);
      $("log-list").innerHTML = (s.logs || []).map(
        (l) => `<div class="log-item" data-level="${l.level}">[${l.time}] ${l.text}</div>`,
      ).join("");
    } catch (err) {
      if (!switchingMode && revision === modeRevision) {
        $("status-msg").textContent = "连接本地服务断开：" + err.message;
      }
    }
  }

  async function currentSettingsPayload() {
    const payload = {
      server_url: $("inp-url").value.trim(),
      instruction: $("inp-instruction").value,
      execute_hz: Number($("inp-exec-hz").value),
      settle_delay_s: Number($("inp-settle-s").value),
      hold_hz: Number($("inp-hold-hz").value),
      enable_gripper: $("inp-gripper").checked,
    };
    await api("/api/settings", payload);
    return payload;
  }

  async function runAction(fn) {
    const revision = modeRevision;
    try {
      await fn();
      await poll();
    } catch (err) {
      if (revision === modeRevision) alert(err.message);
      await poll();
    }
  }

  async function runExecution(kind, fn) {
    if (switchingMode || $("btn-execute").disabled) return;
    const revision = modeRevision;
    const action = { kind };
    pendingAction = action;
    renderActionState();
    try {
      await fn();
    } catch (err) {
      if (revision === modeRevision) alert(err.message);
    } finally {
      if (pendingAction === action) {
        pendingAction = null;
        await poll();
        renderActionState();
      }
    }
  }

  async function stopExecution(releaseMit) {
    modeRevision += 1;
    pendingAction = null;
    try {
      await api("/api/stop", { release_mit: releaseMit });
      backendMode = "idle";
    } catch (err) {
      alert(err.message);
    }
    await poll();
    renderActionState();
  }

  $("btn-save-settings").onclick = () => runAction(currentSettingsPayload);
  $("btn-simulator").onclick = () => window.open("/simulator", "piperx-simulator", "popup,width=1280,height=820,noopener");
  $("btn-request").onclick = () => runExecution("request", async () => {
    if (switchingMode) return;
    const revision = modeRevision;
    const p = await currentSettingsPayload();
    if (switchingMode || revision !== modeRevision) return;
    await api("/api/request", { instruction: p.instruction });
  });
  document.querySelectorAll('input[name="execution-mode"]').forEach((input) => {
    input.onchange = switchExecutionMode;
  });
  $("btn-execute").onclick = () => runExecution("execute", async () => {
    if (switchingMode) return;
    const revision = modeRevision;
    const mode = selectedMode();
    const fetchNew = !$("inp-use-cached").checked;
    const p = await currentSettingsPayload();
    if (switchingMode || revision !== modeRevision) return;
    const payload = { instruction: p.instruction };
    if (mode === "batch") payload.fetch_new = fetchNew;
    await api(EXECUTION_MODES[mode].path, payload);
  });
  $("btn-stop").onclick = () => stopExecution(true);
  $("btn-stop-hold").onclick = () => stopExecution(false);
  $("btn-health").onclick = () => runAction(async () => {
    await currentSettingsPayload();
    const r = await api("/api/server/health");
    $("health-text").textContent = "云端在线: " + JSON.stringify(r.health?.model?.variant || r.health?.status || r.health);
  });

  document.querySelectorAll("[data-calib]").forEach((btn) => {
    btn.onclick = () => runAction(() => {
      const t = btn.dataset.calib;
      const targets = t === "all" ? ["head", "left", "right"] : [t];
      return api("/api/calibration/charuco", { targets, world_origin: $("world-origin").value });
    });
  });
  $("btn-save-tcp").onclick = () => runAction(() => api("/api/calibration/params", {
    world_origin: $("world-origin").value,
    base_mid_x_m: Number($("base-mid-x").value),
    board_z_m: Number($("board-z").value),
    left_tcp: { xyz_m: parseVec3($("left-tcp-xyz").value), rpy_deg: parseVec3($("left-tcp-rpy").value) },
    right_tcp: { xyz_m: parseVec3($("right-tcp-xyz").value), rpy_deg: parseVec3($("right-tcp-rpy").value) },
  }));
  $("btn-record-home").onclick = () => runAction(() => api("/api/calibration/record_home", {}));
  $("btn-go-home").onclick = () => runAction(() => api("/api/calibration/go_home", {}));

  renderExecutionMode();
  setInterval(poll, 350);
  poll();
})();

