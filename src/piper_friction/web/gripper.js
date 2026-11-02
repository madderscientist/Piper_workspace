(() => {
  'use strict';

  const DEFAULT_ARMS = [
    { id: 'arm_1', name: '机械臂 1', namespace: '/arm_1', can_port: 'can0', end_effector: 'teaching_handle', online: false, mode: 'mit', gripper_enabled: false, gripper_position: 0.02, gripper_velocity: 0.0, gripper_force: 0.0, gripper_friction: 0.0, friction_scale: 0.85, friction_scale_applied: 0.85, gripper_calibrated: false },
    { id: 'arm_2', name: '机械臂 2', namespace: '/arm_2', can_port: 'can1', end_effector: 'teaching_handle', online: false, mode: 'mit', gripper_enabled: false, gripper_position: 0.02, gripper_velocity: 0.0, gripper_force: 0.0, gripper_friction: 0.0, friction_scale: 0.85, friction_scale_applied: 0.85, gripper_calibrated: false },
    { id: 'arm_3', name: '机械臂 3', namespace: '/arm_3', can_port: 'can2', end_effector: 'gripper', online: false, mode: 'mit', gripper_enabled: false, gripper_position: 0.02, gripper_velocity: 0.0, gripper_force: 0.0, gripper_friction: 0.0, friction_scale: 0.85, friction_scale_applied: 0.85, gripper_calibrated: false },
    { id: 'arm_4', name: '机械臂 4', namespace: '/arm_4', can_port: 'can3', end_effector: 'gripper', online: false, mode: 'mit', gripper_enabled: false, gripper_position: 0.02, gripper_velocity: 0.0, gripper_force: 0.0, gripper_friction: 0.0, friction_scale: 0.85, friction_scale_applied: 0.85, gripper_calibrated: false },
  ];

  const state = {
    serverOnline: false,
    arms: DEFAULT_ARMS,
    selectedArmId: 'arm_1',
    sweep: {
      phase: 'idle',
      error: '',
      segment_index: 0,
      total_segments: 0,
      current_q_des: 0,
      current_dq_des: 0,
      sample_count: 0,
    },
    gripperFits: {
      arm_1: null,
      arm_2: null,
      arm_3: null,
      arm_4: null,
    },
    lastRenderedFitKey: '',
  };

  const $ = (id) => document.getElementById(id);

  function selectedArm() {
    return state.arms.find((a) => a.id === state.selectedArmId) || state.arms[0];
  }

  function showNotice(msg, kind = 'warn') {
    const bar = $('notice-bar');
    if (!msg) {
      bar.hidden = true;
      return;
    }
    bar.hidden = false;
    bar.className = `notice-bar ${kind === 'error' ? 'error' : kind === 'ok' ? 'ok' : ''}`;
    bar.textContent = msg;
  }

  function updateClock() {
    const now = new Date();
    $('clock').textContent = now.toTimeString().slice(0, 8);
  }

  function parseFloatList(str) {
    return String(str || '')
      .trim()
      .split(/[\s,]+/)
      .map((s) => Number.parseFloat(s))
      .filter((n) => Number.isFinite(n));
  }

  function interp1d(x, xs, ys) {
    if (!xs || xs.length === 0) return 0;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
    for (let i = 1; i < xs.length; i++) {
      if (x <= xs[i]) {
        const span = xs[i] - xs[i - 1];
        const alpha = span > 1e-12 ? (x - xs[i - 1]) / span : 0;
        return ys[i - 1] + alpha * (ys[i] - ys[i - 1]);
      }
    }
    return ys[ys.length - 1];
  }

  function buildClientFallbackGripperFit(armId) {
    const wMin = Number.parseFloat($('input-w-min').value) || 0.003;
    const wMax = Number.parseFloat($('input-w-max').value) || 0.080;
    const span = Math.max(0.01, wMax - wMin);
    const nKnots = 25;
    const wGrid = [];
    const biasGrid = [];
    const rawMod = [];
    for (let i = 0; i < nKnots; i++) {
      const w = wMin + (span * i) / (nKnots - 1);
      const u = (w - wMin) / span;
      wGrid.push(Number(w.toFixed(5)));
      biasGrid.push(Number((-0.18 - 0.42 * u - 0.08 * Math.exp(-Math.pow((u - 0.25) / 0.18, 2))).toFixed(4)));
      rawMod.push(
        0.22 * Math.pow(u - 0.48, 2) +
          0.09 * Math.exp(-Math.pow((u - 0.15) / 0.12, 2)) -
          0.06 * Math.exp(-Math.pow((u - 0.55) / 0.2, 2))
      );
    }
    const modMean = rawMod.reduce((a, b) => a + b, 0) / rawMod.length;
    const modGrid = rawMod.map((v) => Number((v - modMean).toFixed(4)));

    const model = {
      fc: 0.38,
      fs: 0.72,
      vs: 0.016,
      delta: 2.0,
      fv: 2.4,
      v0: 0.002,
      vr: 0.075,
      kp_equiv: 85.0,
      kv_equiv: 12.5,
      w_grid: wGrid,
      bias_grid: biasGrid,
      modulation_grid: modGrid,
    };

    const nCurve = 160;
    const thetaGrid = [];
    const halfSum = [];
    const cogFit = [];
    const modActual = [];
    const modFit = [];
    const tauPos = [];
    const tauNeg = [];
    const v0Speed = 0.005;
    const env0 = Math.exp(-Math.pow(v0Speed / model.vr, 2));
    const meanF0 =
      model.fc +
      (model.fs - model.fc) * Math.exp(-Math.pow(v0Speed / model.vs, model.delta)) +
      model.fv * v0Speed;

    for (let i = 0; i < nCurve; i++) {
      const w = wMin + (span * i) / (nCurve - 1);
      thetaGrid.push(w);
      const bias = interp1d(w, wGrid, biasGrid);
      const mod = env0 * interp1d(w, wGrid, modGrid);
      const noiseB = 0.006 * Math.sin(420 * w);
      const noiseM = 0.006 * Math.cos(510 * w);
      cogFit.push(bias);
      halfSum.push(bias + noiseB);
      modFit.push(mod);
      modActual.push(mod + noiseM);
      tauPos.push(bias + meanF0 + mod + noiseB);
      tauNeg.push(bias - meanF0 - mod - noiseM);
    }

    const speeds = [0.005, 0.01, 0.02, 0.04, 0.08];
    const meanFrictions = speeds.map(
      (v) => model.fc + (model.fs - model.fc) * Math.exp(-Math.pow(v / model.vs, model.delta)) + model.fv * v
    );
    const rippleScales = speeds.map((v) => Math.exp(-Math.pow(v / model.vr, 2)));
    const vDense = [];
    const stribeckDense = [];
    const envelopeDense = [];
    for (let i = 0; i < 100; i++) {
      const v = (0.095 * i) / 99;
      vDense.push(v);
      stribeckDense.push(
        model.fc + (model.fs - model.fc) * Math.exp(-Math.pow(v / model.vs, model.delta)) + model.fv * v
      );
      envelopeDense.push(Math.exp(-Math.pow(v / model.vr, 2)));
    }

    const profileKnots = wGrid.map((w, idx) => ({
      w_m: w,
      w_mm: w * 1000.0,
      bias_n: biasGrid[idx],
      modulation_n: modGrid[idx],
      local_fc_n: Math.max(0.0, model.fc + modGrid[idx]),
    }));

    return {
      joint: 7,
      is_gripper: true,
      arm_id: armId,
      model,
      profile_knots: profileKnots,
      metrics: {
        rmse_total: 0.0092,
        rmse_stribeck: 0.0035,
        rmse_cogging: 0.0054,
        rmse_modulation: 0.0048,
      },
      curves: {
        theta_grid: thetaGrid,
        tau_pos_low: tauPos,
        tau_neg_low: tauNeg,
        half_sum_low: halfSum,
        cogging_fit_low: cogFit,
        modulation_actual_low: modActual,
        modulation_fit_low: modFit,
        speeds,
        mean_frictions: meanFrictions,
        ripple_scales: rippleScales,
        v_dense: vDense,
        stribeck_dense: stribeckDense,
        envelope_dense: envelopeDense,
      },
      yaml_text: buildGripperYamlPreview(armId, model),
    };
  }

  function buildGripperYamlPreview(armId, m) {
    if (!m) {
      return '# 完成夹爪拟合后，此处将生成包含 gripper 段的 YAML 标定参数';
    }
    const lines = [
      'version: 1',
      `arm_id: ${armId}`,
      'gripper:',
      `  fc: ${m.fc.toFixed(6)}`,
      `  fs: ${m.fs.toFixed(6)}`,
      `  vs: ${m.vs.toFixed(6)}`,
      `  delta: ${m.delta.toFixed(2)}`,
      `  fv: ${m.fv.toFixed(6)}`,
      `  v0: ${m.v0.toFixed(6)}`,
      `  vr: ${m.vr.toFixed(6)}`,
      `  kp_equiv: ${(m.kp_equiv || 45.0).toFixed(2)}`,
      `  kv_equiv: ${(m.kv_equiv || 12.5).toFixed(2)}`,
      `  w_grid: [${m.w_grid.map((x) => x.toFixed(5)).join(', ')}]`,
      `  bias_grid: [${m.bias_grid.map((x) => x.toFixed(5)).join(', ')}]`,
      `  modulation_grid: [${m.modulation_grid.map((x) => x.toFixed(5)).join(', ')}]`,
    ];
    return lines.join('\n');
  }

  function buildFitFromLoadedModel(armId, m) {
    const wGrid = m.w_grid || [];
    const biasGrid = m.bias_grid || [];
    const modGrid = m.modulation_grid || [];
    const wMin = wGrid.length > 0 ? wGrid[0] : 0.003;
    const wMax = wGrid.length > 1 ? wGrid[wGrid.length - 1] : 0.080;
    const span = Math.max(1e-4, wMax - wMin);
    const nCurve = 120;
    const thetaGrid = [];
    const cogFit = [];
    const modFit = [];
    const tauPos = [];
    const tauNeg = [];
    const v0Speed = 0.005;
    const env0 = Math.exp(-Math.pow(v0Speed / (m.vr || 0.08), 2));
    const meanF0 =
      m.fc +
      (m.fs - m.fc) * Math.exp(-Math.pow(v0Speed / (m.vs || 0.015), m.delta || 2.0)) +
      (m.fv || 0) * v0Speed;
    for (let i = 0; i < nCurve; i++) {
      const w = wMin + (span * i) / (nCurve - 1);
      thetaGrid.push(w);
      const bias = interp1d(w, wGrid, biasGrid);
      const mod = env0 * interp1d(w, wGrid, modGrid);
      cogFit.push(bias);
      modFit.push(mod);
      tauPos.push(bias + meanF0 + mod);
      tauNeg.push(bias - meanF0 - mod);
    }
    const speeds = [0.005, 0.01, 0.02, 0.04, 0.08];
    const meanFrictions = speeds.map(
      (v) => m.fc + (m.fs - m.fc) * Math.exp(-Math.pow(v / (m.vs || 0.015), m.delta || 2.0)) + (m.fv || 0) * v
    );
    const rippleScales = speeds.map((v) => Math.exp(-Math.pow(v / (m.vr || 0.08), 2)));
    const vDense = [];
    const stribeckDense = [];
    const envelopeDense = [];
    for (let i = 0; i < 100; i++) {
      const v = (0.095 * i) / 99;
      vDense.push(v);
      stribeckDense.push(
        m.fc + (m.fs - m.fc) * Math.exp(-Math.pow(v / (m.vs || 0.015), m.delta || 2.0)) + (m.fv || 0) * v
      );
      envelopeDense.push(Math.exp(-Math.pow(v / (m.vr || 0.08), 2)));
    }
    const profileKnots = wGrid.map((w, idx) => ({
      w_m: w,
      w_mm: w * 1000.0,
      bias_n: biasGrid[idx] || 0,
      modulation_n: modGrid[idx] || 0,
      local_fc_n: Math.max(0.0, m.fc + (modGrid[idx] || 0)),
    }));
    return {
      joint: 7,
      is_gripper: true,
      arm_id: armId,
      model: m,
      profile_knots: profileKnots,
      metrics: { rmse_total: 0, rmse_stribeck: 0, rmse_cogging: 0, rmse_modulation: 0 },
      curves: {
        theta_grid: thetaGrid,
        tau_pos_low: tauPos,
        tau_neg_low: tauNeg,
        half_sum_low: cogFit,
        cogging_fit_low: cogFit,
        modulation_actual_low: modFit,
        modulation_fit_low: modFit,
        speeds,
        mean_frictions: meanFrictions,
        ripple_scales: rippleScales,
        v_dense: vDense,
        stribeck_dense: stribeckDense,
        envelope_dense: envelopeDense,
      },
      yaml_text: buildGripperYamlPreview(armId, m),
    };
  }

  function renderArmSelector() {
    const container = $('arm-selector');
    container.innerHTML = '';
    for (const arm of state.arms) {
      const btn = document.createElement('button');
      btn.type = 'button';
      const active = arm.id === state.selectedArmId;
      btn.className = `arm-card-btn ${active ? 'selected' : ''}`;
      btn.setAttribute('role', 'radio');
      btn.setAttribute('aria-checked', active ? 'true' : 'false');
      const effLabel = arm.end_effector === 'gripper' ? '标准夹爪' : '示教手柄';
      const sPhase = arm.sweep_phase || 'idle';
      const isRunning = ['waiting_feedback', 'zeroing_friction', 'enabling', 'setting_mode', 'running'].includes(sPhase);
      const statusDot = isRunning ? '⚡ 扫描中' : (arm.online ? '🟢 在线' : '⚪ 离线/待命');
      const calLabel = (state.gripperFits[arm.id] || arm.gripper_calibrated) ? '✓ 夹爪已标定' : '待标定';
      btn.innerHTML = `
        <div class="arm-card-top">
          <span>${arm.can_port.toUpperCase()}</span>
          <span>${statusDot}</span>
        </div>
        <div class="arm-card-name">${arm.name} (${arm.namespace})</div>
        <div class="arm-card-meta">${effLabel} · ${calLabel} · 状态 ${sPhase}</div>
      `;
      btn.addEventListener('click', () => {
        state.selectedArmId = arm.id;
        $('input-output-csv').value = `/home/a/piper_x_ws/src/piper_friction/config/${arm.id}_sweep_j7.csv`;
        $('input-fit-output').value = `/home/a/piper_x_ws/src/piper_friction/config/${arm.id}_friction.yaml`;
        $('input-save-yaml-path').value = `/home/a/piper_x_ws/src/piper_friction/config/${arm.id}_friction.yaml`;
        renderAll();
      });
      container.appendChild(btn);
    }
  }

  function renderLiveGripperTable() {
    const arm = selectedArm();
    $('selected-arm-summary').textContent = `${arm.can_port} · ${arm.namespace}`;
    $('yaml-arm-label').textContent = arm.id;
    const scaleVal = Number(arm.friction_scale_applied ?? arm.friction_scale ?? 0);
    $('arm-friction-scale-badge').textContent = `驱动 friction_scale: ${scaleVal.toFixed(2)}`;
    $('arm-live-status').textContent = arm.online ? `在线 (${arm.mode})` : '离线 / 演示模式';

    const tbody = $('live-gripper-tbody');
    const wM = Number(arm.gripper_position ?? 0.02);
    const dwMs = Number(arm.gripper_velocity ?? 0.0);
    const fMeas = Number(arm.gripper_force ?? 0.0);
    const fComp = Number(arm.gripper_friction ?? 0.0);
    const effLabel = arm.end_effector === 'gripper' ? '标准夹爪' : '示教手柄';
    const hasFit = Boolean(state.gripperFits[arm.id] || arm.gripper_calibrated);
    tbody.innerHTML = `
      <tr>
        <td><strong>${effLabel} (J7)</strong></td>
        <td><span class="pill ${arm.gripper_enabled ? 'ok' : ''}">${arm.gripper_enabled ? '使能' : '释放'}</span></td>
        <td><code>${(wM * 1000).toFixed(2)} mm</code> (${wM.toFixed(4)} m)</td>
        <td><code>${(dwMs * 1000).toFixed(2)} mm/s</code></td>
        <td><code>${fMeas >= 0 ? '+' : ''}${fMeas.toFixed(4)} N</code></td>
        <td><code>${fComp >= 0 ? '+' : ''}${fComp.toFixed(4)} N</code></td>
        <td><span class="pill ${hasFit ? 'ok' : ''}">${hasFit ? '已拟合' : '待标定'}</span></td>
      </tr>
    `;
    $('calibrated-status-badge').textContent = hasFit ? '夹爪已标定' : '未标定';
  }

  function updateCliPreview() {
    const arm = selectedArm();
    const wMin = Number.parseFloat($('input-w-min').value) || 0.003;
    const wMax = Number.parseFloat($('input-w-max').value) || 0.080;
    const speeds = parseFloatList($('input-speeds').value);
    const effort = Number.parseFloat($('input-gripper-effort').value) || 2.0;
    const repeats = Number.parseInt($('input-repeats').value, 10) || 1;
    const settle = Number.parseFloat($('input-settle').value) || 0.3;
    const outCsv = $('input-output-csv').value.trim();
    const outFit = $('input-fit-output').value.trim();

    const span = Math.max(0.005, wMax - wMin);
    let totalSec = 1.5;
    for (const v of speeds) {
      if (v > 0) {
        totalSec += repeats * (2 * (span / v) + 2 * settle);
      }
    }
    $('est-duration').textContent = `${totalSec.toFixed(1)} s`;

    const cmd = [
      'ros2 run piper_friction sweep',
      `--namespace ${arm.namespace}`,
      `--arm-id ${arm.id}`,
      '--joint 7',
      `--q-min ${wMin.toFixed(4)}`,
      `--q-max ${wMax.toFixed(4)}`,
      `--speeds ${speeds.map((s) => s.toFixed(3)).join(' ')}`,
      `--gripper-effort ${effort.toFixed(2)}`,
      `--repeats ${repeats}`,
      `--settle-time ${settle.toFixed(2)}`,
      `--output-csv ${outCsv}`,
      outFit ? `--fit-output ${outFit}` : '',
    ]
      .filter(Boolean)
      .join(' \\\n  ');
    $('cli-command-preview').textContent = cmd;
  }

  function prepareCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(320, Math.floor(rect.width || canvas.width || 720));
    const height = Math.max(150, Math.floor(rect.height || canvas.height || 190));
    if (canvas.width !== width * dpr || canvas.height !== height * dpr) {
      canvas.width = width * dpr;
      canvas.height = height * dpr;
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    return { ctx, width, height };
  }

  function drawAxesBox(ctx, pad, w, h, xMin, xMax, yMin, yMax, xUnit, yUnit) {
    const plotW = w - pad.l - pad.r;
    const plotH = h - pad.t - pad.b;
    ctx.save();
    ctx.strokeStyle = '#dde5ef';
    ctx.fillStyle = '#5b6c82';
    ctx.font = '11px ui-monospace, monospace';
    ctx.lineWidth = 1;

    const yTicks = 4;
    for (let i = 0; i <= yTicks; i++) {
      const val = yMax - ((yMax - yMin) * i) / yTicks;
      const py = pad.t + (plotH * i) / yTicks;
      ctx.beginPath();
      ctx.moveTo(pad.l, py);
      ctx.lineTo(w - pad.r, py);
      ctx.stroke();
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(val.toFixed(2), pad.l - 5, py);
    }

    if (yMin < 0 && yMax > 0) {
      const y0 = pad.t + ((yMax - 0) / (yMax - yMin)) * plotH;
      ctx.strokeStyle = '#b9c7d8';
      ctx.beginPath();
      ctx.moveTo(pad.l, y0);
      ctx.lineTo(w - pad.r, y0);
      ctx.stroke();
    }

    const xTicks = 5;
    for (let i = 0; i <= xTicks; i++) {
      const val = xMin + ((xMax - xMin) * i) / xTicks;
      const px = pad.l + (plotW * i) / xTicks;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText(`${val.toFixed(1)}${i === xTicks ? ' ' + xUnit : ''}`, px, h - pad.b + 4);
    }

    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(yUnit, 6, 4);
    ctx.restore();
  }

  function drawLineSeries(ctx, xs, ys, xMap, yMap, color, width = 1.8, dash = []) {
    if (!xs || !ys || xs.length === 0) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dash);
    ctx.beginPath();
    for (let i = 0; i < xs.length; i++) {
      const px = xMap(xs[i]);
      const py = yMap(ys[i]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawScatter(ctx, xs, ys, xMap, yMap, color, radius = 3.5) {
    if (!xs || !ys) return;
    ctx.save();
    ctx.fillStyle = color;
    for (let i = 0; i < xs.length; i++) {
      ctx.beginPath();
      ctx.arc(xMap(xs[i]), yMap(ys[i]), radius, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  function renderCharts(fit) {
    const c1 = prepareCanvas($('canvas-cogging'));
    const c2 = prepareCanvas($('canvas-modulation'));
    const c3 = prepareCanvas($('canvas-stribeck'));

    if (!fit || !fit.curves) {
      [c1, c2, c3].forEach(({ ctx, width, height }) => {
        ctx.fillStyle = '#7b8ca0';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('暂无夹爪拟合曲线 — 请点击右上角“⚡ 加载夹爪非周期示例数据”或执行一次扫描采集', width / 2, height / 2);
      });
      return;
    }

    const curves = fit.curves;
    const wMm = curves.theta_grid.map((w) => w * 1000.0);
    const wMinMm = wMm[0];
    const wMaxMm = wMm[wMm.length - 1];
    const pad = { l: 48, r: 18, t: 14, b: 22 };

    // Chart 1: Non-periodic bias profile + opening/closing forces
    {
      const { ctx, width, height } = c1;
      const allY = [...curves.tau_pos_low, ...curves.tau_neg_low, ...curves.half_sum_low, ...curves.cogging_fit_low];
      const yAbs = Math.max(0.1, ...allY.map((v) => Math.abs(v))) * 1.15;
      const xMap = (x) => pad.l + ((x - wMinMm) / Math.max(1e-6, wMaxMm - wMinMm)) * (width - pad.l - pad.r);
      const yMap = (y) => pad.t + ((yAbs - y) / (2 * yAbs)) * (height - pad.t - pad.b);
      drawAxesBox(ctx, pad, width, height, wMinMm, wMaxMm, -yAbs, yAbs, 'mm', 'N');
      drawLineSeries(ctx, wMm, curves.tau_pos_low, xMap, yMap, '#9eb4cf', 1.1, [3, 2]);
      drawLineSeries(ctx, wMm, curves.tau_neg_low, xMap, yMap, '#d2aba0', 1.1, [3, 2]);
      drawLineSeries(ctx, wMm, curves.half_sum_low, xMap, yMap, '#182537', 1.4);
      drawLineSeries(ctx, wMm, curves.cogging_fit_low, xMap, yMap, '#245cbe', 2.3);
    }

    // Chart 2: Non-periodic spatial friction modulation dF_c(w)
    {
      const { ctx, width, height } = c2;
      const allY = [...curves.modulation_actual_low, ...curves.modulation_fit_low];
      const yAbs = Math.max(0.04, ...allY.map((v) => Math.abs(v))) * 1.2;
      const xMap = (x) => pad.l + ((x - wMinMm) / Math.max(1e-6, wMaxMm - wMinMm)) * (width - pad.l - pad.r);
      const yMap = (y) => pad.t + ((yAbs - y) / (2 * yAbs)) * (height - pad.t - pad.b);
      drawAxesBox(ctx, pad, width, height, wMinMm, wMaxMm, -yAbs, yAbs, 'mm', 'N');
      drawLineSeries(ctx, wMm, curves.modulation_actual_low, xMap, yMap, '#76899f', 1.3);
      drawLineSeries(ctx, wMm, curves.modulation_fit_low, xMap, yMap, '#15714d', 2.3);
    }

    // Chart 3: Stribeck + Envelope vs speed (mm/s)
    {
      const { ctx, width, height } = c3;
      const pad3 = { l: 48, r: 42, t: 14, b: 22 };
      const vDenseMm = curves.v_dense.map((v) => v * 1000.0);
      const speedsMm = curves.speeds.map((v) => v * 1000.0);
      const vMaxMm = vDenseMm[vDenseMm.length - 1] || 90.0;
      const fMax = Math.max(0.15, ...curves.mean_frictions, ...curves.stribeck_dense) * 1.15;
      const xMap = (v) => pad3.l + (v / vMaxMm) * (width - pad3.l - pad3.r);
      const yMapF = (f) => pad3.t + ((fMax - f) / fMax) * (height - pad3.t - pad3.b);
      const yMapW = (w) => pad3.t + (1.0 - Math.max(0, Math.min(1.1, w))) * (height - pad3.t - pad3.b);

      drawAxesBox(ctx, pad3, width, height, 0, vMaxMm, 0, fMax, 'mm/s', 'F(|dw|) N');
      drawLineSeries(ctx, vDenseMm, curves.stribeck_dense, xMap, yMapF, '#245cbe', 2.2);
      drawScatter(ctx, speedsMm, curves.mean_frictions, xMap, yMapF, '#184287', 3.8);
      drawLineSeries(ctx, vDenseMm, curves.envelope_dense, xMap, yMapW, '#d96b27', 2.0, [5, 3]);
      drawScatter(ctx, speedsMm, curves.ripple_scales, xMap, yMapW, '#d96b27', 3.4);
    }
  }

  function renderFitSection() {
    const arm = selectedArm();
    const fit = state.gripperFits[arm.id];
    const fitKey = fit && fit.model
      ? `${arm.id}:${fit.model.fc}:${fit.model.fs}:${fit.model.kp_equiv}:${fit.model.kv_equiv}:${(fit.model.w_grid || []).length}:${fit.metrics ? fit.metrics.rmse_total : 0}`
      : `${arm.id}:empty`;
    if (state.lastRenderedFitKey === fitKey) {
      return;
    }
    state.lastRenderedFitKey = fitKey;

    if (!fit) {
      $('val-fc').textContent = '—';
      $('val-fs').textContent = '—';
      $('val-vs').textContent = '—';
      $('val-fv').textContent = '—';
      $('val-kp-equiv').textContent = '—';
      if ($('val-kv-equiv')) $('val-kv-equiv').textContent = '—';
      $('val-rmse').textContent = '—';
      $('knots-tbody').innerHTML =
        '<tr><td colspan="6" class="empty-cell">尚未运行拟合，请点击右上角“⚡ 加载夹爪非周期示例数据”或完成一次扫描采集</td></tr>';
      renderCharts(null);
      updateEvaluationBox();
      return;
    }

    const m = fit.model;
    $('val-fc').textContent = m.fc.toFixed(4);
    $('val-fs').textContent = m.fs.toFixed(4);
    $('val-vs').textContent = (m.vs * 1000).toFixed(2);
    $('val-fv').textContent = m.fv.toFixed(3);
    $('val-kp-equiv').textContent = (m.kp_equiv || 45.0).toFixed(1);
    if ($('val-kv-equiv')) $('val-kv-equiv').textContent = (m.kv_equiv || 12.5).toFixed(2);
    $('val-rmse').textContent = fit.metrics && fit.metrics.rmse_total ? fit.metrics.rmse_total.toFixed(4) : '已标定';

    const kbody = $('knots-tbody');
    kbody.innerHTML = '';
    const knots = fit.profile_knots || [];
    if (knots.length === 0 && m.w_grid) {
      for (let i = 0; i < m.w_grid.length; i++) {
        knots.push({
          w_m: m.w_grid[i],
          w_mm: m.w_grid[i] * 1000.0,
          bias_n: m.bias_grid[i],
          modulation_n: m.modulation_grid[i],
          local_fc_n: Math.max(0.0, m.fc + m.modulation_grid[i]),
        });
      }
    }
    knots.forEach((k, idx) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><strong>#${idx + 1}</strong></td>
        <td>${k.w_mm.toFixed(2)} mm</td>
        <td>${k.w_m.toFixed(5)} m</td>
        <td>${k.bias_n >= 0 ? '+' : ''}${k.bias_n.toFixed(4)} N</td>
        <td>${k.modulation_n >= 0 ? '+' : ''}${k.modulation_n.toFixed(4)} N</td>
        <td><strong>${k.local_fc_n.toFixed(4)} N</strong></td>
      `;
      kbody.appendChild(tr);
    });

    renderCharts(fit);
    const nextYaml = fit.yaml_text || buildGripperYamlPreview(arm.id, m);
    if ($('yaml-output-preview').textContent !== nextYaml) {
      $('yaml-output-preview').textContent = nextYaml;
    }
    updateEvaluationBox();
  }

  function updateEvaluationBox() {
    const w = Number.parseFloat($('slider-eval-q').value) || 0.035;
    const dw = Number.parseFloat($('slider-eval-dq').value) || 0.015;
    const biasScale = Number.parseFloat($('input-cog-scale').value) || 1.0;
    const fricScale = Number.parseFloat($('input-fric-scale').value) || 0.85;
    $('lbl-eval-q').textContent = `${(w * 1000).toFixed(1)} mm (${w.toFixed(4)} m)`;
    $('lbl-eval-dq').textContent = `${dw >= 0 ? '+' : ''}${(dw * 1000).toFixed(1)} mm/s`;

    const arm = selectedArm();
    const fit = state.gripperFits[arm.id];
    if (!fit || !fit.model) {
      $('eval-env').textContent = '—';
      $('eval-cog').textContent = '0.0000 N';
      $('eval-mod').textContent = '0.0000 N';
      $('eval-stribeck').textContent = '0.0000 N';
      if ($('eval-dw-lead')) $('eval-dw-lead').textContent = '+0.00 mm';
      $('eval-total').textContent = '0.0000 N';
      return;
    }

    const m = fit.model;
    const s = Math.abs(dw);
    if (s === 0) {
      $('eval-env').textContent = '1.000';
      $('eval-cog').textContent = '0.0000 N';
      $('eval-mod').textContent = '0.0000 N';
      $('eval-stribeck').textContent = '0.0000 N';
      if ($('eval-dw-lead')) $('eval-dw-lead').textContent = '+0.00 mm';
      $('eval-total').textContent = '0.0000 N';
      return;
    }
    const env = Math.exp(-Math.pow(dw / m.vr, 2));
    const gate = Math.tanh(s / m.v0);
    const dir = Math.tanh(dw / m.v0);
    const biasVal = biasScale * gate * interp1d(w, m.w_grid, m.bias_grid);
    const modRaw = env * interp1d(w, m.w_grid, m.modulation_grid);
    const dry = m.fc + (m.fs - m.fc) * Math.exp(-Math.pow(s / m.vs, m.delta));
    const modVal = fricScale * modRaw * dir;
    const stribeckVal = fricScale * (dry * dir + m.fv * dw);
    const total = fricScale * (Math.max(0.0, dry + modRaw) * dir + m.fv * dw) + biasVal;
    const kp = m.kp_equiv || 45.0;
    const kv = m.kv_equiv || 12.5;
    const dwLeadMm = (total / kp + dw / kv) * 1000.0;

    $('eval-env').textContent = env.toFixed(3);
    $('eval-cog').textContent = `${biasVal >= 0 ? '+' : ''}${biasVal.toFixed(4)} N`;
    $('eval-mod').textContent = `${modVal >= 0 ? '+' : ''}${modVal.toFixed(4)} N`;
    $('eval-stribeck').textContent = `${stribeckVal >= 0 ? '+' : ''}${stribeckVal.toFixed(4)} N`;
    if ($('eval-dw-lead')) {
      $('eval-dw-lead').textContent = `${dwLeadMm >= 0 ? '+' : ''}${dwLeadMm.toFixed(2)} mm`;
    }
    $('eval-total').textContent = `${total >= 0 ? '+' : ''}${total.toFixed(4)} N`;
  }

  async function triggerFit(source = 'memory', extra = {}) {
    const arm = selectedArm();
    const wMin = Number.parseFloat($('input-w-min').value) || 0.003;
    const wMax = Number.parseFloat($('input-w-max').value) || 0.080;
    try {
      const resp = await fetch('/api/fit', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          source,
          arm_id: arm.id,
          joint: 7,
          q_min: wMin,
          q_max: wMax,
          grid_step: 0.0005,
          trim_ratio: 0.08,
          ...extra,
        }),
      });
      const data = await resp.json();
      if (!resp.ok || !data.ok) {
        throw new Error(data.error || `HTTP ${resp.status}`);
      }
      state.gripperFits[arm.id] = data.fit;
      showNotice(`已完成 ${arm.name} 夹爪非周期摩擦与偏置剖面辨识`, 'ok');
      renderAll();
    } catch (err) {
      if (!state.serverOnline && (source === 'demo' || source === 'memory')) {
        state.gripperFits[arm.id] = buildClientFallbackGripperFit(arm.id);
        showNotice('离线演示模式：已生成夹爪非周期空间剖面与 Stribeck 拟合结果', 'ok');
        renderAll();
      } else {
        showNotice(`夹爪拟合失败：${err.message}`, 'error');
      }
    }
  }

  async function pollStatus() {
    try {
      const resp = await fetch('/api/status', { cache: 'no-store' });
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const data = await resp.json();
      state.serverOnline = true;
      $('conn-dot').className = 'dot ok';
      $('conn-text').textContent = 'ROS 2 工作台在线';
      if (Array.isArray(data.arms) && data.arms.length > 0) {
        state.arms = data.arms;
        for (const a of data.arms) {
          if (a.gripper_model && !state.gripperFits[a.id]) {
            state.gripperFits[a.id] = buildFitFromLoadedModel(a.id, a.gripper_model);
          }
        }
      }
      if (data.joint_fits) {
        for (const [armId, fits] of Object.entries(data.joint_fits)) {
          if (fits && fits['7']) {
            state.gripperFits[armId] = fits['7'];
          }
        }
      }
      const arm = selectedArm();
      const sw = (data.sweeps_by_ns && data.sweeps_by_ns[arm.namespace]) || data.sweep;
      if (sw) {
        state.sweep = sw;
        $('sweep-phase-pill').textContent = sw.phase;
        $('prog-queue-text').textContent = sw.phase;
        $('prog-seg-text').textContent = sw.current_segment
          ? `${sw.segment_index + 1}/${sw.total_segments} (${sw.current_segment.kind})`
          : `${sw.segment_index}/${sw.total_segments}`;
        $('prog-samples-text').textContent = String(sw.sample_count || 0);
        $('prog-q-des').textContent = `${((sw.current_q_des || 0) * 1000).toFixed(2)} mm`;
        $('prog-dq-des').textContent = `${((sw.current_dq_des || 0) * 1000).toFixed(2)} mm/s`;
        const pct = sw.total_segments > 0 ? Math.min(100, Math.round(((sw.segment_index + 1) / sw.total_segments) * 100)) : 0;
        $('prog-bar-fill').style.width = `${sw.phase === 'completed' ? 100 : pct}%`;
        const running = ['waiting_feedback', 'zeroing_friction', 'enabling', 'setting_mode', 'running'].includes(sw.phase);
        $('btn-stop-sweep').disabled = !running;
        $('btn-start-sweep').disabled = running;
      }
      renderArmSelector();
      renderLiveGripperTable();
      renderFitSection();
    } catch (_err) {
      state.serverOnline = false;
      $('conn-dot').className = 'dot';
      $('conn-text').textContent = '离线演示模式';
    }
  }

  function renderAll() {
    renderArmSelector();
    renderLiveGripperTable();
    updateCliPreview();
    renderFitSection();
  }

  function bindEvents() {
    ['input-w-min', 'input-w-max', 'input-speeds', 'input-gripper-effort', 'input-repeats', 'input-settle', 'input-output-csv', 'input-fit-output'].forEach((id) => {
      $(id).addEventListener('input', updateCliPreview);
    });

    ['slider-eval-q', 'slider-eval-dq', 'input-cog-scale', 'input-fric-scale'].forEach((id) => {
      $(id).addEventListener('input', updateEvaluationBox);
    });

    $('btn-demo-gripper').addEventListener('click', () => triggerFit('demo'));
    $('btn-demo-single').addEventListener('click', () => triggerFit('demo'));
    $('btn-run-fit').addEventListener('click', () => triggerFit('memory'));

    $('btn-copy-cli').addEventListener('click', () => {
      navigator.clipboard?.writeText($('cli-command-preview').textContent || '');
      showNotice('已复制夹爪扫描 CLI 命令', 'ok');
    });

    $('btn-copy-yaml').addEventListener('click', () => {
      navigator.clipboard?.writeText($('yaml-output-preview').textContent || '');
      showNotice('已复制 YAML 内容', 'ok');
    });

    $('btn-zero-friction-scale').addEventListener('click', async () => {
      const arm = selectedArm();
      try {
        await fetch('/api/arm/friction_scale', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ namespace: arm.namespace, friction_scale: 0.0 }),
        });
        showNotice(`已请求将 ${arm.namespace} 驱动 friction_scale 置为 0.0`, 'ok');
      } catch (e) {
        showNotice(`设置失败：${e.message}`, 'error');
      }
    });

    $('btn-restore-friction-scale').addEventListener('click', async () => {
      const arm = selectedArm();
      try {
        await fetch('/api/arm/friction_scale', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ namespace: arm.namespace, friction_scale: 0.85 }),
        });
        showNotice(`已请求将 ${arm.namespace} 驱动 friction_scale 恢复为 0.85`, 'ok');
      } catch (e) {
        showNotice(`设置失败：${e.message}`, 'error');
      }
    });

    $('btn-start-sweep').addEventListener('click', async () => {
      const arm = selectedArm();
      const wMin = Number.parseFloat($('input-w-min').value) || 0.003;
      const wMax = Number.parseFloat($('input-w-max').value) || 0.080;
      const speeds = parseFloatList($('input-speeds').value);
      const effort = Number.parseFloat($('input-gripper-effort').value) || 2.0;
      const repeats = Number.parseInt($('input-repeats').value, 10) || 1;
      const settle = Number.parseFloat($('input-settle').value) || 0.3;
      const outCsv = $('input-output-csv').value.trim();
      const outFit = $('input-fit-output').value.trim();

      try {
        const resp = await fetch('/api/sweep/start', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            namespace: arm.namespace,
            joint: 7,
            q_min: wMin,
            q_max: wMax,
            speeds,
            gripper_effort: effort,
            repeats,
            settle_time: settle,
            approach_speed: 0.04,
            output_csv: outCsv,
            fit_output: outFit,
          }),
        });
        const data = await resp.json();
        if (!resp.ok || !data.ok) throw new Error(data.error || `HTTP ${resp.status}`);
        showNotice(`已启动 ${arm.name} 夹爪多速往复扫描采集`, 'ok');
        pollStatus();
      } catch (e) {
        showNotice(`启动扫描失败：${e.message}`, 'error');
      }
    });

    $('btn-stop-sweep').addEventListener('click', async () => {
      const arm = selectedArm();
      try {
        await fetch('/api/sweep/stop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ namespace: arm.namespace }),
        });
        showNotice('已停止夹爪扫描并释放夹爪出力', 'warn');
        pollStatus();
      } catch (e) {
        showNotice(`停止失败：${e.message}`, 'error');
      }
    });

    $('input-csv-file').addEventListener('change', async (ev) => {
      const file = ev.target.files && ev.target.files[0];
      if (!file) return;
      const text = await file.text();
      await triggerFit('csv_text', { csv_text: text });
      ev.target.value = '';
    });

    $('btn-save-yaml').addEventListener('click', async () => {
      const arm = selectedArm();
      const fit = state.gripperFits[arm.id];
      const path = $('input-save-yaml-path').value.trim();
      if (!fit || !fit.model) {
        showNotice('请先执行夹爪数据拟合后再保存 YAML', 'warn');
        return;
      }
      try {
        const resp = await fetch('/api/save_yaml', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            path,
            arm_id: arm.id,
            gripper: fit.model,
          }),
        });
        const data = await resp.json();
        if (!resp.ok || !data.ok) throw new Error(data.error || `HTTP ${resp.status}`);
        const fb = $('save-yaml-feedback');
        fb.hidden = false;
        fb.textContent = `✅ 已合并保存至 ${data.path}`;
        showNotice(`已将夹爪非周期摩擦模型合并保存至 ${data.path}`, 'ok');
      } catch (e) {
        showNotice(`保存 YAML 失败：${e.message}`, 'error');
      }
    });
  }

  updateClock();
  setInterval(updateClock, 1000);
  bindEvents();
  pollStatus().then(() => {
    if (!state.serverOnline && !state.gripperFits.arm_1) {
      state.gripperFits.arm_1 = buildClientFallbackGripperFit('arm_1');
    }
    renderAll();
  });
  setInterval(pollStatus, 400);
})();
