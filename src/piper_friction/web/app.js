(() => {
  'use strict';

  const DEFAULT_ARMS = [
    { id: 'arm_1', name: '机械臂 1', namespace: '/arm_1', can_port: 'can0', end_effector: 'teaching_handle', online: false, mode: 'mit', enabled: [false, false, false, false, false, false], position: [0, 0.5, -0.5, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0], torque: [0, 0, 0, 0, 0, 0], gravity_torque: [0, 2.1, -0.8, 0, 0.1, 0], calibrated_joints: [] },
    { id: 'arm_2', name: '机械臂 2', namespace: '/arm_2', can_port: 'can1', end_effector: 'teaching_handle', online: false, mode: 'mit', enabled: [false, false, false, false, false, false], position: [0, 0.5, -0.5, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0], torque: [0, 0, 0, 0, 0, 0], gravity_torque: [0, 2.1, -0.8, 0, 0.1, 0], calibrated_joints: [] },
    { id: 'arm_3', name: '机械臂 3', namespace: '/arm_3', can_port: 'can2', end_effector: 'gripper', online: false, mode: 'mit', enabled: [false, false, false, false, false, false], position: [0, 0.5, -0.5, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0], torque: [0, 0, 0, 0, 0, 0], gravity_torque: [0, 2.1, -0.8, 0, 0.1, 0], calibrated_joints: [] },
    { id: 'arm_4', name: '机械臂 4', namespace: '/arm_4', can_port: 'can3', end_effector: 'gripper', online: false, mode: 'mit', enabled: [false, false, false, false, false, false], position: [0, 0.5, -0.5, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0], torque: [0, 0, 0, 0, 0, 0], gravity_torque: [0, 2.1, -0.8, 0, 0.1, 0], calibrated_joints: [] },
  ];

  const JOINT_TIPS = {
    1: '直立安装下转轴平行于重力，τ_g ≡ 0，零重力污染',
    2: '肩部俯仰主承重轴，自动扣除 gravity_torques(q)',
    3: '肘部俯仰轴，建议 J2 保持约 35°~50° 安全姿态',
    4: '前臂滚转轴，前臂竖直时重力矩近似为 0',
    5: '腕部俯仰轴，自动扣除末端与腕部理论重力矩',
    6: '法兰旋转轴，对称末端下重力矩接近 0',
  };

  const state = {
    serverOnline: false,
    arms: DEFAULT_ARMS,
    selectedArmId: 'arm_1',
    activeJoint: 1,
    jointPlan: {
      1: { checked: true, q_min: -0.50, q_max: 0.50 },
      2: { checked: true, q_min: 0.55, q_max: 1.20 },
      3: { checked: true, q_min: -1.80, q_max: -1.15 },
      4: { checked: true, q_min: -0.45, q_max: 0.45 },
      5: { checked: true, q_min: -0.40, q_max: 0.40 },
      6: { checked: true, q_min: -0.50, q_max: 0.50 },
    },
    sweep: {
      phase: 'idle',
      error: '',
      queue_joints: [],
      queue_index: 0,
      queue_total: 0,
      segment_index: 0,
      total_segments: 0,
      current_q_des: 0,
      current_dq_des: 0,
      sample_count: 0,
    },
    jointFits: {
      arm_1: {},
      arm_2: {},
      arm_3: {},
      arm_4: {},
    },
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

  function radToDeg(rad) {
    return (rad * 180) / Math.PI;
  }

  // Client-side fallback demo generator when opened as static file:// without backend
  function buildClientFallbackFit(armId, joint) {
    const scale = [1.0, 1.25, 0.95, 0.45, 0.38, 0.25][joint - 1];
    const w1 = [36, 36, 36, 28, 28, 24][joint - 1];
    const w2 = w1 * 2;
    const model = {
      fc: Number((0.32 * scale).toFixed(4)),
      fs: Number((0.76 * scale).toFixed(4)),
      vs: Number((0.10 + 0.01 * joint).toFixed(4)),
      delta: 2.0,
      fv: Number((0.14 * scale).toFixed(4)),
      v0: 0.005,
      vr: Number((0.32 + 0.02 * joint).toFixed(4)),
      harmonics: [
        { omega: w1, a: Number((0.15 * scale).toFixed(4)), b: Number((-0.07 * scale).toFixed(4)), c: Number((0.08 * scale).toFixed(4)), d: Number((0.04 * scale).toFixed(4)) },
        { omega: w2, a: Number((-0.04 * scale).toFixed(4)), b: Number((0.05 * scale).toFixed(4)), c: Number((0.03 * scale).toFixed(4)), d: Number((-0.02 * scale).toFixed(4)) },
      ],
    };
    const plan = state.jointPlan[joint];
    const n = 260;
    const thetaGrid = [];
    const halfSum = [];
    const cogFit = [];
    const modActual = [];
    const modFit = [];
    const tauPos = [];
    const tauNeg = [];
    const v0Speed = 0.03;
    const env0 = Math.exp(-Math.pow(v0Speed / model.vr, 2));
    const meanF0 = model.fc + (model.fs - model.fc) * Math.exp(-Math.pow(v0Speed / model.vs, model.delta)) + model.fv * v0Speed;

    for (let i = 0; i < n; i++) {
      const th = plan.q_min + ((plan.q_max - plan.q_min) * i) / (n - 1);
      thetaGrid.push(th);
      const cog = env0 * model.harmonics.reduce((acc, h) => acc + h.a * Math.cos(h.omega * th) + h.b * Math.sin(h.omega * th), 0);
      const mod = env0 * model.harmonics.reduce((acc, h) => acc + h.c * Math.cos(h.omega * th) + h.d * Math.sin(h.omega * th), 0);
      const noiseS = 0.003 * scale * Math.sin(137 * th);
      const noiseD = 0.003 * scale * Math.cos(173 * th);
      cogFit.push(cog);
      halfSum.push(cog + noiseS);
      modFit.push(mod);
      modActual.push(mod + noiseD);
      tauPos.push(cog + meanF0 + mod + noiseS);
      tauNeg.push(cog - meanF0 - mod - noiseD);
    }

    const speeds = [0.03, 0.06, 0.12, 0.25, 0.5, 1.0, 1.5];
    const meanFrictions = speeds.map((v) => model.fc + (model.fs - model.fc) * Math.exp(-Math.pow(v / model.vs, model.delta)) + model.fv * v);
    const rippleScales = speeds.map((v) => Math.exp(-Math.pow(v / model.vr, 2)));
    const vDense = [];
    const stribeckDense = [];
    const envelopeDense = [];
    for (let i = 0; i < 100; i++) {
      const v = (1.65 * i) / 99;
      vDense.push(v);
      stribeckDense.push(model.fc + (model.fs - model.fc) * Math.exp(-Math.pow(v / model.vs, model.delta)) + model.fv * v);
      envelopeDense.push(Math.exp(-Math.pow(v / model.vr, 2)));
    }

    return {
      joint,
      arm_id: armId,
      model,
      harmonics: model.harmonics.map((h) => ({
        ...h,
        cogging_amplitude: Math.hypot(h.a, h.b),
        modulation_amplitude: Math.hypot(h.c, h.d),
        period_deg: 360 / h.omega,
      })),
      metrics: {
        rmse_total: Number((0.0042 * scale).toFixed(5)),
        rmse_stribeck: Number((0.0018 * scale).toFixed(5)),
        rmse_cogging: Number((0.0024 * scale).toFixed(5)),
        rmse_modulation: Number((0.0021 * scale).toFixed(5)),
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
      yaml_text: '',
    };
  }

  function buildYamlTextForArm(armId) {
    const fits = state.jointFits[armId] || {};
    const lines = ['version: 1', `arm_id: ${armId}`, 'joints:'];
    const joints = Object.keys(fits).map(Number).sort((a, b) => a - b);
    if (joints.length === 0) {
      return '# 完成任意关节拟合后，此处将生成对应的 YAML 标定参数';
    }
    for (const j of joints) {
      const m = fits[j].model;
      lines.push(`  joint${j}:`);
      lines.push(`    fc: ${m.fc.toFixed(6)}`);
      lines.push(`    fs: ${m.fs.toFixed(6)}`);
      lines.push(`    vs: ${m.vs.toFixed(6)}`);
      lines.push(`    delta: ${m.delta.toFixed(2)}`);
      lines.push(`    fv: ${m.fv.toFixed(6)}`);
      lines.push(`    v0: ${m.v0.toFixed(6)}`);
      lines.push(`    vr: ${m.vr.toFixed(6)}`);
      lines.push('    harmonics:');
      for (const h of m.harmonics) {
        lines.push(`    - omega: ${h.omega.toFixed(4)}`);
        lines.push(`      a: ${h.a.toFixed(6)}`);
        lines.push(`      b: ${h.b.toFixed(6)}`);
        lines.push(`      c: ${h.c.toFixed(6)}`);
        lines.push(`      d: ${h.d.toFixed(6)}`);
      }
    }
    return lines.join('\n');
  }

  // Render CAN Port / Arm Selector Cards
  function renderArmSelector() {
    const container = $('arm-selector');
    container.innerHTML = '';
    for (const arm of state.arms) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = `arm-card-btn ${arm.id === state.selectedArmId ? 'selected' : ''}`;
      const calCount = Object.keys(state.jointFits[arm.id] || {}).length;
      const sPhase = arm.sweep_phase || 'idle';
      const isRunning = ['waiting_feedback', 'zeroing_friction', 'enabling', 'setting_mode', 'running'].includes(sPhase);
      const statusDot = isRunning ? '⚡ 扫描中' : (arm.online ? '🟢 在线' : '⚪ 离线/待命');
      btn.innerHTML = `
        <div class="arm-card-top">
          <span>${arm.can_port.toUpperCase()}</span>
          <span>${statusDot}</span>
        </div>
        <div class="arm-card-name">${arm.name} (${arm.namespace})</div>
        <div class="arm-card-meta">已标定 ${calCount}/6 轴 · 状态 ${sPhase}</div>
      `;
      btn.addEventListener('click', () => {
        state.selectedArmId = arm.id;
        if (state.sweepsByNs && state.sweepsByNs[arm.namespace]) {
          state.sweep = state.sweepsByNs[arm.namespace];
        }
        const defaultCsv = `/home/a/piper_x_ws/src/piper_friction/config/${arm.id}_sweep.csv`;
        const defaultYaml = `/home/a/piper_x_ws/src/piper_friction/config/${arm.id}_friction.yaml`;
        $('input-output-csv').value = defaultCsv;
        $('input-fit-output').value = defaultYaml;
        if ($('input-save-yaml-path')) {
          $('input-save-yaml-path').value = defaultYaml;
        }
        setDriverFrictionScale(arm.namespace, 0.0, true);
        renderAll();
      });
      container.appendChild(btn);
    }
    const cur = selectedArm();
    $('selected-arm-summary').textContent = `${cur.can_port} · ${cur.namespace}`;
    $('yaml-arm-label').textContent = `${cur.id} (${cur.can_port})`;
  }

  async function setDriverFrictionScale(namespace, scale, silent = false) {
    try {
      const res = await fetch('/api/arm/friction_scale', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ namespace, friction_scale: scale }),
      });
      const data = await res.json();
      if (!silent && res.ok && data.ok) {
        showNotice(`已将 ${namespace} 驱动 friction_scale 设为 ${Number(scale).toFixed(2)}`, 'ok');
      }
    } catch (_) {
      // Ignore in offline mode
    }
  }

  // Render 6-Joint Live Telemetry Table
  function renderLiveTelemetry() {
    const arm = selectedArm();
    const pill = $('arm-live-status');
    const fBadge = $('arm-friction-scale-badge');
    if (fBadge) {
      const fScale = typeof arm.friction_scale === 'number' ? arm.friction_scale : 0.0;
      fBadge.textContent = `驱动 friction_scale: ${fScale.toFixed(2)}`;
      fBadge.className = `pill ${fScale === 0 ? 'live' : 'busy'}`;
    }
    if (arm.online) {
      pill.textContent = `遥测在线 · 模式 ${arm.mode}`;
      pill.className = 'pill live';
    } else {
      pill.textContent = state.serverOnline ? '驱动未发布遥测' : '静态/离线预览模式';
      pill.className = 'pill';
    }

    const tbody = $('live-joints-tbody');
    tbody.innerHTML = '';
    const fits = state.jointFits[arm.id] || {};
    for (let j = 1; j <= 6; j++) {
      const idx = j - 1;
      const q = arm.position ? arm.position[idx] : null;
      const dq = arm.velocity ? arm.velocity[idx] : null;
      const tau = arm.torque ? arm.torque[idx] : null;
      const g = arm.gravity_torque ? arm.gravity_torque[idx] : 0;
      const en = arm.enabled ? arm.enabled[idx] : false;
      const fit = fits[j];
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><strong>J${j}</strong></td>
        <td>${en ? '<span style="color:var(--green)">● 使能</span>' : '<span style="color:var(--muted)">○ 失能</span>'}</td>
        <td>${q !== null ? `${q.toFixed(3)} (${radToDeg(q).toFixed(1)}°)` : '—'}</td>
        <td>${dq !== null ? dq.toFixed(3) : '—'}</td>
        <td>${tau !== null ? tau.toFixed(3) : '—'}</td>
        <td>${g.toFixed(3)}</td>
        <td>${fit ? `<span style="color:var(--green);font-weight:600">✓ 已标定 (RMSE ${fit.metrics.rmse_total.toFixed(4)})</span>` : '<span style="color:var(--muted)">待标定</span>'}</td>
      `;
      tbody.appendChild(tr);
    }
  }

  // Render 6-Joint Sweep Plan Table
  function renderPlanTable() {
    const tbody = $('plan-joints-tbody');
    tbody.innerHTML = '';
    for (let j = 1; j <= 6; j++) {
      const item = state.jointPlan[j];
      const spanRad = Math.max(0, item.q_max - item.q_min);
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><input type="checkbox" data-plan-check="${j}" ${item.checked ? 'checked' : ''}></td>
        <td><strong>J${j}</strong></td>
        <td><input type="number" step="0.05" data-plan-min="${j}" value="${item.q_min}"></td>
        <td><input type="number" step="0.05" data-plan-max="${j}" value="${item.q_max}"></td>
        <td><code>${spanRad.toFixed(2)} rad (${radToDeg(spanRad).toFixed(0)}°)</code></td>
        <td style="font-size:10.5px;color:var(--muted)">${JOINT_TIPS[j]}</td>
        <td><button type="button" class="btn-xs" data-run-single="${j}">▶ 仅扫 J${j}</button></td>
      `;
      tbody.appendChild(tr);
    }

    tbody.querySelectorAll('[data-plan-check]').forEach((el) => {
      el.addEventListener('change', (e) => {
        const j = Number(e.target.dataset.planCheck);
        state.jointPlan[j].checked = e.target.checked;
        updateCliPreview();
      });
    });
    tbody.querySelectorAll('[data-plan-min]').forEach((el) => {
      el.addEventListener('input', (e) => {
        const j = Number(e.target.dataset.planMin);
        const val = Number.parseFloat(e.target.value);
        if (Number.isFinite(val)) {
          state.jointPlan[j].q_min = val;
          updateCliPreview();
        }
      });
    });
    tbody.querySelectorAll('[data-plan-max]').forEach((el) => {
      el.addEventListener('input', (e) => {
        const j = Number(e.target.dataset.planMax);
        const val = Number.parseFloat(e.target.value);
        if (Number.isFinite(val)) {
          state.jointPlan[j].q_max = val;
          updateCliPreview();
        }
      });
    });
    tbody.querySelectorAll('[data-run-single]').forEach((el) => {
      el.addEventListener('click', (e) => {
        const j = Number(e.currentTarget.dataset.runSingle);
        state.activeJoint = j;
        renderJointTabs();
        startOnlineSweep([j]);
      });
    });

    updateCliPreview();
  }

  function updateCliPreview() {
    const arm = selectedArm();
    const speeds = parseFloatList($('input-speeds').value);
    const repeats = Math.max(1, Number.parseInt($('input-repeats').value, 10) || 1);
    const settle = Math.max(0, Number.parseFloat($('input-settle').value) || 0.4);
    const kp = parseFloatList($('input-kp').value);
    const outCsv = $('input-output-csv').value.trim() || `/tmp/${arm.id}_sweep.csv`;
    const fitOut = $('input-fit-output').value.trim() || `/tmp/${arm.id}_friction.yaml`;

    const j = state.activeJoint;
    const plan = state.jointPlan[j];
    const span = Math.max(0.05, plan.q_max - plan.q_min);
    let oneJointSec = 2.0;
    for (const v of speeds) {
      if (v > 0) {
        oneJointSec += repeats * (2 * (span / v) + 2 * settle);
      }
    }
    const checkedJoints = [1, 2, 3, 4, 5, 6].filter((idx) => state.jointPlan[idx].checked);
    const totalSec = oneJointSec * Math.max(1, checkedJoints.length);
    $('est-duration').textContent = `单轴约 ${oneJointSec.toFixed(0)}s / 勾选 ${checkedJoints.length} 轴共约 ${(totalSec / 60).toFixed(1)} min`;

    const cmd = [
      `ros2 run piper_friction sweep \\`,
      `  --namespace ${arm.namespace} --arm-id ${arm.id} --joint ${j} \\`,
      `  --q-min ${plan.q_min.toFixed(2)} --q-max ${plan.q_max.toFixed(2)} \\`,
      `  --speeds ${speeds.join(' ')} --repeats ${repeats} --settle-time ${settle} \\`,
      `  --kp ${kp.join(' ')} \\`,
      `  --output-csv ${outCsv.replace(/\.csv$/, `_j${j}.csv`)} \\`,
      `  --fit-output ${fitOut}`,
    ].join('\n');
    $('cli-command-preview').textContent = cmd;
    $('lbl-active-joint').textContent = `J${state.activeJoint}`;
  }

  function renderSweepProgress() {
    const sw = state.sweep;
    const pill = $('sweep-phase-pill');
    const busy = [
      'waiting_feedback', 'zeroing_friction', 'enabling', 'setting_mode', 'running', 'restoring_friction',
      'v0_pending_fit', 'v0_waiting', 'v0_zeroing', 'v0_enabling',
      'v0_setting_mode', 'v0_noise', 'v0_burst', 'v0_restoring',
    ].includes(sw.phase);
    $('btn-start-queue').disabled = busy;
    $('btn-start-single').disabled = busy;
    if ($('btn-start-v0')) $('btn-start-v0').disabled = busy;
    $('btn-stop-sweep').disabled = !busy;

    pill.textContent = `${sw.phase}${sw.error ? ' · ' + sw.error : ''}`;
    pill.className = `pill ${busy ? 'busy' : sw.phase === 'completed' ? 'live' : sw.phase === 'failed' ? 'error' : ''}`;

    const qJoints = sw.queue_joints || [];
    if (qJoints.length > 0) {
      const activeJ = qJoints[sw.queue_index] || qJoints[qJoints.length - 1];
      $('prog-queue-text').textContent = `正在扫 J${activeJ} (${sw.queue_index + 1} / ${qJoints.length} 轴: [${qJoints.map((x) => 'J' + x).join('→')}])`;
    } else {
      $('prog-queue-text').textContent = sw.phase === 'completed' ? '队列全部完成 ✓' : '未开始';
    }

    const totalSegs = sw.total_segments || 0;
    const segIdx = sw.segment_index || 0;
    $('prog-seg-text').textContent = totalSegs > 0 ? `${Math.min(segIdx + 1, totalSegs)} / ${totalSegs}` : '—';
    $('prog-samples-text').textContent = String(sw.sample_count || 0);
    $('prog-q-des').textContent = `${Number(sw.current_q_des || 0).toFixed(3)} rad`;
    $('prog-dq-des').textContent = `${Number(sw.current_dq_des || 0).toFixed(3)} rad/s`;

    let pct = 0;
    if (sw.phase === 'completed') {
      pct = 100;
    } else if (totalSegs > 0) {
      const qTotal = Math.max(1, sw.queue_total || 1);
      const qIdx = sw.queue_index || 0;
      pct = Math.min(99, (((qIdx + segIdx / totalSegs) / qTotal) * 100));
    }
    $('prog-bar-fill').style.width = `${pct.toFixed(1)}%`;

    updateWorkflowBanner(busy);
  }

  function updateWorkflowBanner(busy) {
    const arm = selectedArm();
    const calCount = Object.keys(state.jointFits[arm.id] || {}).length;
    const s1 = $('wf-step-1');
    const s2 = $('wf-step-2');
    const s3 = $('wf-step-3');
    const s4 = $('wf-step-4');
    const s5 = $('wf-step-5');
    [s1, s2, s3, s4, s5].forEach((el) => el.classList.remove('active', 'done'));

    s1.classList.add('done');
    s2.classList.add('done');
    if (busy) {
      s3.classList.add('active');
    } else if (calCount > 0) {
      s3.classList.add('done');
      s4.classList.add('done');
      s5.classList.add('active');
    } else {
      s3.classList.add('active');
    }
  }

  function renderJointTabs() {
    const arm = selectedArm();
    const fits = state.jointFits[arm.id] || {};
    const container = $('fit-joint-tabs');
    container.innerHTML = '';
    for (let j = 1; j <= 6; j++) {
      const btn = document.createElement('button');
      btn.type = 'button';
      const hasFit = Boolean(fits[j]);
      btn.className = `joint-tab-btn ${state.activeJoint === j ? 'active' : ''} ${hasFit ? 'calibrated' : ''}`;
      btn.textContent = `J${j} ${hasFit ? '✓' : ''}`;
      btn.addEventListener('click', () => {
        state.activeJoint = j;
        renderJointTabs();
        renderFitSection();
        updateCliPreview();
      });
      container.appendChild(btn);
    }
    const calCount = Object.keys(fits).length;
    $('calibrated-count-badge').textContent = `已标定 ${calCount} / 6 轴`;
  }

  // Canvas Plotting Helpers
  function prepareCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    if (!canvas.dataset.logicalHeight) {
      canvas.dataset.logicalHeight = String(Number(canvas.getAttribute('height')) || 190);
    }
    const height = Number(canvas.dataset.logicalHeight);
    canvas.style.height = `${height}px`;
    const parentWidth = canvas.parentElement ? canvas.parentElement.clientWidth - 20 : 640;
    const width = Math.max(320, Math.floor(parentWidth));
    const targetW = Math.round(width * dpr);
    const targetH = Math.round(height * dpr);
    if (canvas.width !== targetW || canvas.height !== targetH) {
      canvas.width = targetW;
      canvas.height = targetH;
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    return { ctx, width, height };
  }

  function drawLineSeries(ctx, xs, ys, xMap, yMap, color, lineWidth = 1.6, dash = []) {
    if (!xs || !ys || xs.length === 0) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
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

  function drawAxesBox(ctx, pad, w, h, xMin, xMax, yMin, yMax, xUnit, yUnit) {
    const plotW = w - pad.l - pad.r;
    const plotH = h - pad.t - pad.b;
    ctx.save();
    ctx.strokeStyle = '#dde5ef';
    ctx.fillStyle = '#586a7e';
    ctx.font = '10px ui-monospace, monospace';
    ctx.lineWidth = 1;

    // Horizontal grid lines
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

    // Zero line if in range
    if (yMin < 0 && yMax > 0) {
      const y0 = pad.t + ((yMax - 0) / (yMax - yMin)) * plotH;
      ctx.strokeStyle = '#b9c7d8';
      ctx.beginPath();
      ctx.moveTo(pad.l, y0);
      ctx.lineTo(w - pad.r, y0);
      ctx.stroke();
    }

    // X ticks
    const xTicks = 5;
    for (let i = 0; i <= xTicks; i++) {
      const val = xMin + ((xMax - xMin) * i) / xTicks;
      const px = pad.l + (plotW * i) / xTicks;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText(`${val.toFixed(2)}${i === xTicks ? ' ' + xUnit : ''}`, px, h - pad.b + 4);
    }

    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(yUnit, 6, 4);
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
        ctx.fillText('暂无拟合曲线 — 请点击右上角“⚡ 加载整臂 J1~J6 示例数据”或执行一次扫描采集', width / 2, height / 2);
      });
      return;
    }

    const curves = fit.curves;
    const th = curves.theta_grid;
    const thMin = th[0];
    const thMax = th[th.length - 1];
    const pad = { l: 48, r: 18, t: 14, b: 22 };

    // Chart 1: Cogging + Forward/Reverse
    {
      const { ctx, width, height } = c1;
      const allY = [...curves.tau_pos_low, ...curves.tau_neg_low, ...curves.half_sum_low, ...curves.cogging_fit_low];
      const yAbs = Math.max(0.05, ...allY.map((v) => Math.abs(v))) * 1.12;
      const xMap = (x) => pad.l + ((x - thMin) / Math.max(1e-6, thMax - thMin)) * (width - pad.l - pad.r);
      const yMap = (y) => pad.t + ((yAbs - y) / (2 * yAbs)) * (height - pad.t - pad.b);
      drawAxesBox(ctx, pad, width, height, thMin, thMax, -yAbs, yAbs, 'rad', 'Nm');
      drawLineSeries(ctx, th, curves.tau_pos_low, xMap, yMap, '#9eb4cf', 1.1, [3, 2]);
      drawLineSeries(ctx, th, curves.tau_neg_low, xMap, yMap, '#d2aba0', 1.1, [3, 2]);
      drawLineSeries(ctx, th, curves.half_sum_low, xMap, yMap, '#182537', 1.4);
      drawLineSeries(ctx, th, curves.cogging_fit_low, xMap, yMap, '#245cbe', 2.2);
    }

    // Chart 2: Modulation
    {
      const { ctx, width, height } = c2;
      const allY = [...curves.modulation_actual_low, ...curves.modulation_fit_low];
      const yAbs = Math.max(0.02, ...allY.map((v) => Math.abs(v))) * 1.18;
      const xMap = (x) => pad.l + ((x - thMin) / Math.max(1e-6, thMax - thMin)) * (width - pad.l - pad.r);
      const yMap = (y) => pad.t + ((yAbs - y) / (2 * yAbs)) * (height - pad.t - pad.b);
      drawAxesBox(ctx, pad, width, height, thMin, thMax, -yAbs, yAbs, 'rad', 'Nm');
      drawLineSeries(ctx, th, curves.modulation_actual_low, xMap, yMap, '#76899f', 1.3);
      drawLineSeries(ctx, th, curves.modulation_fit_low, xMap, yMap, '#15714d', 2.2);
    }

    // Chart 3: Stribeck + Envelope
    {
      const { ctx, width, height } = c3;
      const pad3 = { l: 48, r: 42, t: 14, b: 22 };
      const vMax = curves.v_dense[curves.v_dense.length - 1] || 1.6;
      const fMax = Math.max(0.1, ...curves.mean_frictions, ...curves.stribeck_dense) * 1.15;
      const xMap = (v) => pad3.l + (v / vMax) * (width - pad3.l - pad3.r);
      const yMapF = (f) => pad3.t + ((fMax - f) / fMax) * (height - pad3.t - pad3.b);
      const yMapW = (w) => pad3.t + (1.0 - Math.max(0, Math.min(1.1, w))) * (height - pad3.t - pad3.b);

      drawAxesBox(ctx, pad3, width, height, 0, vMax, 0, fMax, 'rad/s', 'F(v) Nm');
      drawLineSeries(ctx, curves.v_dense, curves.stribeck_dense, xMap, yMapF, '#245cbe', 2.2);
      drawScatter(ctx, curves.speeds, curves.mean_frictions, xMap, yMapF, '#184287', 3.8);

      drawLineSeries(ctx, curves.v_dense, curves.envelope_dense, xMap, yMapW, '#d96b27', 2.0, [5, 3]);
      drawScatter(ctx, curves.speeds, curves.ripple_scales, xMap, yMapW, '#d96b27', 3.4);

      // Right axis label for envelope w(v)
      ctx.save();
      ctx.fillStyle = '#d96b27';
      ctx.font = '10px ui-monospace, monospace';
      ctx.textAlign = 'right';
      ctx.fillText('w(v) 1.0', width - 4, pad3.t + 4);
      ctx.fillText('w(v) 0.0', width - 4, height - pad3.b - 6);
      ctx.restore();
    }
  }

  function renderFitSection() {
    const arm = selectedArm();
    const fit = (state.jointFits[arm.id] || {})[state.activeJoint] || null;
    if (!fit) {
      $('val-fc').textContent = '—';
      $('val-fs').textContent = '—';
      $('val-vs').textContent = '—';
      $('val-fv').textContent = '—';
      if ($('val-v0')) {
        const v0Rep = ((state.v0Reports || {})[arm.id] || {})[state.activeJoint];
        $('val-v0').textContent = v0Rep ? Number(v0Rep.v0).toFixed(4) : '—';
      }
      $('val-vr').textContent = '—';
      $('val-rmse').textContent = '—';
      $('harmonics-tbody').innerHTML = '<tr><td colspan="9" class="empty-cell">当前关节尚未拟合，请点击右上角“⚡ 加载整臂 J1~J6 示例数据”或启动该轴扫描</td></tr>';
      renderCharts(null);
      const nextYaml = buildYamlTextForArm(arm.id);
      if ($('yaml-output-preview').textContent !== nextYaml) {
        $('yaml-output-preview').textContent = nextYaml;
      }
      updateEvaluationBox();
      return;
    }

    const m = fit.model;
    $('val-fc').textContent = m.fc.toFixed(4);
    $('val-fs').textContent = m.fs.toFixed(4);
    $('val-vs').textContent = m.vs.toFixed(4);
    $('val-fv').textContent = m.fv.toFixed(4);
    if ($('val-v0')) $('val-v0').textContent = Number(m.v0).toFixed(4);
    $('val-vr').textContent = m.vr.toFixed(4);
    $('val-rmse').textContent = fit.metrics.rmse_total.toFixed(4);

    const hbody = $('harmonics-tbody');
    hbody.innerHTML = '';
    if (!fit.harmonics || fit.harmonics.length === 0) {
      hbody.innerHTML = '<tr><td colspan="9" class="empty-cell">未检出显著空间周期谐波</td></tr>';
    } else {
      fit.harmonics.forEach((h, idx) => {
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td><strong>H${idx + 1}</strong></td>
          <td>${h.omega.toFixed(2)}</td>
          <td>${h.period_deg.toFixed(2)}°</td>
          <td>${h.a >= 0 ? '+' : ''}${h.a.toFixed(5)}</td>
          <td>${h.b >= 0 ? '+' : ''}${h.b.toFixed(5)}</td>
          <td><strong>${h.cogging_amplitude.toFixed(5)}</strong></td>
          <td>${h.c >= 0 ? '+' : ''}${h.c.toFixed(5)}</td>
          <td>${h.d >= 0 ? '+' : ''}${h.d.toFixed(5)}</td>
          <td><strong>${h.modulation_amplitude.toFixed(5)}</strong></td>
        `;
        hbody.appendChild(tr);
      });
    }

    renderCharts(fit);
    const nextYaml = fit.yaml_text || buildYamlTextForArm(arm.id);
    if ($('yaml-output-preview').textContent !== nextYaml) {
      $('yaml-output-preview').textContent = nextYaml;
    }
    updateEvaluationBox();
  }

  function updateEvaluationBox() {
    const theta = Number.parseFloat($('slider-eval-q').value) || 0;
    const dq = Number.parseFloat($('slider-eval-dq').value) || 0;
    const cogScale = Number.parseFloat($('input-cog-scale').value) || 1.0;
    const fricScale = Number.parseFloat($('input-fric-scale').value) || 0.8;
    $('lbl-eval-q').textContent = `${theta.toFixed(3)} rad / ${radToDeg(theta).toFixed(1)}°`;
    $('lbl-eval-dq').textContent = `${dq.toFixed(3)} rad/s`;

    const arm = selectedArm();
    const fit = (state.jointFits[arm.id] || {})[state.activeJoint];
    if (!fit || !fit.model) {
      $('eval-env').textContent = '—';
      $('eval-cog').textContent = '0.0000 Nm';
      $('eval-mod').textContent = '0.0000 Nm';
      $('eval-stribeck').textContent = '0.0000 Nm';
      $('eval-total').textContent = '0.0000 Nm';
      return;
    }

    const m = fit.model;
    const env = Math.exp(-Math.pow(dq / m.vr, 2));
    const cogRaw = (m.harmonics || []).reduce((acc, h) => acc + h.a * Math.cos(h.omega * theta) + h.b * Math.sin(h.omega * theta), 0);
    const modRaw = (m.harmonics || []).reduce((acc, h) => acc + h.c * Math.cos(h.omega * theta) + h.d * Math.sin(h.omega * theta), 0);
    const signSmooth = Math.tanh(dq / m.v0);
    const dry = m.fc + (m.fs - m.fc) * Math.exp(-Math.pow(Math.abs(dq) / m.vs, m.delta));
    const stribeck = (dry * signSmooth + m.fv * dq) * fricScale;
    const cog = env * cogRaw * cogScale;
    const mod = env * modRaw * signSmooth * fricScale;
    const total = stribeck + cog + mod;

    $('eval-env').textContent = env.toFixed(4);
    $('eval-cog').textContent = `${cog >= 0 ? '+' : ''}${cog.toFixed(4)} Nm`;
    $('eval-mod').textContent = `${mod >= 0 ? '+' : ''}${mod.toFixed(4)} Nm`;
    $('eval-stribeck').textContent = `${stribeck >= 0 ? '+' : ''}${stribeck.toFixed(4)} Nm`;
    $('eval-total').textContent = `${total >= 0 ? '+' : ''}${total.toFixed(4)} Nm`;
  }

  async function startOnlineSweep(jointsList) {
    const arm = selectedArm();
    const speeds = parseFloatList($('input-speeds').value);
    const repeats = Math.max(1, Number.parseInt($('input-repeats').value, 10) || 1);
    const settle_time = Math.max(0, Number.parseFloat($('input-settle').value) || 0.4);
    const kp = parseFloatList($('input-kp').value);
    const output_csv = $('input-output-csv').value.trim();
    const fit_output = $('input-fit-output').value.trim();

    const joints_plan = jointsList.map((j) => ({
      joint: j,
      q_min: state.jointPlan[j].q_min,
      q_max: state.jointPlan[j].q_max,
    }));

    try {
      const res = await fetch('/api/sweep/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          namespace: arm.namespace,
          joints_plan,
          speeds,
          repeats,
          settle_time,
          kp,
          output_csv,
          fit_output,
        }),
      });
      const data = await res.json();
      if (!res.ok || !data.ok) {
        throw new Error(data.error || '启动扫描失败');
      }
      showNotice(`已向 ${arm.can_port} (${arm.namespace}) 下发扫描队列：[${jointsList.map((j) => 'J' + j).join(' → ')}]`, 'ok');
      await pollStatus();
    } catch (err) {
      showNotice(`无法启动在线扫描（请确认已启动 ros2 run piper_friction web 与对应臂驱动）：${err.message}`, 'error');
    }
  }

  async function loadDemoAllJoints() {
    const arm = selectedArm();
    const maxHarmonics = Number.parseInt($('input-max-harmonics').value, 10) || 2;
    const omegas = parseFloatList($('input-omegas').value);
    try {
      if (state.serverOnline) {
        const res = await fetch('/api/fit', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            source: 'demo_all',
            arm_id: arm.id,
            joint: state.activeJoint,
            max_harmonics: maxHarmonics,
            omegas: omegas.length > 0 ? omegas : null,
            output_yaml: $('input-fit-output').value.trim() || null,
          }),
        });
        const data = await res.json();
        if (res.ok && data.ok && data.joint_fits) {
          state.jointFits[arm.id] = data.joint_fits;
          renderAll();
          showNotice(`已完成 ${arm.can_port} (${arm.id}) 全六轴 J1~J6 示例数据解耦与参数拟合`, 'ok');
          return;
        }
      }
    } catch (_) {
      // Fall through to client-side synthesis
    }
    const localFits = {};
    for (let j = 1; j <= 6; j++) {
      localFits[j] = buildClientFallbackFit(arm.id, j);
    }
    state.jointFits[arm.id] = localFits;
    const yamlText = buildYamlTextForArm(arm.id);
    for (let j = 1; j <= 6; j++) {
      state.jointFits[arm.id][j].yaml_text = yamlText;
    }
    renderAll();
    showNotice(`已在浏览器本地生成 ${arm.can_port} (${arm.id}) 全六轴 J1~J6 辨识曲线与 YAML 参数`, 'ok');
  }

  async function runFitSingle(source = 'memory', csvText = '') {
    const arm = selectedArm();
    const maxHarmonics = Number.parseInt($('input-max-harmonics').value, 10) || 2;
    const omegas = parseFloatList($('input-omegas').value);
    try {
      const res = await fetch('/api/fit', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          source,
          arm_id: arm.id,
          joint: state.activeJoint,
          q_min: state.jointPlan[state.activeJoint].q_min,
          q_max: state.jointPlan[state.activeJoint].q_max,
          max_harmonics: maxHarmonics,
          omegas: omegas.length > 0 ? omegas : null,
          csv_text: csvText,
          output_yaml: $('input-fit-output').value.trim() || null,
        }),
      });
      const data = await res.json();
      if (!res.ok || !data.ok) {
        throw new Error(data.error || '拟合失败');
      }
      if (data.joint_fits) {
        state.jointFits[arm.id] = data.joint_fits;
      } else if (data.fit) {
        state.jointFits[arm.id][data.fit.joint] = data.fit;
      }
      renderAll();
      showNotice(`关节 J${state.activeJoint} 参数辨识完成 (RMSE=${data.fit.metrics.rmse_total.toFixed(5)} Nm)`, 'ok');
    } catch (err) {
      if (source === 'demo') {
        state.jointFits[arm.id][state.activeJoint] = buildClientFallbackFit(arm.id, state.activeJoint);
        state.jointFits[arm.id][state.activeJoint].yaml_text = buildYamlTextForArm(arm.id);
        renderAll();
        showNotice(`已在本地生成 J${state.activeJoint} 示例辨识结果`, 'ok');
      } else {
        showNotice(`拟合请求失败：${err.message}`, 'error');
      }
    }
  }

  async function pollStatus() {
    try {
      const res = await fetch('/api/status', { cache: 'no-store' });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const data = await res.json();
      state.serverOnline = true;
      $('conn-dot').className = 'dot online';
      $('conn-text').textContent = 'ROS 2 标定服务在线';
      if (Array.isArray(data.arms) && data.arms.length > 0) {
        state.arms = data.arms;
      }
      if (data.sweeps_by_ns) {
        state.sweepsByNs = data.sweeps_by_ns;
        const curNs = selectedArm().namespace;
        if (data.sweeps_by_ns[curNs]) {
          state.sweep = data.sweeps_by_ns[curNs];
        }
      } else if (data.sweep) {
        state.sweep = data.sweep;
      }
      let fitsChanged = false;
      if (data.joint_fits) {
        for (const [armId, fits] of Object.entries(data.joint_fits)) {
          const keys = Object.keys(fits);
          if (keys.length > 0) {
            const prevJson = JSON.stringify(state.jointFits[armId] || {});
            const nextJson = JSON.stringify(fits);
            if (prevJson !== nextJson) {
              state.jointFits[armId] = fits;
              fitsChanged = true;
            }
          }
        }
      }
      if (data.v0_reports) {
        const prevV0 = JSON.stringify(state.v0Reports || {});
        const nextV0 = JSON.stringify(data.v0_reports);
        if (prevV0 !== nextV0) {
          state.v0Reports = data.v0_reports;
          fitsChanged = true;
        }
      }
      renderArmSelector();
      renderLiveTelemetry();
      renderSweepProgress();
      if (fitsChanged) {
        renderJointTabs();
        renderFitSection();
      }
    } catch (_) {
      state.serverOnline = false;
      $('conn-dot').className = 'dot offline';
      $('conn-text').textContent = '离线演示模式（可随时加载示例数据）';
      renderArmSelector();
      renderLiveTelemetry();
      renderSweepProgress();
    }
  }

  function renderAll() {
    renderArmSelector();
    renderLiveTelemetry();
    renderPlanTable();
    renderSweepProgress();
    renderJointTabs();
    renderFitSection();
  }

  function bindEvents() {
    $('btn-demo-all').addEventListener('click', loadDemoAllJoints);
    $('btn-demo-single').addEventListener('click', () => runFitSingle('demo'));
    $('btn-run-fit').addEventListener('click', () => runFitSingle('memory'));

    if ($('btn-zero-friction-scale')) {
      $('btn-zero-friction-scale').addEventListener('click', () => {
        setDriverFrictionScale(selectedArm().namespace, 0.0, false);
      });
    }
    if ($('btn-restore-friction-scale')) {
      $('btn-restore-friction-scale').addEventListener('click', () => {
        setDriverFrictionScale(selectedArm().namespace, 1.0, false);
      });
    }

    $('btn-select-all-joints').addEventListener('click', () => {
      const allChecked = [1, 2, 3, 4, 5, 6].every((j) => state.jointPlan[j].checked);
      for (let j = 1; j <= 6; j++) {
        state.jointPlan[j].checked = !allChecked;
      }
      renderPlanTable();
    });

    $('btn-center-around-live').addEventListener('click', () => {
      const arm = selectedArm();
      if (!arm.position) {
        showNotice('当前暂无真机角度反馈，保留默认安全范围', 'warn');
        return;
      }
      for (let j = 1; j <= 6; j++) {
        const q0 = arm.position[j - 1];
        state.jointPlan[j].q_min = Number((q0 - 0.40).toFixed(2));
        state.jointPlan[j].q_max = Number((q0 + 0.40).toFixed(2));
      }
      renderPlanTable();
      showNotice(`已按 ${arm.can_port} 当前实测角 ±0.40 rad 更新 J1~J6 扫描区间`, 'ok');
    });

    $('btn-start-queue').addEventListener('click', () => {
      const chosen = [1, 2, 3, 4, 5, 6].filter((j) => state.jointPlan[j].checked);
      if (chosen.length === 0) {
        showNotice('请至少勾选一个需要标定的关节', 'error');
        return;
      }
      startOnlineSweep(chosen);
    });

    $('btn-start-single').addEventListener('click', () => {
      startOnlineSweep([state.activeJoint]);
    });

    if ($('btn-start-v0')) {
      $('btn-start-v0').addEventListener('click', async () => {
        const arm = selectedArm();
        const chosen = [1, 2, 3, 4, 5, 6].filter((j) => state.jointPlan[j].checked);
        const joints = chosen.length > 0 ? chosen : [1, 2, 3, 4, 5, 6];
        const fit_output = $('input-fit-output').value.trim();
        try {
          const res = await fetch('/api/v0/start', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ namespace: arm.namespace, joints, fit_output }),
          });
          const data = await res.json();
          if (!res.ok || !data.ok) throw new Error(data.error || '启动 v0 测算失败');
          showNotice(`已启动 ${arm.namespace} 闭环破拆脉冲 + 递减搜寻临界 v0 (~5s)，找到起振临界点后自动收敛`, 'ok');
          await pollStatus();
        } catch (err) {
          showNotice(`无法启动 v0 测算：${err.message}`, 'error');
        }
      });
    }

    $('btn-stop-sweep').addEventListener('click', async () => {
      try {
        const arm = selectedArm();
        await fetch('/api/sweep/stop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ namespace: arm.namespace }),
        });
        showNotice(`已停止 ${arm.namespace} 扫描并保持当前位置`, 'warn');
        await pollStatus();
      } catch (err) {
        showNotice(`停止请求失败：${err.message}`, 'error');
      }
    });

    $('input-csv-file').addEventListener('change', async (e) => {
      const file = e.target.files && e.target.files[0];
      if (!file) return;
      const text = await file.text();
      await runFitSingle('csv_text', text);
      e.target.value = '';
    });

    ['input-speeds', 'input-repeats', 'input-settle', 'input-kp', 'input-output-csv', 'input-fit-output'].forEach((id) => {
      $(id).addEventListener('input', updateCliPreview);
    });

    ['slider-eval-q', 'slider-eval-dq', 'input-cog-scale', 'input-fric-scale'].forEach((id) => {
      $(id).addEventListener('input', updateEvaluationBox);
    });

    $('btn-copy-cli').addEventListener('click', () => {
      navigator.clipboard.writeText($('cli-command-preview').textContent || '');
      showNotice('已复制 CLI 命令到剪贴板', 'ok');
    });

    $('btn-copy-yaml').addEventListener('click', () => {
      navigator.clipboard.writeText($('yaml-output-preview').textContent || '');
      showNotice('已复制 YAML 参数到剪贴板', 'ok');
    });

    $('btn-download-yaml').addEventListener('click', () => {
      const arm = selectedArm();
      const yaml = $('yaml-output-preview').textContent || '';
      const blob = new Blob([yaml], { type: 'text/yaml;charset=utf-8' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = `${arm.id}_friction.yaml`;
      a.click();
      URL.revokeObjectURL(url);
    });

    $('input-fit-output').addEventListener('input', (e) => {
      if ($('input-save-yaml-path')) {
        $('input-save-yaml-path').value = e.target.value;
      }
    });
    if ($('input-save-yaml-path')) {
      $('input-save-yaml-path').addEventListener('input', (e) => {
        $('input-fit-output').value = e.target.value;
        updateCliPreview();
      });
    }

    $('btn-save-yaml').addEventListener('click', async () => {
      const arm = selectedArm();
      const pathInput = $('input-save-yaml-path') || $('input-fit-output');
      const path = pathInput.value.trim();
      const fb = $('save-yaml-feedback');
      const nowStr = new Date().toTimeString().slice(0, 8);
      try {
        const res = await fetch('/api/save_yaml', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            arm_id: arm.id,
            path,
            joints: state.jointFits[arm.id] || {},
          }),
        });
        const data = await res.json();
        if (!res.ok || !data.ok) throw new Error(data.error || '保存失败');
        const msg = `✓ [${nowStr}] 已保存至服务器路径：${data.path}`;
        if (fb) {
          fb.hidden = false;
          fb.className = 'save-feedback';
          fb.textContent = msg;
        }
        showNotice(msg, 'ok');
      } catch (err) {
        const msg = `✕ [${nowStr}] 保存失败：${err.message}（也可点击右侧“⬇ 下载 YAML 文件”）`;
        if (fb) {
          fb.hidden = false;
          fb.className = 'save-feedback error';
          fb.textContent = msg;
        }
        showNotice(msg, 'error');
      }
    });

    window.addEventListener('resize', () => {
      const arm = selectedArm();
      renderCharts((state.jointFits[arm.id] || {})[state.activeJoint] || null);
    });
  }

  // Initialize
  updateClock();
  setInterval(updateClock, 1000);
  bindEvents();
  renderAll();
  pollStatus().finally(() => {
    setDriverFrictionScale(selectedArm().namespace, 0.0, true);
    if (!state.serverOnline) {
      loadDemoAllJoints();
    }
  });
  setInterval(pollStatus, 400);
})();
