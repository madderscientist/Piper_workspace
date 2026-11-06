(() => {
  const JOINT_NOTES = {
    1: 'J1 绕竖直轴旋转，重力矩理论为 0（仅测静摩擦 Fs）',
    2: 'J2 肩部俯仰主承重轴，重点标定 scale / angle_offset / bias',
    3: 'J3 肘部俯仰主承重轴，重点标定 scale / angle_offset / bias',
    4: 'J4 前臂滚转轴，受前端姿态耦合重力影响',
    5: 'J5 腕部俯仰轴，受末端夹爪/手柄重心影响',
    6: 'J6 法兰旋转轴，重力矩极小（主要测静摩擦 Fs）',
  };

  const DEFAULT_ARMS = [
    { id: 'arm_1', name: '机械臂 1', namespace: '/arm_1', can_port: 'can0', online: false, mode: 'mit', calibrated_joints: [] },
    { id: 'arm_2', name: '机械臂 2', namespace: '/arm_2', can_port: 'can1', online: false, mode: 'mit', calibrated_joints: [] },
    { id: 'arm_3', name: '机械臂 3', namespace: '/arm_3', can_port: 'can2', online: false, mode: 'mit', calibrated_joints: [] },
    { id: 'arm_4', name: '机械臂 4', namespace: '/arm_4', can_port: 'can3', online: false, mode: 'mit', calibrated_joints: [] },
  ];

  const state = {
    selectedArmId: 'arm_1',
    selectedNamespace: '/arm_1',
    activeJointTab: 2,
    arms: DEFAULT_ARMS,
    defaultRanges: {
      1: [-0.50, 0.50],
      2: [0.55, 1.20],
      3: [-1.80, -1.15],
      4: [-0.45, 0.45],
      5: [-0.40, 0.40],
      6: [-0.50, 0.50],
    },
    planEnabled: { 1: false, 2: true, 3: true, 4: true, 5: true, 6: false },
    jointFits: {},
    models: {},
  };

  const el = (id) => document.getElementById(id);

  function renderArmSelector() {
    const container = el('arm-selector');
    if (!container) return;
    container.innerHTML = '';
    const armsList = state.arms && state.arms.length ? state.arms : DEFAULT_ARMS;
    armsList.forEach((arm) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = `arm-card-btn ${arm.id === state.selectedArmId ? 'selected' : ''}`;
      const calCount = (arm.calibrated_joints || []).length;
      const pPhase = arm.probe_phase || 'idle';
      const isRunning = ['waiting_feedback', 'zeroing_compensation', 'enabling', 'setting_mode', 'running', 'restoring_compensation'].includes(pPhase);
      const statusDot = isRunning ? '⚡ 采集中' : (arm.online ? '🟢 在线' : '⚪ 离线/待命');
      btn.innerHTML = `
        <div class="arm-card-top">
          <span>${String(arm.can_port || '').toUpperCase()}</span>
          <span>${statusDot}</span>
        </div>
        <div class="arm-card-name">${arm.name} (${arm.namespace})</div>
        <div class="arm-card-meta">已标定 ${calCount}/6 轴 · 状态 ${pPhase}</div>
      `;
      btn.addEventListener('click', () => {
        state.selectedArmId = arm.id;
        state.selectedNamespace = arm.namespace;
        const gPath = `/home/a/piper_x_ws/src/piper_gravity/config/${arm.id}_gravity.yaml`;
        const fPath = `/home/a/piper_x_ws/src/piper_friction/config/${arm.id}_friction.yaml`;
        if (el('input-output-yaml')) el('input-output-yaml').value = gPath;
        if (el('input-save-gravity-path')) el('input-save-gravity-path').value = gPath;
        if (el('input-sync-friction-path')) el('input-sync-friction-path').value = fPath;
        if (el('selected-arm-summary')) {
          el('selected-arm-summary').textContent = `${arm.can_port} · ${arm.namespace}`;
        }
        renderArmSelector();
        renderJointTabs();
        updateFitView();
      });
      container.appendChild(btn);
    });
    const cur = armsList.find((a) => a.id === state.selectedArmId) || armsList[0];
    if (cur && el('selected-arm-summary')) {
      el('selected-arm-summary').textContent = `${cur.can_port} · ${cur.namespace}`;
    }
  }

  function notify(msg, isError = false) {
    const bar = el('notice-bar');
    if (!bar) return;
    bar.hidden = false;
    bar.textContent = msg;
    bar.style.background = isError ? '#fff1f3' : '#e9f7f0';
    bar.style.color = isError ? '#b42332' : '#15714d';
  }

  function renderPlanTable() {
    const tbody = el('plan-tbody');
    tbody.innerHTML = '';
    for (let j = 1; j <= 6; j++) {
      const [lo, hi] = state.defaultRanges[j] || [-0.4, 0.4];
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><input type="checkbox" data-plan-check="${j}" ${state.planEnabled[j] ? 'checked' : ''} /></td>
        <td><strong>J${j}</strong></td>
        <td><input type="number" step="0.05" data-plan-min="${j}" value="${lo.toFixed(2)}" style="width:76px" /></td>
        <td><input type="number" step="0.05" data-plan-max="${j}" value="${hi.toFixed(2)}" style="width:76px" /></td>
        <td style="color:#586a7e;font-size:11px">${JOINT_NOTES[j]}</td>
        <td><button type="button" class="btn-xs" data-single-probe="${j}">▶ 仅测 J${j}</button></td>
      `;
      tbody.appendChild(tr);
    }
    tbody.querySelectorAll('[data-single-probe]').forEach((btn) => {
      btn.addEventListener('click', () => startProbe([Number(btn.dataset.singleProbe)]));
    });
  }

  function renderJointTabs() {
    const box = el('joint-tabs');
    box.innerHTML = '';
    for (let j = 1; j <= 6; j++) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'btn-xs' + (state.activeJointTab === j ? ' btn-primary' : '');
      const hasFit = state.jointFits[state.selectedArmId] && state.jointFits[state.selectedArmId][String(j)];
      btn.textContent = `J${j}${hasFit ? ' ✓' : ''}`;
      btn.addEventListener('click', () => {
        state.activeJointTab = j;
        renderJointTabs();
        updateFitView();
      });
      box.appendChild(btn);
    }
  }

  function drawChart(canvasId, theta, seriesList) {
    const canvas = el(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    if (!theta || theta.length < 2) return;

    const pad = { l: 48, r: 16, t: 14, b: 24 };
    const xMin = Math.min(...theta);
    const xMax = Math.max(...theta);
    let yMin = Infinity;
    let yMax = -Infinity;
    seriesList.forEach((s) => {
      (s.data || []).forEach((v) => {
        if (Number.isFinite(v)) {
          if (v < yMin) yMin = v;
          if (v > yMax) yMax = v;
        }
      });
    });
    if (!Number.isFinite(yMin) || yMax - yMin < 1e-4) {
      yMin = -0.5;
      yMax = 0.5;
    } else {
      const margin = 0.1 * (yMax - yMin);
      yMin -= margin;
      yMax += margin;
    }

    const xToPx = (x) => pad.l + ((x - xMin) / Math.max(1e-6, xMax - xMin)) * (W - pad.l - pad.r);
    const yToPx = (y) => H - pad.b - ((y - yMin) / Math.max(1e-6, yMax - yMin)) * (H - pad.t - pad.b);

    ctx.strokeStyle = '#e6ecf2';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const yv = yMin + (i / 4) * (yMax - yMin);
      const py = yToPx(yv);
      ctx.beginPath();
      ctx.moveTo(pad.l, py);
      ctx.lineTo(W - pad.r, py);
      ctx.stroke();
      ctx.fillStyle = '#586a7e';
      ctx.font = '10px ui-monospace, monospace';
      ctx.fillText(yv.toFixed(2), 4, py + 3);
    }

    seriesList.forEach((s) => {
      if (!s.data || !s.data.length) return;
      ctx.save();
      ctx.strokeStyle = s.color;
      ctx.lineWidth = s.width || 1.8;
      if (s.dash) ctx.setLineDash(s.dash);
      ctx.beginPath();
      s.data.forEach((yv, idx) => {
        const px = xToPx(theta[idx]);
        const py = yToPx(yv);
        if (idx === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      ctx.restore();
    });
  }

  function updateFitView() {
    const armFits = state.jointFits[state.selectedArmId] || {};
    const fit = armFits[String(state.activeJointTab)];
    const modelYaml = (state.models && state.models[state.selectedArmId]) || (fit && fit.yaml_text) || '';
    if (modelYaml && el('yaml-preview').textContent !== modelYaml) {
      el('yaml-preview').textContent = modelYaml;
    }
    el('payload-mass-card').hidden = !fit?.payload;
    if (!fit) return;
    if (fit.payload) {
      const p = fit.payload;
      el('kpi-payload-mass').textContent = `${p.added_mass_kg.toFixed(4)} / ${p.total_mass_kg.toFixed(4)} kg${p.mass_identified ? '' : '（待重力轴数据）'}`;
    }
    const m = fit.model;
    const met = fit.metrics;
    el('kpi-scale').textContent = m.scale.toFixed(4);
    el('kpi-offset').textContent = `${m.angle_offset >= 0 ? '+' : ''}${m.angle_offset_deg.toFixed(2)}°`;
    el('kpi-bias').textContent = `${m.bias >= 0 ? '+' : ''}${m.bias.toFixed(4)} Nm`;
    el('kpi-fs').textContent = `${met.static_friction_mean.toFixed(3)} / ${met.static_friction_p85.toFixed(3)} Nm`;
    el('kpi-rmse').textContent = `${met.rmse_cad.toFixed(3)} → ${met.rmse_calibrated.toFixed(3)} Nm`;

    const c = fit.curves;
    drawChart('canvas-gravity', c.theta_grid, [
      { data: c.tau_pos, color: '#245cbe', dash: [4, 3], width: 1.3 },
      { data: c.tau_neg, color: '#d97706', dash: [4, 3], width: 1.3 },
      { data: c.gravity_cad, color: '#94a3b8', width: 1.8 },
      { data: c.gravity_measured, color: '#0f172a', width: 1.5 },
      { data: c.gravity_fitted, color: '#15714d', width: 2.4 },
    ]);
    const fsMeanLine = c.theta_grid.map(() => met.static_friction_mean);
    drawChart('canvas-stiction', c.theta_grid, [
      { data: c.stiction_profile, color: '#7c3aed', width: 1.8 },
      { data: fsMeanLine, color: '#15714d', dash: [5, 4], width: 1.8 },
    ]);
  }

  async function postJson(url, payload) {
    const resp = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await resp.json();
    if (!resp.ok || data.ok === false) {
      throw new Error(data.error || `HTTP ${resp.status}`);
    }
    return data;
  }

  async function startProbe(jointsList) {
    try {
      const plan = jointsList.map((j) => ({
        joint: j,
        q_min: Number(document.querySelector(`[data-plan-min="${j}"]`).value),
        q_max: Number(document.querySelector(`[data-plan-max="${j}"]`).value),
      }));
      await postJson('/api/probe/start', {
        namespace: state.selectedNamespace,
        joints_plan: plan,
        num_points: Number(el('input-num-points').value),
        pre_travel: Number(el('input-pre-travel').value),
        approach_speed: Number(el('input-approach-speed').value),
        settle_time: Number(el('input-settle-time').value),
        fit_output: el('input-output-yaml').value,
      });
      notify(`已启动 ${state.selectedNamespace} 双向准静态逼近采样: J${jointsList.join(', J')}`);
    } catch (err) {
      notify(err.message, true);
    }
  }

  async function pollStatus() {
    try {
      const resp = await fetch('/api/status');
      const data = await resp.json();
      el('ros-dot').className = 'dot online';
      el('ros-text').textContent = 'ROS 2 已连接';
      if (data.arms && data.arms.length) state.arms = data.arms;
      let fitsChanged = false;
      if (data.models) {
        if (JSON.stringify(state.models) !== JSON.stringify(data.models)) {
          state.models = data.models;
          fitsChanged = true;
        }
      }
      if (data.joint_fits) {
        if (JSON.stringify(state.jointFits) !== JSON.stringify(data.joint_fits)) {
          state.jointFits = data.joint_fits;
          fitsChanged = true;
        }
      }
      renderArmSelector();

      const arm = (data.arms || []).find((a) => a.id === state.selectedArmId) || (data.arms || [])[0];
      if (arm) {
        el('arm-scales-badge').textContent =
          `gravity: ${Number(arm.gravity_scale_applied).toFixed(2)} | friction: ${Number(arm.friction_scale_applied).toFixed(2)}`;
        el('arm-live-status').textContent = arm.online ? `在线 · ${arm.mode}` : '等待反馈';
        const tbody = el('live-joints-tbody');
        tbody.innerHTML = '';
        for (let i = 0; i < 6; i++) {
          const q = arm.position ? arm.position[i].toFixed(3) : '--';
          const tm = arm.torque ? arm.torque[i].toFixed(3) : '--';
          const gc = arm.gravity_cad ? arm.gravity_cad[i].toFixed(3) : '--';
          const gcal = arm.gravity_calibrated ? arm.gravity_calibrated[i].toFixed(3) : '--';
          const done = (arm.calibrated_joints || []).includes(i + 1);
          const tr = document.createElement('tr');
          tr.innerHTML = `
            <td>J${i + 1}</td>
            <td>${arm.enabled && arm.enabled[i] ? 'ON' : 'OFF'}</td>
            <td>${q}</td>
            <td>${tm}</td>
            <td>${gc}</td>
            <td>${gcal}</td>
            <td>${done ? '✓ 已标定' : '默认 CAD'}</td>
          `;
          tbody.appendChild(tr);
        }
      }

      const probe = (data.probes_by_ns && data.probes_by_ns[state.selectedNamespace]) || data.probe;
      if (probe) {
        el('probe-phase-badge').textContent = `${state.selectedNamespace} · ${probe.phase}`;
        el('probe-sample-count').textContent = `${probe.sample_count} 样本`;
        const pct = probe.segment_total > 0
          ? Math.round((100 * probe.segment_index) / probe.segment_total)
          : 0;
        el('probe-progress-fill').style.width = `${pct}%`;
        el('probe-progress-label').textContent =
          `${probe.phase} (${probe.segment_index}/${probe.segment_total})`;
      }
      if (fitsChanged) {
        renderJointTabs();
        updateFitView();
      }
    } catch (_) {
      el('ros-dot').className = 'dot offline';
      el('ros-text').textContent = '离线模式';
    }
  }

  document.addEventListener('DOMContentLoaded', () => {
    renderArmSelector();
    renderPlanTable();
    renderJointTabs();
    setInterval(() => {
      el('clock').textContent = new Date().toTimeString().slice(0, 8);
    }, 1000);

    el('btn-select-gravity-joints').addEventListener('click', () => {
      [1, 2, 3, 4, 5, 6].forEach((j) => {
        state.planEnabled[j] = j >= 2 && j <= 5;
        const cb = document.querySelector(`[data-plan-check="${j}"]`);
        if (cb) cb.checked = state.planEnabled[j];
      });
    });

    el('btn-select-all-joints').addEventListener('click', () => {
      [1, 2, 3, 4, 5, 6].forEach((j) => {
        state.planEnabled[j] = true;
        const cb = document.querySelector(`[data-plan-check="${j}"]`);
        if (cb) cb.checked = true;
      });
    });

    el('btn-start-queue').addEventListener('click', () => {
      const selected = [1, 2, 3, 4, 5, 6].filter((j) => {
        const cb = document.querySelector(`[data-plan-check="${j}"]`);
        return cb && cb.checked;
      });
      if (!selected.length) {
        notify('请至少勾选一个待测关节', true);
        return;
      }
      startProbe(selected);
    });

    el('btn-stop-probe').addEventListener('click', async () => {
      try {
        await postJson('/api/probe/stop', { namespace: state.selectedNamespace });
        notify(`已停止 ${state.selectedNamespace} 双向逼近采样`);
      } catch (err) {
        notify(err.message, true);
      }
    });

    el('btn-zero-scales').addEventListener('click', async () => {
      try {
        await postJson('/api/arm/compensation_scales', {
          namespace: state.selectedNamespace,
          gravity_scale: 0.0,
          friction_scale: 0.0,
        });
        notify('已请求关闭重力与摩擦补偿 (0.0, 0.0)');
      } catch (err) {
        notify(err.message, true);
      }
    });

    el('btn-restore-scales').addEventListener('click', async () => {
      try {
        await postJson('/api/arm/compensation_scales', {
          namespace: state.selectedNamespace,
          gravity_scale: 1.0,
          friction_scale: 0.5,
        });
        notify('已请求恢复重力与摩擦补偿 (1.0, 0.5)');
      } catch (err) {
        notify(err.message, true);
      }
    });

    el('btn-demo-all').addEventListener('click', async () => {
      try {
        const res = await postJson('/api/fit', {
          source: 'demo_all',
          arm_id: state.selectedArmId,
          torque_source: el('select-torque-source').value,
        });
        state.jointFits[state.selectedArmId] = res.joint_fits || {};
        renderJointTabs();
        updateFitView();
        notify('已完成 J1~J6 双向准静态重力与静摩擦仿真数据拟合');
      } catch (err) {
        notify(err.message, true);
      }
    });

    el('btn-fit-csv').addEventListener('click', async () => {
      const fileInput = el('input-csv-file');
      if (!fileInput.files || !fileInput.files[0]) {
        notify('请先选择 CSV 文件', true);
        return;
      }
      try {
        const text = await fileInput.files[0].text();
        const res = await postJson('/api/fit', {
          source: 'csv_text',
          arm_id: state.selectedArmId,
          torque_source: el('select-torque-source').value,
          csv_text: text,
        });
        const j = res.fit.joint;
        state.jointFits[state.selectedArmId] = state.jointFits[state.selectedArmId] || {};
        state.jointFits[state.selectedArmId][String(j)] = res.fit;
        state.activeJointTab = j;
        renderJointTabs();
        updateFitView();
        notify(`已从 CSV 拟合 J${j} 重力与静摩擦参数`);
      } catch (err) {
        notify(err.message, true);
      }
    });

    el('btn-save-yaml').addEventListener('click', async () => {
      try {
        const res = await postJson('/api/save', {
          arm_id: state.selectedArmId,
          output_yaml: el('input-save-gravity-path').value,
          update_friction_yaml: el('input-sync-friction-path').value,
        });
        notify(`已保存重力标定文件至 ${res.path}`);
      } catch (err) {
        notify(err.message, true);
      }
    });

    if (el('btn-copy-yaml')) {
      el('btn-copy-yaml').addEventListener('click', async () => {
        const text = el('yaml-preview').textContent || '';
        try {
          await navigator.clipboard.writeText(text);
          notify('已复制 YAML 参数到剪贴板');
        } catch (_) {
          const ta = document.createElement('textarea');
          ta.value = text;
          document.body.appendChild(ta);
          ta.select();
          document.execCommand('copy');
          document.body.removeChild(ta);
          notify('已复制 YAML 参数到剪贴板');
        }
      });
    }

    if (el('btn-download-yaml')) {
      el('btn-download-yaml').addEventListener('click', () => {
        const text = el('yaml-preview').textContent || '';
        const blob = new Blob([text], { type: 'text/yaml;charset=utf-8' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${state.selectedArmId}_gravity.yaml`;
        a.click();
        URL.revokeObjectURL(url);
      });
    }

    pollStatus();
    setInterval(pollStatus, 500);
  });
})();
