(() => {
  'use strict';

  const $ = (id) => document.getElementById(id);
  let lastServerOffsetSig = '';
  let lastServerGripEffort = '';
  let lastServerWrenchSource = '';

  const fmt = (val, digits = 1, unit = '') => (Number.isFinite(val) ? val.toFixed(digits) + unit : '—');

  function setNotice(text, isError = false) {
    const bar = $('notice-bar');
    const span = $('notice-text');
    if (bar && span) {
      span.textContent = text;
      bar.classList.toggle('error', Boolean(isError));
    }
  }

  async function postJson(url, body = {}) {
    const res = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const data = await res.json();
    if (!res.ok || data.ok === false) {
      throw new Error(data.error || data.message || ('HTTP ' + res.status));
    }
    if (data.message) setNotice(data.message, false);
    return data;
  }

  function runPost(url, bodyFn) {
    postJson(url, typeof bodyFn === 'function' ? bodyFn() : bodyFn).catch((e) => setNotice(e.message, true));
  }

  function readTipOffsetCm() {
    return ['input-tip-x-cm', 'input-tip-y-cm', 'input-tip-z-cm'].map((id, i) => {
      const v = Number.parseFloat($(id).value);
      return Number.isFinite(v) ? v : (i === 2 ? 19.5 : 0.0);
    });
  }

  function renderStatus(data) {
    const arm = data.arm || {};
    const grip = data.gripper || {};
    const wrench = data.wrench || {};
    const policy = data.policy || {};
    const goal = data.goal;

    $('can-badge').textContent = (arm.can_label || 'can4').toUpperCase() + ' · ' + (arm.namespace || '/arm_4') + ' (' + (arm.can_port || 'can3') + ')';
    $('conn-pill').dataset.state = arm.online ? 'online' : 'offline';
    $('conn-pill').textContent = arm.online ? 'ROS 在线' : '等待遥测';

    const pLabels = { enabled: '六轴已使能', disabled: '六轴已失能', mixed: '部分使能', unknown: '使能未知' };
    $('power-pill').dataset.state = arm.power_state || 'unknown';
    $('power-pill').textContent = pLabels[arm.power_state] || '使能未知';

    const mLabels = { idle: '待机', drag: '零力拖动中', policy: '策略执行中 (50Hz)', hold: '位置保持' };
    $('mode-pill').dataset.mode = policy.mode || 'idle';
    $('mode-pill').textContent = '模式：' + (mLabels[policy.mode] || policy.mode) + ' (驱动: ' + (arm.driver_mode || '—') + ')';

    if (policy.last_error) setNotice('错误：' + policy.last_error, true);
    else if (policy.status_message) setNotice(policy.status_message, false);

    const gLabels = { idle: '夹爪：待机', open: '夹爪：已打开', closed: '夹爪：已夹紧保持', released: '夹爪：零力释放' };
    $('gripper-state-badge').textContent = gLabels[grip.state] || ('夹爪：' + grip.state);
    $('gripper-state-badge').dataset.tone = grip.state === 'closed' ? 'ok' : '';
    $('grip-actual-mm').textContent = Number.isFinite(arm.gripper_width_mm) ? arm.gripper_width_mm.toFixed(1) + ' mm' : '— mm';
    if (Number.isFinite(grip.effort)) {
      const effStr = grip.effort.toFixed(2);
      if (effStr !== lastServerGripEffort && document.activeElement !== $('input-grip-effort')) {
        lastServerGripEffort = effStr;
        if ($('input-grip-effort')) $('input-grip-effort').value = effStr;
        if ($('grip-effort-val')) $('grip-effort-val').textContent = effStr;
      }
    }

    if (goal) {
      $('goal-status-badge').textContent = '已记录目标';
      $('goal-status-badge').dataset.tone = 'ok';
      $('goal-pos-mm').textContent = '[' + goal.position_mm.map((v) => v.toFixed(1)).join(', ') + '] mm';
      $('goal-rpy-deg').textContent = '[' + goal.rpy_deg.map((v) => v.toFixed(1)).join(', ') + ']°';
      $('goal-joints-deg').textContent = '[' + goal.joints_deg.map((v) => v.toFixed(1)).join('°, ') + '°]';
    } else {
      $('goal-status-badge').textContent = '未记录';
      $('goal-status-badge').dataset.tone = '';
      $('goal-pos-mm').textContent = '尚未记录（请先将插头插入孔中再点击记录）';
      $('goal-rpy-deg').textContent = '—';
      $('goal-joints-deg').textContent = '—';
    }

    $('step-card-1').dataset.active = String(policy.mode === 'drag' && !goal);
    $('step-card-2').dataset.active = String(policy.mode === 'drag' && Boolean(goal));
    $('step-card-3').dataset.active = String(policy.mode === 'policy');

    if (Array.isArray(policy.tip_offset_cm) && policy.tip_offset_cm.length === 3) {
      const sig = policy.tip_offset_cm.map((v) => v.toFixed(3)).join(',');
      if (sig !== lastServerOffsetSig) {
        lastServerOffsetSig = sig;
        $('input-tip-x-cm').value = policy.tip_offset_cm[0].toFixed(1);
        $('input-tip-y-cm').value = policy.tip_offset_cm[1].toFixed(1);
        $('input-tip-z-cm').value = policy.tip_offset_cm[2].toFixed(1);
      }
    }

    const wSource = wrench.source || policy.wrench_source || 'paxini';
    const selSource = $('select-wrench-source');
    if (selSource && wSource !== lastServerWrenchSource && document.activeElement !== selSource) {
      lastServerWrenchSource = wSource;
      selSource.value = wSource;
    }
    const srcBadge = $('wrench-source-badge');
    if (srcBadge) {
      if (wSource === 'paxini') {
        const hz = Number.isFinite(wrench.paxini_hz) ? wrench.paxini_hz.toFixed(0) + ' Hz' : '— Hz';
        const sensorHz = Number.isFinite(wrench.paxini_sensor_hz) ? wrench.paxini_sensor_hz.toFixed(0) + ' Hz' : '— Hz';
        srcBadge.textContent = wrench.paxini_fresh ? ('力传感器 · 驱动 ' + sensorHz + ' · insert 接收 ' + hz) : 'paxini_driver 等待数据';
        srcBadge.dataset.tone = wrench.paxini_fresh ? 'ok' : '';
      } else {
        const hz = Number.isFinite(wrench.estimated_hz) ? wrench.estimated_hz.toFixed(0) + ' Hz' : '— Hz';
        srcBadge.textContent = wrench.estimated_fresh ? ('估计力在线 · ' + hz) : (wrench.estimated_enabled === false ? '估计力：等待开启' : '估计力：等待数据');
        srcBadge.dataset.tone = wrench.estimated_fresh ? 'ok' : '';
      }
    }

    $('policy-state-badge').textContent = mLabels[policy.mode] || policy.mode;
    if (policy.mode === 'policy' && policy.controller === 'rule') {
      const stageLabel = policy.rule_stage === 'press' ? '垂直下压' : '水平+角度对准';
      $('policy-state-badge').textContent = '规则插入：' + stageLabel;
    }
    $('policy-state-badge').dataset.tone = policy.mode === 'policy' ? 'active' : '';
    $('policy-steps-time').textContent = (policy.steps || 0) + ' 步 · ' + (policy.elapsed_s || 0).toFixed(1) + ' s';
    const lat = policy.last_step ? (policy.last_step.compute_ms ?? policy.last_step.latency_ms).toFixed(2) + ' ms' : '— ms';
    $('policy-rate-latency').textContent = (policy.rate_hz || 0).toFixed(1) + ' Hz · ' + lat;
    $('policy-reached').textContent = policy.reached_goal
      ? '✓ 已到位 (保持 ≥5 步)'
      : policy.mode === 'policy' ? '执行中 (' + (policy.reached_goal_steps || 0) + '/5)' : '等待执行';
    $('policy-last-action').textContent = policy.last_step
      ? '[' + policy.last_step.dp_h_mm.map((v) => v.toFixed(2)).join(', ') + '] mm / [' + policy.last_step.dr_h_deg.map((v) => v.toFixed(2)).join(', ') + ']°'
      : '—';

    $('tip-pose-b-tag').textContent = arm.tip_pose
      ? 'B 系尖端: [' + arm.tip_pose.position_mm.map((v) => v.toFixed(1)).join(', ') + '] mm'
      : 'B 系尖端：—';

    const rel = arm.hole_relative;
    const dot = $('xy-tip-dot');
    const zMarker = $('z-tip-marker');
    if (rel) {
      const [dx, dy, dz] = rel.p_ht_mm;
      const cx = Math.max(-35, Math.min(35, dx)).toFixed(2);
      const cy = Math.max(-35, Math.min(35, -dy)).toFixed(2);
      $('rel-dx').textContent = (dx >= 0 ? '+' : '') + dx.toFixed(2) + ' mm';
      $('rel-dy').textContent = (dy >= 0 ? '+' : '') + dy.toFixed(2) + ' mm';
      $('rel-dz').textContent = (dz >= 0 ? '+' : '') + dz.toFixed(2) + ' mm';
      $('rel-xy-norm').textContent = rel.xy_distance_mm.toFixed(2) + ' mm';
      $('rel-total-norm').textContent = rel.total_distance_mm.toFixed(2) + ' mm';
      $('rel-angle-err').textContent = rel.angle_error_deg.toFixed(2) + ' °';
      dot.setAttribute('cx', cx);
      dot.setAttribute('cy', cy);
      dot.dataset.aligned = String(rel.xy_distance_mm <= 3.0);
      if ($('xy-tip-x')) { $('xy-tip-x').setAttribute('x1', cx); $('xy-tip-x').setAttribute('x2', cx); }
      if ($('xy-tip-y')) { $('xy-tip-y').setAttribute('y1', cy); $('xy-tip-y').setAttribute('y2', cy); }
      zMarker.style.bottom = Math.max(5, Math.min(92, 22 + (dz / 50.0) * 70)).toFixed(1) + '%';
      zMarker.dataset.inserted = String(dz <= 0.2);
    } else {
      ['rel-dx', 'rel-dy', 'rel-dz', 'rel-xy-norm', 'rel-total-norm'].forEach((id) => { $(id).textContent = '— mm'; });
      $('rel-angle-err').textContent = '— °';
      dot.setAttribute('cx', '0');
      dot.setAttribute('cy', '0');
      dot.dataset.aligned = 'false';
      zMarker.style.bottom = '22%';
      zMarker.dataset.inserted = 'false';
    }

    const wt = wrench.tip_t || [0, 0, 0, 0, 0, 0];
    ['fx', 'fy', 'fz'].forEach((k, i) => {
      $('w-' + k).textContent = fmt(wt[i], 2);
      if ($('wb-' + k)) $('wb-' + k).style.width = Math.min(100, (Math.abs(wt[i] || 0) / 15.0) * 100).toFixed(1) + '%';
    });
    ['mx', 'my', 'mz'].forEach((k, i) => {
      $('w-' + k).textContent = fmt(wt[i + 3], 3);
      if ($('wb-' + k)) $('wb-' + k).style.width = Math.min(100, (Math.abs(wt[i + 3] || 0) / 2.0) * 100).toFixed(1) + '%';
    });
    $('wrench-summary-tag').textContent = '|F| ' + fmt(wrench.force_norm_n, 2) + ' N · |M| ' + fmt(wrench.moment_norm_nm, 3) + ' N·m';
    if (Number.isFinite(wrench.filter_tau_ms) && $('wrench-card-title')) {
      const srcTag = wSource === 'paxini' ? 'paxini_driver' : '估计力';
      const alphaTag = Number.isFinite(wrench.filter_alpha) ? ' · α=' + wrench.filter_alpha.toFixed(3) : '';
      $('wrench-card-title').textContent = '插头末端六维力旋量 (' + srcTag + ' · 滤波 τ=' + wrench.filter_tau_ms.toFixed(2) + 'ms' + alphaTag + ')';
    }

    const qDeg = arm.joints_deg || [];
    const dq = arm.velocities_rad_s || [];
    const tau = arm.efforts_nm || [];
    const qCmd = policy.last_step ? policy.last_step.q_cmd_deg : [];
    $('joint-tbody').querySelectorAll('tr').forEach((tr, i) => {
      const cells = tr.querySelectorAll('td');
      cells[1].textContent = fmt(qDeg[i], 2);
      cells[2].textContent = fmt(dq[i], 3);
      cells[3].textContent = fmt(tau[i], 3);
      cells[4].textContent = fmt(qCmd[i], 2);
    });

    const rates = arm.rates || {};
    $('driver-rates-tag').textContent = Number.isFinite(rates.loop_hz)
      ? '驱动 ' + rates.loop_hz.toFixed(0) + ' Hz · 补偿 ' + (rates.comp_hz || 0).toFixed(0) + ' Hz'
      : '驱动频率：—';

    if ($('log-list') && Array.isArray(data.logs)) {
      $('log-list').innerHTML = data.logs.slice(-8).reverse()
        .map((l) => '<div class="log-row" data-level="' + l.level + '">[' + l.time + '] ' + l.msg + '</div>')
        .join('');
    }
  }

  function bindEvents() {
    $('btn-enable').addEventListener('click', () => runPost('/api/power', { enable: true }));
    $('btn-disable').addEventListener('click', () => runPost('/api/power', { enable: false }));
    $('btn-grip-close').addEventListener('click', () => runPost('/api/gripper', () => ({
      action: 'close',
      width_m: (Number.parseFloat($('input-close-mm').value) || 0.0) / 1000.0,
      effort: Number.isFinite(Number.parseFloat($('input-grip-effort').value)) ? Number.parseFloat($('input-grip-effort').value) : 2.0,
    })));
    $('btn-grip-open').addEventListener('click', () => runPost('/api/gripper', () => ({
      action: 'open',
      width_m: (Number.parseFloat($('input-open-mm').value) || 50.0) / 1000.0,
      effort: Number.isFinite(Number.parseFloat($('input-grip-effort').value)) ? Number.parseFloat($('input-grip-effort').value) : 2.0,
    })));
    $('btn-grip-release').addEventListener('click', () => runPost('/api/gripper', { action: 'release' }));
    if ($('input-grip-effort')) {
      $('input-grip-effort').addEventListener('input', (ev) => {
        const val = Number.parseFloat(ev.target.value);
        if (Number.isFinite(val)) {
          lastServerGripEffort = val.toFixed(2);
          if ($('grip-effort-val')) $('grip-effort-val').textContent = val.toFixed(2);
          runPost('/api/gripper', { action: 'effort', effort: val });
        }
      });
    }
    $('btn-enter-drag').addEventListener('click', () => runPost('/api/drag', {}));
    $('btn-apply-tip-offset').addEventListener('click', () => runPost('/api/parameters', () => ({ tip_offset_cm: readTipOffsetCm() })));
    $('btn-record-goal').addEventListener('click', () => runPost('/api/goal/record', () => ({ tip_offset_cm: readTipOffsetCm() })));
    $('btn-clear-goal').addEventListener('click', () => runPost('/api/goal/clear', {}));
    if ($('select-wrench-source')) {
      $('select-wrench-source').addEventListener('change', (ev) => {
        lastServerWrenchSource = ev.target.value;
        runPost('/api/parameters', { wrench_source: ev.target.value });
      });
    }
    $('btn-start-policy').addEventListener('click', () => runPost('/api/policy/start', () => ({
      controller: 'onnx',
      wrench_source: $('select-wrench-source') ? $('select-wrench-source').value : undefined,
    })));
    if ($('btn-start-rule')) {
      $('btn-start-rule').addEventListener('click', () => runPost('/api/rule/start', () => ({
        wrench_source: $('select-wrench-source') ? $('select-wrench-source').value : undefined,
      })));
    }
    $('btn-stop-drag').addEventListener('click', () => runPost('/api/policy/stop', { mode: 'drag' }));
    $('btn-stop-hold').addEventListener('click', () => runPost('/api/policy/stop', { mode: 'hold' }));
    $('btn-tare-wrench').addEventListener('click', () => runPost('/api/wrench/tare', {}));
    if ($('btn-save-params')) {
      $('btn-save-params').addEventListener('click', () => runPost('/api/parameters', () => ({
        tip_offset_cm: readTipOffsetCm(),
        wrench_source: $('select-wrench-source') ? $('select-wrench-source').value : undefined,
      })));
    }
  }

  async function pollLoop() {
    try {
      const res = await fetch('/api/status', { cache: 'no-store' });
      if (res.ok) renderStatus(await res.json());
    } catch (_) {
      $('conn-pill').dataset.state = 'offline';
      $('conn-pill').textContent = '服务断开';
    } finally {
      setTimeout(pollLoop, 100);
    }
  }

  document.addEventListener('DOMContentLoaded', () => {
    bindEvents();
    pollLoop();
  });
})();
