(() => {
  const $ = (id) => document.getElementById(id);

  const state = {
    promptsInitialized: false,
    tipInputsInitialized: false,
    wrenchSelectInitialized: false,
    autoStopInitialized: false,
    defaultObjectPrompt: '把黑色夹爪捏着的插头的末端，和对应插孔标记出来。',
    defaultOutputPrompt: '',
    lastCalibStamp: 0,
    lastPercStamp: 0,
    latestInferredOffsetCm: null,
    latestPercImages: null,
    latestPercDet2d: null,
    percImageView: 'gpt',
    wristPreviewEnabled: true,
    perceiving: false,
  };

  function setNotice(msg, isError = false) {
    const bar = $('notice-bar');
    const txt = $('notice-text');
    if (!bar || !txt) return;
    txt.textContent = msg;
    bar.classList.toggle('error', Boolean(isError));
  }

  async function postJson(url, body = {}) {
    const resp = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const data = await resp.json().catch(() => ({}));
    if (!resp.ok || data.ok === false) {
      throw new Error(data.error || data.message || ('HTTP ' + resp.status));
    }
    if (data.message) setNotice(data.message, false);
    return data;
  }

  function fmtArr(arr, digits = 1, unit = '') {
    if (!Array.isArray(arr)) return '—';
    return '[' + arr.map((v) => Number(v).toFixed(digits)).join(', ') + ']' + (unit ? ' ' + unit : '');
  }

  // Lightbox for full-res 1920x1080 inspection
  document.addEventListener('click', (ev) => {
    const target = ev.target;
    const box = $('lightbox');
    if (target && target.classList && target.classList.contains('zoomable') && target.src) {
      const fullUrl = target.dataset && target.dataset.full
        ? (target.dataset.full + '?t=' + Date.now())
        : target.src;
      $('lightbox-img').src = fullUrl;
      box.classList.remove('hidden');
      return;
    }
    if (target && (target.id === 'lightbox' || target.id === 'lightbox-img' || target.classList.contains('lightbox-hint'))) {
      box.classList.add('hidden');
    }
  });

  // Topbar & Prep actions
  $('btn-enable').addEventListener('click', () =>
    postJson('/api/power', { enable: true }).catch((e) => setNotice(e.message, true))
  );
  $('btn-disable').addEventListener('click', () =>
    postJson('/api/power', { enable: false }).catch((e) => setNotice(e.message, true))
  );
  $('btn-top-float').addEventListener('click', () =>
    postJson('/api/drag').catch((e) => setNotice(e.message, true))
  );
  $('btn-enter-drag').addEventListener('click', () =>
    postJson('/api/drag').catch((e) => setNotice(e.message, true))
  );
  $('btn-top-hold').addEventListener('click', () =>
    postJson('/api/policy/stop', { mode: 'hold' }).catch((e) => setNotice(e.message, true))
  );

  // Gripper controls
  $('input-grip-effort').addEventListener('input', (ev) => {
    const val = Number(ev.target.value);
    $('grip-effort-val').textContent = val.toFixed(2);
    postJson('/api/gripper', { action: 'effort', effort: val }).catch(() => {});
  });
  $('btn-grip-close').addEventListener('click', () => {
    const w = Number($('input-close-mm').value) / 1000.0;
    const e = Number($('input-grip-effort').value);
    postJson('/api/gripper', { action: 'close', width_m: w, effort: e }).catch((err) => setNotice(err.message, true));
  });
  $('btn-grip-open').addEventListener('click', () => {
    const w = Number($('input-open-mm').value) / 1000.0;
    const e = Number($('input-grip-effort').value);
    postJson('/api/gripper', { action: 'open', width_m: w, effort: e }).catch((err) => setNotice(err.message, true));
  });
  $('btn-grip-release').addEventListener('click', () =>
    postJson('/api/gripper', { action: 'release' }).catch((err) => setNotice(err.message, true))
  );
  $('btn-apply-tip-offset').addEventListener('click', () => {
    const cm = [
      Number($('input-tip-x-cm').value) || 0,
      Number($('input-tip-y-cm').value) || 0,
      Number($('input-tip-z-cm').value) || 0,
    ];
    postJson('/api/parameters', { tip_offset_cm: cm }).catch((err) => setNotice(err.message, true));
  });

  // Stage 1: Calibration
  $('btn-calib-capture').addEventListener('click', async () => {
    const btn = $('btn-calib-capture');
    btn.disabled = true;
    const oldText = btn.textContent;
    btn.textContent = '⏳ 正在同步抓拍三机画面并解算外参…';
    try {
      await postJson('/api/calib/capture');
    } catch (err) {
      setNotice(err.message, true);
    } finally {
      btn.disabled = false;
      btn.textContent = oldText;
    }
  });
  $('btn-calib-clear').addEventListener('click', () =>
    postJson('/api/calib/clear', { delete_saved: false }).catch((err) => setNotice(err.message, true))
  );
  $('btn-calib-reset-file').addEventListener('click', () =>
    postJson('/api/calib/clear', { delete_saved: true }).catch((err) => setNotice(err.message, true))
  );

  // Stage 2: Prompts & Single-call Agent Perception
  $('btn-reset-prompts').addEventListener('click', () => {
    $('input-object-prompt').value = state.defaultObjectPrompt;
    $('input-output-prompt').value = state.defaultOutputPrompt;
    setNotice('已恢复默认两段式 Prompt');
  });

  $('btn-agent-perceive').addEventListener('click', async () => {
    if (state.perceiving) return;
    try {
      const standoffCm = Number($('input-standoff-cm').value) || 0.5;
      const depthCm = Number($('input-insert-depth-cm').value) || 2.0;
      const effort = $('select-reasoning-effort').value || 'max';
      await postJson('/api/agent/perceive', {
        async: true,
        object_prompt: $('input-object-prompt').value,
        output_prompt: $('input-output-prompt').value,
        reasoning_effort: effort,
        standoff_cm: standoffCm,
        insert_depth_cm: depthCm,
      });
    } catch (err) {
      setNotice(err.message, true);
    }
  });

  $('btn-agent-cancel').addEventListener('click', () =>
    postJson('/api/agent/cancel').catch((err) => setNotice(err.message, true))
  );

  function updatePerceptionImages() {
    const imgs = state.latestPercImages;
    if (!imgs) return;
    let view = state.percImageView || 'gpt';
    if (view === 'reproj' && !imgs.reproj_head_1) {
      view = 'gpt';
      state.percImageView = 'gpt';
    }
    document.querySelectorAll('#perc-view-tabs .annot-tab').forEach((btn) => {
      btn.classList.toggle('active', btn.dataset.view === view);
    });
    let u1 = imgs.gpt_head_1 || imgs.head_1;
    let u2 = imgs.gpt_head_2 || imgs.head_2;
    let full1 = imgs.gpt_head_1 || imgs.head_1;
    let full2 = imgs.gpt_head_2 || imgs.head_2;
    if (view === 'crop' && imgs.gpt_crop_head_1 && imgs.gpt_crop_head_2) {
      u1 = imgs.gpt_crop_head_1;
      u2 = imgs.gpt_crop_head_2;
      full1 = imgs.gpt_crop_head_1;
      full2 = imgs.gpt_crop_head_2;
    } else if (view === 'reproj' && imgs.reproj_head_1 && imgs.reproj_head_2) {
      u1 = imgs.reproj_head_1;
      u2 = imgs.reproj_head_2;
      full1 = imgs.reproj_head_1;
      full2 = imgs.reproj_head_2;
    }
    const el1 = $('img-agent-h1');
    const el2 = $('img-agent-h2');
    if (u1) {
      el1.src = u1;
      el1.dataset.full = full1.split('?')[0];
    }
    if (u2) {
      el2.src = u2;
      el2.dataset.full = full2.split('?')[0];
    }
    const d1 = (state.latestPercDet2d && state.latestPercDet2d.head_1) || null;
    const d2 = (state.latestPercDet2d && state.latestPercDet2d.head_2) || null;
    const fmtPt = (pt) => Array.isArray(pt) ? ('[' + Math.round(pt[0]) + ',' + Math.round(pt[1]) + ']') : '—';
    if (d1) {
      $('cap-agent-h1').textContent = 'head_1 · plug_tip=' + fmtPt(d1.plug_tip) + ' · hole_center=' + fmtPt(d1.hole_center);
    }
    if (d2) {
      $('cap-agent-h2').textContent = 'head_2 · plug_tip=' + fmtPt(d2.plug_tip) + ' · hole_center=' + fmtPt(d2.hole_center);
    }
  }

  document.querySelectorAll('#perc-view-tabs .annot-tab').forEach((btn) => {
    btn.addEventListener('click', () => {
      if (btn.disabled) return;
      state.percImageView = btn.dataset.view || 'gpt';
      updatePerceptionImages();
    });
  });

  $('btn-sync-inferred-offset').addEventListener('click', async () => {
    if (!state.latestInferredOffsetCm) return;
    const [x, y, z] = state.latestInferredOffsetCm;
    $('input-tip-x-cm').value = x.toFixed(2);
    $('input-tip-y-cm').value = y.toFixed(2);
    $('input-tip-z-cm').value = z.toFixed(2);
    try {
      await postJson('/api/parameters', { tip_offset_cm: [x, y, z] });
    } catch (err) {
      setNotice(err.message, true);
    }
  });

  // Stage 3: Preliminary move to 0.5 cm in front of hole
  $('input-standoff-cm').addEventListener('input', (ev) => {
    const val = Number(ev.target.value);
    const txt = Number.isFinite(val) ? val.toFixed(1) : '0.5';
    $('btn-start-move').textContent = '🚀 移动插头末端至孔前方 ' + txt + ' cm';
  });

  $('btn-start-move').addEventListener('click', () => {
    const standoffCm = Number($('input-standoff-cm').value);
    const depthCm = Number($('input-insert-depth-cm').value);
    const durRaw = $('input-move-dur').value.trim();
    postJson('/api/move/pre_insert', {
      standoff_cm: Number.isFinite(standoffCm) ? standoffCm : 0.5,
      insert_depth_cm: Number.isFinite(depthCm) ? depthCm : 2.0,
      duration_s: durRaw === '' ? null : Number(durRaw),
      use_relative_delta: $('chk-use-rel-delta').checked,
    }).catch((err) => setNotice(err.message, true));
  });
  $('btn-move-stop-hold').addEventListener('click', () =>
    postJson('/api/policy/stop', { mode: 'hold' }).catch((err) => setNotice(err.message, true))
  );
  $('btn-move-stop-float').addEventListener('click', () =>
    postJson('/api/policy/stop', { mode: 'drag' }).catch((err) => setNotice(err.message, true))
  );

  // Stage 4: Execute Insert Policy (Rule-based / Policy-based)
  $('select-wrench-source').addEventListener('change', (ev) =>
    postJson('/api/wrench/source', { wrench_source: ev.target.value }).catch((err) => setNotice(err.message, true))
  );
  $('chk-auto-stop-stationary').addEventListener('change', (ev) =>
    postJson('/api/parameters', { auto_stop_stationary: ev.target.checked }).catch((err) => setNotice(err.message, true))
  );
  $('btn-start-rule').addEventListener('click', () =>
    postJson('/api/rule/start', {
      wrench_source: $('select-wrench-source').value,
      auto_stop_stationary: $('chk-auto-stop-stationary').checked,
    }).catch((err) => setNotice(err.message, true))
  );
  $('btn-start-policy').addEventListener('click', () =>
    postJson('/api/policy/start', {
      controller: 'onnx',
      wrench_source: $('select-wrench-source').value,
      auto_stop_stationary: $('chk-auto-stop-stationary').checked,
    }).catch((err) => setNotice(err.message, true))
  );
  $('btn-stop-drag').addEventListener('click', () =>
    postJson('/api/policy/stop', { mode: 'drag' }).catch((err) => setNotice(err.message, true))
  );
  $('btn-stop-hold').addEventListener('click', () =>
    postJson('/api/policy/stop', { mode: 'hold' }).catch((err) => setNotice(err.message, true))
  );
  $('btn-tare-wrench').addEventListener('click', () =>
    postJson('/api/wrench/tare').catch((err) => setNotice(err.message, true))
  );

  // Toggle wrist preview stream
  $('btn-toggle-wrist').addEventListener('click', async () => {
    const next = !state.wristPreviewEnabled;
    try {
      const res = await postJson('/api/wrist/preview', { enabled: next });
      state.wristPreviewEnabled = Boolean(res.wrist_preview_enabled);
      $('btn-toggle-wrist').textContent = state.wristPreviewEnabled ? '暂停腕部流' : '开启腕部流';
      if (state.wristPreviewEnabled) {
        $('live-wrist').src = '/api/stream/wrist?t=' + Date.now();
      }
    } catch (err) {
      setNotice(err.message, true);
    }
  });

  function renderCalibration(af) {
    const ext = (af && af.extrinsics) || {};
    const h1 = ext.head_1 || {};
    const h2 = ext.head_2 || {};
    const readyCount = (h1.calibrated ? 1 : 0) + (h2.calibrated ? 1 : 0);

    const pill = $('calib-pill');
    pill.textContent = '外参：' + readyCount + '/2 就绪';
    pill.dataset.state = readyCount === 2 ? 'online' : 'offline';

    const badge = $('calib-card-badge');
    badge.textContent = readyCount === 2 ? '双相机已标定' : '已标定 ' + readyCount + '/2';
    badge.dataset.ok = readyCount === 2 ? 'true' : 'false';

    function fillRow(prefix, item) {
      if (!item || !item.calibrated) {
        $(prefix + '-state').textContent = '未标定';
        $(prefix + '-rms').textContent = '—';
        $(prefix + '-xyz').textContent = '—';
        $(prefix + '-rpy').textContent = '—';
        return;
      }
      $(prefix + '-state').textContent = '已标定 (' + (item.samples || 1) + ' 组)';
      $(prefix + '-rms').textContent = item.reproj_rms_px != null ? item.reproj_rms_px.toFixed(2) + ' px' : '—';
      $(prefix + '-xyz').textContent = fmtArr(item.translation_mm, 1);
      $(prefix + '-rpy').textContent = fmtArr(item.rpy_deg, 1);
    }
    fillRow('cal-h1', h1);
    fillRow('cal-h2', h2);

    const rep = af && af.last_calib_report;
    if (rep && rep.stamp && rep.stamp !== state.lastCalibStamp && rep.detections) {
      state.lastCalibStamp = rep.stamp;
      $('calib-thumbs').classList.remove('hidden');
      const d = rep.detections;
      if (d.wrist) {
        $('img-calib-wrist').src = d.wrist.image_url;
        $('cap-calib-wrist').textContent = 'wrist: ' + d.wrist.corner_count + ' 角点' + (d.wrist.reproj_rms_px != null ? ' (' + d.wrist.reproj_rms_px.toFixed(2) + 'px)' : '');
      }
      if (d.head_1) {
        $('img-calib-h1').src = d.head_1.image_url;
        $('cap-calib-h1').textContent = 'head_1: ' + d.head_1.corner_count + ' 角点' + (d.head_1.reproj_rms_px != null ? ' (' + d.head_1.reproj_rms_px.toFixed(2) + 'px)' : '');
      }
      if (d.head_2) {
        $('img-calib-h2').src = d.head_2.image_url;
        $('cap-calib-h2').textContent = 'head_2: ' + d.head_2.corner_count + ' 角点' + (d.head_2.reproj_rms_px != null ? ' (' + d.head_2.reproj_rms_px.toFixed(2) + 'px)' : '');
      }
    }
  }

  function renderJob(job) {
    const btn = $('btn-agent-perceive');
    const cancelBtn = $('btn-agent-cancel');
    const box = $('job-status-box');
    const txt = $('job-status-text');
    const running = Boolean(job && job.running);
    state.perceiving = running;
    btn.disabled = running;
    cancelBtn.classList.toggle('hidden', !running);

    if (running) {
      box.classList.remove('hidden', 'error');
      const eff = job.reasoning_effort || 'max';
      const sec = Number(job.elapsed_s || 0).toFixed(1);
      const steps = Number(job.reasoning_steps || 0);
      const stageMap = {
        capturing: '正在抓取双路 1920×1080 相机原图…',
        reasoning: 'gpt-6.1-sol (' + eff + ') 深度推理中 · 已完成 ' + steps + ' 个推理块 · ' + sec + 's',
        generating: '正在流式接收 JSON 坐标输出… ' + sec + 's',
        cancelling: '正在取消请求…',
      };
      txt.textContent = stageMap[job.stage] || ('正在请求 gpt-6.1-sol (' + eff + ')… ' + sec + 's');
      btn.textContent = '⏳ 识别运行中 (' + sec + 's)';
      return;
    }

    btn.textContent = '🧠 单次调用 gpt-6.1-sol 识别双图';
    if (job && job.stage === 'error' && job.error) {
      box.classList.remove('hidden');
      box.classList.add('error');
      txt.textContent = '识别出错：' + job.error;
    } else if (job && job.stage === 'cancelled') {
      box.classList.add('hidden');
    } else {
      box.classList.add('hidden');
    }
  }

  function renderPerception(perc) {
    if (!perc) return;
    $('perception-box').classList.remove('hidden');
    const badge = $('agent-card-badge');
    badge.textContent = (perc.reasoning_effort || 'max') + ' · ' + Number(perc.latency_s || 0).toFixed(1) + 's';
    badge.dataset.ok = 'true';

    const rawOut = perc.raw_llm_output || { symmetry_deg: perc.symmetry_deg, detections_2d: perc.detections_2d };
    $('perc-raw-json').textContent = JSON.stringify(rawOut, null, 2);
    state.latestPercDet2d = perc.detections_2d || null;

    const has3d = perc.calibrated_3d !== false && Array.isArray(perc.hole_entrance_mm);
    const reprojBtn = $('tab-view-reproj');
    if (reprojBtn) reprojBtn.disabled = !has3d;

    if (!has3d) {
      $('perc-hole-xyz').textContent = '待标定外参';
      $('perc-hole-gap').textContent = '完成板块 1 标定后自动换算';
      $('perc-hole-axis').textContent = '—';
      $('perc-axis-angles').textContent = '—';
      $('perc-target-rpy').textContent = 'symmetry_deg: ' + Number(perc.symmetry_deg || 0).toFixed(0) + '°';
      $('perc-align-rot').textContent = '—';
      $('perc-align-dxyz').textContent = '—';
      $('perc-pre-xyz').textContent = '—';
      $('perc-tip-compare').textContent = '—';
      $('perc-inferred-offset').textContent = '—';
      $('perc-arm-json').textContent = JSON.stringify({ calibrated_3d: false, symmetry_deg: perc.symmetry_deg }, null, 2);
      if (perc.stamp && perc.stamp !== state.lastPercStamp && perc.images) {
        state.lastPercStamp = perc.stamp;
        state.latestPercImages = perc.images;
        updatePerceptionImages();
      }
      return;
    }

    $('perc-hole-xyz').textContent = fmtArr(perc.hole_entrance_mm, 1, 'mm');
    $('perc-hole-gap').textContent =
      'ray_gap: ' + Number(perc.hole_ray_gap_mm || 0).toFixed(2) + ' mm · stereo_ang: ' + Number(perc.ray_angle_deg || 0).toFixed(1) + '°';

    const al = perc.alignment || {};
    $('perc-hole-axis').textContent = fmtArr(perc.insert_axis_unit, 3);
    const sph = al.insert_axis_spherical_deg || {};
    $('perc-axis-angles').textContent =
      'az/el: [' + (sph.azimuth_xy_deg ?? 0) + '°, ' + (sph.elevation_z_deg ?? 0) + '°] · hole_orient: ' + fmtArr(al.hole_orient_unit, 3);

    $('perc-target-rpy').textContent =
      fmtArr(al.target_rpy_deg, 1, '°') + ' | twist ' + Number(al.cross_section_twist_deg || 0).toFixed(1) + '° (sym ' + Number(al.symmetry_deg || 0) + '°)';
    $('perc-align-rot').textContent =
      'Δθ=' + Number(al.align_angle_deg || 0).toFixed(2) + '° · rotvec=' + fmtArr(al.align_rotvec_deg, 1, '°');

    $('perc-align-dxyz').textContent =
      fmtArr(al.align_delta_mm, 1, 'mm') + ' (|Δp|=' + Number(al.align_distance_mm || 0).toFixed(1) + ' mm)';
    $('perc-pre-xyz').textContent =
      'pre_insert_mm: ' + fmtArr(al.pre_insert_mm, 1) + ' · goal_mm: ' + fmtArr(al.goal_inserted_mm, 1);

    $('perc-tip-compare').textContent =
      'vis ' + fmtArr(perc.plug_tip_vis_mm, 1) + ' / fk ' + fmtArr(perc.plug_tip_fk_mm, 1) +
      ' (diff ' + fmtArr(perc.plug_vis_minus_fk_mm, 1, 'mm') + ')';
    state.latestInferredOffsetCm = perc.inferred_tip_offset_cm || null;
    $('perc-inferred-offset').textContent =
      'inferred_tip_offset_cm: ' + fmtArr(perc.inferred_tip_offset_cm, 2) +
      ' · plug_ray_gap_mm: ' + Number(perc.plug_ray_gap_mm || 0).toFixed(2);

    const armNumeric = {
      hole_entrance_mm: perc.hole_entrance_mm,
      hole_ray_gap_mm: perc.hole_ray_gap_mm,
      insert_axis_unit: perc.insert_axis_unit,
      hole_orient_unit: al.hole_orient_unit,
      plug_orient_unit: al.plug_orient_unit,
      symmetry_deg: al.symmetry_deg,
      cross_section_twist_deg: al.cross_section_twist_deg,
      q_bh_wxyz: al.q_bh_wxyz,
      target_rpy_deg: al.target_rpy_deg,
      pre_insert_mm: al.pre_insert_mm,
      goal_inserted_mm: al.goal_inserted_mm,
      align_delta_mm: al.align_delta_mm,
      align_rotvec_deg: al.align_rotvec_deg,
      plug_tip_vis_mm: perc.plug_tip_vis_mm,
      plug_tip_fk_mm: perc.plug_tip_fk_mm,
      inferred_tip_offset_cm: perc.inferred_tip_offset_cm,
    };
    $('perc-arm-json').textContent = JSON.stringify(armNumeric, null, 2);

    if (perc.stamp && perc.stamp !== state.lastPercStamp && perc.images) {
      state.lastPercStamp = perc.stamp;
      state.latestPercImages = perc.images;
      updatePerceptionImages();
    }
  }

  function renderTelemetry(data) {
    const arm = data.arm || {};
    const grip = data.gripper || {};
    const wrench = data.wrench || {};
    const pol = data.policy || {};
    const af = data.agent_flow || {};

    // Initialize defaults once
    if (!state.promptsInitialized && af.llm) {
      state.defaultObjectPrompt = af.llm.default_object_prompt || state.defaultObjectPrompt;
      state.defaultOutputPrompt = af.llm.default_output_prompt || '';
      $('input-object-prompt').value = state.defaultObjectPrompt;
      $('input-output-prompt').value = state.defaultOutputPrompt;
      state.promptsInitialized = true;
    }
    if (!state.tipInputsInitialized && Array.isArray(pol.tip_offset_cm)) {
      $('input-tip-x-cm').value = Number(pol.tip_offset_cm[0]).toFixed(1);
      $('input-tip-y-cm').value = Number(pol.tip_offset_cm[1]).toFixed(1);
      $('input-tip-z-cm').value = Number(pol.tip_offset_cm[2]).toFixed(1);
      state.tipInputsInitialized = true;
    }
    if (!state.wrenchSelectInitialized && wrench.source) {
      $('select-wrench-source').value = wrench.source;
      state.wrenchSelectInitialized = true;
    }
    if (!state.autoStopInitialized && typeof af.auto_stop_stationary === 'boolean') {
      $('chk-auto-stop-stationary').checked = af.auto_stop_stationary;
      state.autoStopInitialized = true;
    }

    $('can-badge').textContent = (arm.can_label || 'can4').toUpperCase() + ' · ' + (arm.namespace || '/arm_4');

    const connPill = $('conn-pill');
    connPill.dataset.state = arm.online ? 'online' : 'offline';
    connPill.textContent = arm.online ? '机械臂在线' : '等待遥测';

    const pwrPill = $('power-pill');
    pwrPill.dataset.state = arm.power_state || 'unknown';
    const pwrLabels = { enabled: '已使能', disabled: '已失能', mixed: '部分使能', unknown: '使能未知' };
    pwrPill.textContent = pwrLabels[arm.power_state] || '使能未知';

    const modePill = $('mode-pill');
    const modeNames = {
      drag: 'Float (零力拖动)',
      move: '初步移动中',
      hold: 'Hold (位置保持)',
      policy: pol.controller === 'rule' ? 'Rule 插入中' : 'ONNX 策略中',
      idle: '待机',
    };
    modePill.dataset.mode = pol.mode || 'idle';
    modePill.textContent = '模式：' + (modeNames[pol.mode] || pol.mode);

    if (data.status_message && !state.perceiving) {
      setNotice(data.status_message, Boolean(data.last_error));
    }

    // Gripper
    const gMap = { idle: '待机', open: '已打开', closed: '已闭合夹紧', released: '零力释放' };
    $('gripper-state-badge').textContent = '夹爪：' + (gMap[grip.state] || grip.state || '待机');
    $('grip-actual-mm').textContent = arm.gripper_width_mm != null ? arm.gripper_width_mm.toFixed(1) + ' mm' : '— mm';

    // Stage 1 & Stage 2
    renderCalibration(af);
    renderJob(af.job);
    if (af.perception && af.perception.stamp !== state.lastPercStamp) {
      renderPerception(af.perception);
    }

    // Stage 3 Move status
    const mv = af.move || {};
    const pct = Math.round(Number(mv.progress || 0) * 100);
    $('move-progress-fill').style.width = (mv.active ? pct : (pol.mode === 'hold' && pct >= 100 ? 100 : pct)) + '%';
    $('move-progress-text').textContent = mv.active
      ? ('正在平滑移动… ' + pct + '%')
      : (pct >= 100 ? '初步移动已完成 (100%)' : '等待移动指令');
    $('move-dist-remain').textContent =
      mv.distance_to_pre_mm != null ? ('距预插点：' + mv.distance_to_pre_mm.toFixed(1) + ' mm') : '距预插点：— mm';
    const moveBadge = $('move-card-badge');
    moveBadge.textContent = mv.active ? ('移动中 ' + pct + '%') : (pct >= 100 ? '已就位' : '待执行');
    moveBadge.dataset.ok = (!mv.active && pct >= 100) ? 'true' : 'false';

    // Stage 4 Policy & Wrench
    const wsOk = wrench.source === 'paxini' ? wrench.paxini_fresh : wrench.estimated_fresh;
    const wsHz = wrench.source === 'paxini' ? wrench.paxini_hz : wrench.estimated_hz;
    const wsBadge = $('wrench-source-badge');
    wsBadge.textContent = (wrench.source === 'paxini' ? 'Paxini' : '估计力') + (wsOk ? (' · ' + wsHz.toFixed(0) + ' Hz') : ' · 等待数据');
    wsBadge.dataset.ok = wsOk ? 'true' : 'false';

    const pBadge = $('policy-state-badge');
    if (pol.mode === 'policy') {
      pBadge.textContent = pol.controller === 'rule'
        ? ('Rule · ' + (pol.rule_stage || '运行中'))
        : 'ONNX 运行中';
      pBadge.dataset.ok = 'true';
    } else {
      pBadge.textContent = pol.reached_goal ? '已到位' : ('策略：' + (modeNames[pol.mode] || '待机'));
      pBadge.dataset.ok = pol.reached_goal ? 'true' : 'false';
    }

    $('policy-steps-time').textContent = (pol.steps || 0) + ' 步 · ' + Number(pol.elapsed_s || 0).toFixed(1) + ' s';
    const lat = pol.last_step ? pol.last_step.compute_ms.toFixed(2) : '—';
    $('policy-rate-latency').textContent = Number(pol.rate_hz || 0).toFixed(1) + ' Hz · ' + lat + ' ms';
    const penStr = af.penetration_past_entrance_mm != null ? (af.penetration_past_entrance_mm.toFixed(1) + ' mm') : '— mm';
    $('policy-penetration').textContent = penStr + ' · ' + (pol.reached_goal ? '已达目标深度' : (pol.mode === 'policy' ? '插入中' : '待机'));
    if (pol.last_step && Array.isArray(pol.last_step.dp_h_mm)) {
      $('policy-last-action').textContent = fmtArr(pol.last_step.dp_h_mm, 2, 'mm');
    } else {
      $('policy-last-action').textContent = '—';
    }

    // Hole relative visualizer
    if (arm.tip_pose && Array.isArray(arm.tip_pose.position_mm)) {
      $('tip-pose-b-tag').textContent = 'B 系尖端：' + fmtArr(arm.tip_pose.position_mm, 1, 'mm');
    }
    const rel = arm.hole_relative;
    if (rel && Array.isArray(rel.p_ht_mm)) {
      const [dx, dy, dz] = rel.p_ht_mm;
      $('rel-dx').textContent = dx.toFixed(1) + ' mm';
      $('rel-dy').textContent = dy.toFixed(1) + ' mm';
      $('rel-dz').textContent = dz.toFixed(1) + ' mm';
      $('rel-xy-norm').textContent = rel.xy_distance_mm.toFixed(2) + ' mm';
      $('rel-total-norm').textContent = rel.total_distance_mm.toFixed(1) + ' mm';
      $('rel-angle-err').textContent = rel.angle_error_deg.toFixed(2) + ' °';

      const cx = Math.max(-36, Math.min(36, dx));
      const cy = Math.max(-36, Math.min(36, -dy));
      $('xy-tip-dot').setAttribute('cx', cx.toFixed(2));
      $('xy-tip-dot').setAttribute('cy', cy.toFixed(2));
      $('xy-tip-x').setAttribute('x1', cx.toFixed(2));
      $('xy-tip-x').setAttribute('x2', cx.toFixed(2));
      $('xy-tip-y').setAttribute('y1', cy.toFixed(2));
      $('xy-tip-y').setAttribute('y2', cy.toFixed(2));

      const pctZ = Math.max(4, Math.min(95, 20 + (dz / 40.0) * 75));
      $('z-tip-marker').style.bottom = pctZ.toFixed(1) + '%';
    }

    // Wrench bars
    if (Array.isArray(wrench.tip_t) && wrench.tip_t.length === 6) {
      const [fx, fy, fz, mx, my, mz] = wrench.tip_t;
      const setW = (id, barId, val, scale, dig) => {
        $(id).textContent = val.toFixed(dig);
        const wPct = Math.min(100, (Math.abs(val) / scale) * 100);
        $(barId).style.width = wPct.toFixed(1) + '%';
      };
      setW('w-fx', 'wb-fx', fx, 15.0, 2);
      setW('w-fy', 'wb-fy', fy, 15.0, 2);
      setW('w-fz', 'wb-fz', fz, 20.0, 2);
      setW('w-mx', 'wb-mx', mx, 1.5, 3);
      setW('w-my', 'wb-my', my, 1.5, 3);
      setW('w-mz', 'wb-mz', mz, 1.5, 3);
      $('wrench-summary-tag').textContent =
        '|F| ' + Number(wrench.force_norm_n || 0).toFixed(2) + ' N · |M| ' + Number(wrench.moment_norm_nm || 0).toFixed(3) + ' N·m';
    }

    // Joints table
    if (Array.isArray(arm.joints_deg) && arm.joints_deg.length >= 6) {
      const rows = $('joint-tbody').querySelectorAll('tr');
      const vels = arm.velocities_rad_s || [];
      const effs = arm.efforts_nm || [];
      const cmds = (pol.last_step && pol.last_step.q_cmd_deg) || [];
      for (let i = 0; i < 6; i++) {
        const tds = rows[i].querySelectorAll('td');
        tds[1].textContent = arm.joints_deg[i].toFixed(2);
        tds[2].textContent = vels[i] != null ? vels[i].toFixed(3) : '—';
        tds[3].textContent = effs[i] != null ? effs[i].toFixed(2) : '—';
        tds[4].textContent = cmds[i] != null ? cmds[i].toFixed(2) : '—';
      }
    }
    const rates = arm.rates || {};
    if (rates.feedback_hz != null) {
      $('driver-rates-tag').textContent = '反馈 ' + Number(rates.feedback_hz).toFixed(0) + ' Hz';
    }

    // Logs
    if (Array.isArray(data.logs)) {
      const list = $('log-list');
      list.innerHTML = data.logs
        .slice(-20)
        .reverse()
        .map((item) => '<div class="log-row ' + (item.level || '') + '">[' + item.time + '] ' + item.msg + '</div>')
        .join('');
    }
  }

  // Poll /api/status
  let polling = false;
  setInterval(async () => {
    if (polling) return;
    polling = true;
    try {
      const resp = await fetch('/api/status');
      if (resp.ok) {
        const data = await resp.json();
        renderTelemetry(data);
      }
    } catch (_e) {
      $('conn-pill').dataset.state = 'offline';
      $('conn-pill').textContent = '连接断开';
    } finally {
      polling = false;
    }
  }, 150);
})();
