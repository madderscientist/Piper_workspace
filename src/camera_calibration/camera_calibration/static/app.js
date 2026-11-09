
'use strict';

let STATE = null;
const PROFILE = {};
const PICKED = new Set();
let BOARD_SIG = '';

function wh(camera) {
  const [w, h] = (PROFILE[camera] || '0x0').split('x').map(Number);
  return { width: w, height: h };
}

document.querySelectorAll('.tab').forEach((btn) => {
  btn.onclick = () => {
    document.querySelectorAll('.tab').forEach((b) => b.classList.remove('active'));
    document.querySelectorAll('.tab-body').forEach((s) => { s.hidden = true; });
    btn.classList.add('active');
    $('tab-' + btn.dataset.tab).hidden = false;
    render();
  };
});

async function refresh() {
  try {
    STATE = await get('/api/state');
    render();
  } catch (err) {
    banner('连不上标定节点：' + err.message, 'bad', true);
  }
}

function render() {
  if (!STATE) return;
  const b = STATE.board;
  $('arm-badge').textContent =
    `${STATE.arm} (${STATE.arm_namespace}) · ${STATE.camera}` +
    (STATE.camera_ip ? ` · ${STATE.camera_ip}` : '');
  $('board-info').textContent =
    `板 ${b.squares_x}×${b.squares_y} · 方格 ${b.square_size_mm ?? b.square_size * 1000}mm · ` +
    `marker ${b.marker_size_mm ?? b.marker_size * 1000}mm · ${b.dictionary}（${b.corner_count} 角点）`;
  $('calib-path').textContent = '标定文件：' + STATE.calib_file;
  $('board-path').textContent = STATE.board_config_file ? ('板配置：' + STATE.board_config_file) : '';

  syncBoardForm(b);
  syncRefForm(STATE.reference_point);
  renderCoach();
  renderIntrinsic();
  renderExtrinsic();
}

function syncBoardForm(b) {
  const sig = JSON.stringify([b.squares_x, b.squares_y, b.square_size, b.marker_size, b.dictionary]);
  const card = $('board-card');
  if (card && card.contains(document.activeElement)) return;
  if (sig === BOARD_SIG) return;
  BOARD_SIG = sig;
  $('board-squares-x').value = b.squares_x;
  $('board-squares-y').value = b.squares_y;
  $('board-square-mm').value = b.square_size_mm ?? Number((b.square_size * 1000).toFixed(3));
  $('board-marker-mm').value = b.marker_size_mm ?? Number((b.marker_size * 1000).toFixed(3));
  $('board-dictionary').value = b.dictionary;
}

$('btn-apply-board').onclick = (e) => guard(e.target, async () => {
  const payload = {
    squares_x: Number($('board-squares-x').value),
    squares_y: Number($('board-squares-y').value),
    square_size_mm: Number($('board-square-mm').value),
    marker_size_mm: Number($('board-marker-mm').value),
    dictionary: $('board-dictionary').value.trim(),
  };
  const out = await post('/api/board', payload);
  BOARD_SIG = '';
  out._banner = [
    `已切换标定板规格：${out.board.squares_x}×${out.board.squares_y} · ` +
    `Checker ${out.board.square_size_mm}mm · Marker ${out.board.marker_size_mm}mm · ${out.board.dictionary}`,
    'good',
  ];
  return out;
});

// ---------- 内参 tab ----------

function renderIntrinsic() {
  const host = $('cameras');
  const shape = STATE.cameras.map(
    (c) => c.name + ':' + c.profiles.map((p) => profileKey(p.width, p.height)).join(',')).join('|');
  if (stale(host, shape)) buildCameraCards(host);
  STATE.cameras.forEach(updateCamera);
}

function buildCameraCards(host) {
  host.innerHTML = '';
  STATE.cameras.forEach((cam) => {
    if (!PROFILE[cam.name]) {
      const live = cam.width && profileKey(cam.width, cam.height);
      const declared = new Set(cam.profiles.map((p) => profileKey(p.width, p.height)));
      PROFILE[cam.name] = declared.has(live) ? live
        : profileKey(cam.profiles[0].width, cam.profiles[0].height);
    }
    PICKED.add(cam.name);

    const card = document.createElement('div');
    card.className = 'card';
    card.dataset.cam = cam.name;
    const options = cam.profiles.map((p) => {
      const k = profileKey(p.width, p.height);
      return `<option value="${k}">${p.width}×${p.height}</option>`;
    }).join('');

    card.innerHTML = `
      <div class="cam-head">
        <span class="dot"></span>
        <span class="title">${cam.label} (${cam.name}${cam.ip ? ' · ' + cam.ip : ''})</span>
        <span class="grow"></span>
        <select class="profile">${options}</select>
        ${cam.switchable
          ? '<button class="apply small" data-done="档位已切换">切换</button>'
          : '<span class="muted">固定档位</span>'}
        ${cam.profiles.length > 1
          ? '<button class="relate small" title="两个档位都标完后，实测判定它们是缩放还是裁剪">判定档位关系</button>'
          : ''}
      </div>
      <img class="preview" alt="${cam.label}">
      <div class="stats"></div>
      <div class="toolbar">
        <button class="solve small primary" data-done="求解完成">求解</button>
        <button class="save small" data-done="已保存">保存</button>
        <button class="drop small" data-done="已删除离群张">删离群</button>
        <button class="clear small danger" data-done="已清空">清空本档位</button>
      </div>
      <div class="shots thumbs"></div>
      <div class="result"></div>`;
    host.appendChild(card);

    const sel = card.querySelector('.profile');
    sel.value = PROFILE[cam.name];
    sel.onchange = () => { PROFILE[cam.name] = sel.value; render(); };
    Object.entries({
      apply: '/api/profile',
      solve: '/api/solve_intrinsic',
      save: '/api/save_intrinsic',
      drop: '/api/drop_outliers',
      clear: '/api/clear_shots',
    }).forEach(([cls, api]) => {
      const btn = card.querySelector('.' + cls);
      if (btn) {
        btn.onclick = (e) =>
          guard(e.target, () => post(api, { camera: cam.name, ...wh(cam.name) }));
      }
    });
    const relBtn = card.querySelector('.relate');
    if (relBtn) {
      relBtn.onclick = (e) => guard(e.target, async () => {
        const out = await post('/api/relate', { camera: cam.name });
        const summary = (out.relations || []).map(
          (r) => `${r.from.join('×')} → ${r.to.join('×')}: ${r.kind} (误差 ${r.scale_error_pct}%)`).join('；');
        out._banner = [`档位关系：${summary}`, 'good'];
        return out;
      });
    }
  });
}

function updateCamera(cam) {
  const card = document.querySelector(`[data-cam="${cam.name}"]`);
  if (!card) return;
  card.querySelector('.dot').classList.toggle('on', cam.online);

  const img = card.querySelector('.preview');
  const overlay = $('chk-overlay').checked ? 1 : 0;
  refreshPreview(img, `/api/preview?camera=${cam.name}&overlay=${overlay}&t=${Date.now()}`);

  const picked = wh(cam.name);
  const shot = (cam.shots || []).find(
    (s) => s.width === picked.width && s.height === picked.height) || {};
  const live = cam.online ? profileKey(cam.width, cam.height) : '离线';
  const mismatch = cam.online && live !== PROFILE[cam.name];
  card.querySelector('.stats').innerHTML = `
    <span>在推 <b class="${mismatch ? 'warnish' : ''}">${live}</b>${
      mismatch ? '（和选中档位不符，先点切换）' : ''}</span>
    <span>角点 <b class="${cam.corners >= cam.max_corners * 0.5 ? 'ok' : 'no'}">${
      cam.corners}</b>/${cam.max_corners}</span>
    <span>marker <b>${cam.markers}</b></span>
    <span>单张覆盖 <b>${(cam.coverage * 100).toFixed(0)}%</b></span>
    <span>已拍 <b>${shot.shots || 0}</b> 张</span>
    <span>已存 RMS <b>${shot.rms === null || shot.rms === undefined ? '—' : shot.rms}</b></span>`;

  renderShots(card.querySelector('.shots'), cam.name, picked, shot.names || []);
  renderResult(card.querySelector('.result'), cam.name, picked);
}

function renderShots(host, camera, picked, names) {
  if (!stale(host, `${picked.width}x${picked.height}:${names.join(',')}`)) return;
  host.innerHTML = '';
  if (!names.length) { host.innerHTML = '<span class="muted">还没拍</span>'; return; }
  names.forEach((name) => {
    const box = document.createElement('div');
    box.className = 'thumb';
    box.innerHTML = `
      <img loading="lazy" src="/api/shot?camera=${camera}&width=${picked.width}`
      + `&height=${picked.height}&name=${name}">
      <div class="cap">${name}</div>
      <button class="x" title="删掉这张">×</button>`;
    box.querySelector('.x').onclick = (e) =>
      guard(e.target, () => post('/api/delete_shot', { camera, ...picked, name }));
    host.appendChild(box);
  });
}

function renderResult(host, camera, picked) {
  const result = STATE.results[`${camera}:${picked.width}x${picked.height}`];
  if (!stale(host, JSON.stringify(result || null))) return;
  if (!result) { host.innerHTML = ''; return; }
  if (!result.ok) {
    host.innerHTML = `<div class="no">求解失败：${result.reason}</div>`;
    return;
  }
  const k = result.camera_matrix;
  const outliers = new Set(result.outliers || []);
  host.innerHTML = `
    <table>
      <tr><th>RMS</th><td>${result.rms} px</td>
          <th>用了</th><td>${result.images} 张</td></tr>
      <tr><th>fx / fy</th><td>${num(k[0], 2)} / ${num(k[4], 2)}</td>
          <th>cx / cy</th><td>${num(k[2], 2)} / ${num(k[5], 2)}</td></tr>
      <tr><th>畸变</th><td colspan="3">${vec(result.distortion_coefficients, 4)}</td></tr>
      <tr><th>FOV</th><td>${result.fov_deg[0]}° × ${result.fov_deg[1]}°</td>
          <th>整体覆盖</th><td class="${result.coverage > 0.6 ? 'ok' : 'warnish'}">${
            (result.coverage * 100).toFixed(0)}%</td></tr>
    </table>
    <table>
      <tr><th>图</th><th>角点</th><th>重投影 RMS</th><th>最大</th><th>覆盖</th></tr>
      ${result.per_image.map((r) => `<tr class="${outliers.has(r.name) ? 'bad' : ''}">
        <td>${r.name}</td><td>${r.corners}</td><td>${r.error_px}</td>
        <td>${r.max_px}</td><td>${(r.coverage * 100).toFixed(0)}%</td></tr>`).join('')}
    </table>`;
}

$('btn-capture').onclick = (e) => guard(e.target, async () => {
  const out = await post('/api/capture', { cameras: [STATE.camera] });
  const good = out.saved.filter((s) => s.ok);
  const bad = out.saved.filter((s) => !s.ok);
  const why = bad.map((s) => `${s.camera}(${s.reason})`).join('、');
  if (!good.length) throw new Error('未保存图像：' + why);
  out._banner = [`已拍 ${good.map((s) => `${s.camera} (#${s.name}, ${s.corners} 角点)`).join('、')}`, 'good'];
  return out;
});

$('btn-probe').onclick = (e) => guard(e.target, async () => {
  const camera = STATE.cameras[0];
  if (!camera || !camera.online) throw new Error('当前相机未在线');
  const out = await post('/api/probe_board', { camera: camera.name });
  const panel = $('probe-panel');
  panel.hidden = false;
  $('probe-table').innerHTML = `
    <p class="muted">用 ${camera.label} 当前这一帧探测</p>
    <table>
      <tr><th>字典</th><th>格数</th><th>marker</th><th>角点</th><th>共面残差</th><th></th></tr>
      ${out.candidates.map((c, i) => `<tr class="${i === 0 && c.corners ? 'best' : ''}">
        <td>${c.dictionary}</td><td>${c.squares_x}×${c.squares_y}</td>
        <td>${c.markers ?? '—'}</td><td>${c.corners ?? 0}/${c.max_corners ?? '—'}</td>
        <td>${c.residual_px === null || c.residual_px === undefined
          ? (c.reason || '—') : c.residual_px + ' px'}</td>
        <td>${c.ok ? `<button class="small use-probe" data-dict="${c.dictionary}" data-sx="${c.squares_x}" data-sy="${c.squares_y}">采用</button>` : ''}</td></tr>`).join('')}
    </table>`;
  $('probe-table').querySelectorAll('.use-probe').forEach((btn) => {
    btn.onclick = (ev) => guard(ev.target, () => post('/api/board', {
      squares_x: Number(btn.dataset.sx),
      squares_y: Number(btn.dataset.sy),
      dictionary: btn.dataset.dict,
    }));
  });
  return out;
});

$('chk-overlay').onchange = render;

// ---------- 外参 tab ----------

function renderExtrinsic() {
  renderExtrinsicPreviews();
  renderPreflight();
  renderPoses();
  renderExtrinsicResult();
}

const INTRINSIC_LABEL = {
  calibrated: ['标定值', 'ok'],
  factory: ['出厂值', 'warnish'],
  none: ['无', 'no'],
};

function renderExtrinsicPreviews() {
  const host = $('ext-cameras');
  if (stale(host, STATE.cameras.map((c) => c.name).join('|'))) {
    host.innerHTML = STATE.cameras.map((cam) => `
      <div class="card" data-ext="${cam.name}">
        <div class="cam-head">
          <span class="dot"></span>
          <span class="title">${cam.label} (${cam.name})</span>
          <span class="grow"></span>
          <span class="muted">父坐标系：${cam.parent_frame}</span>
        </div>
        <img class="preview" alt="${cam.label}">
        <div class="stats"></div>
      </div>`).join('');
  }
  STATE.cameras.forEach((cam) => {
    const card = host.querySelector(`[data-ext="${cam.name}"]`);
    if (!card) return;
    card.querySelector('.dot').classList.toggle('on', cam.online);
    refreshPreview(card.querySelector('.preview'),
                   `/api/preview?camera=${cam.name}&overlay=1&t=${Date.now()}`);
    const [label, cls] = INTRINSIC_LABEL[cam.intrinsic_source] || INTRINSIC_LABEL.none;
    const enough = cam.corners >= 6;
    card.querySelector('.stats').innerHTML = `
      <span>${cam.online ? profileKey(cam.width, cam.height) : '离线'}</span>
      <span>角点 <b class="${enough ? 'ok' : 'no'}">${cam.corners}</b>/${cam.max_corners}</span>
      <span>内参 <b class="${cls}">${label}</b></span>`;
  });
}

function renderPreflight() {
  const rows = [];
  const motion = STATE.motion;
  rows.push([motion.ok, `${STATE.arm} 机械臂静止`,
    motion.ok ? `已静止 ${motion.still_s}s（${motion.speed} rad/s）` : motion.reason]);

  const tf = STATE.tf;
  Object.entries(tf.links).forEach(([name, link]) => {
    const poseInfo = link.xyz
      ? `可用 (${link.source === 'fk' ? '关节正运动学' : 'TF'} xyz=[${link.xyz.join(', ')}] m)`
      : (link.ok ? '可用' : `未就绪，请确认 ${STATE.arm} 驱动已启动`);
    rows.push([link.ok, `末端位姿 ${tf.base_frame} → ${link.frame}`, poseInfo]);
  });

  STATE.cameras.forEach((cam) => {
    rows.push([cam.online, `${cam.label} 在线`,
      cam.online ? `${cam.width}×${cam.height}` : '收不到图']);
    rows.push([cam.intrinsic_source !== 'none', `${cam.label} 内参`,
      { calibrated: '使用已标定内参',
        factory: '使用相机自报出厂内参',
        none: '当前档位尚未标定内参，请先在「① 内参」完成标定并保存',
      }[cam.intrinsic_source]]);
    rows.push([cam.corners >= 6, `${cam.label} 看得见标定板`,
      `${cam.corners} 个角点（至少需要 6 个）`]);
  });

  const sig = JSON.stringify(rows);
  const host = $('preflight');
  if (!stale(host, sig)) return;
  host.innerHTML = '<h3>采集前检查</h3><div class="checks">'
    + rows.map(([ok, what, detail]) =>
      `<div><span class="${ok ? 'ok' : 'no'}">${ok ? '✓' : '✗'}</span>
       <b>${what}</b><span class="muted">${detail}</span></div>`).join('')
    + '</div>';
}

function renderPoses() {
  const host = $('pose-list');
  if (!stale(host, JSON.stringify(STATE.poses))) return;
  if (!STATE.poses.length) {
    host.innerHTML = '<h3>已采姿态</h3><p class="muted">还没采（手眼标定至少需要 3 个不同旋转方向的姿态）</p>';
    return;
  }
  host.innerHTML = `<h3>已采姿态（${STATE.poses.length}）</h3>
    <table>
      <tr><th>名字</th><th>时间</th><th>角点数</th><th>内参来源</th><th>末端位置 xyz (m)</th><th>预览</th><th></th></tr>
      ${STATE.poses.map((p) => `<tr>
        <td>${p.name}</td><td class="muted">${(p.stamp || '').replace('T', ' ')}</td>
        <td>${p.corners}</td>
        <td class="${p.intrinsic_source === 'factory' ? 'warnish' : ''}">${
          (INTRINSIC_LABEL[p.intrinsic_source] || ['—'])[0]}</td>
        <td>${p.xyz ? p.xyz.join(', ') : '—'}</td>
        <td><img src="/api/pose_image?name=${p.name}&camera=${STATE.camera}" style="height:36px;border-radius:4px"></td>
        <td><button class="small danger del" data-name="${p.name}">删</button></td>
      </tr>`).join('')}
    </table>`;
  host.querySelectorAll('.del').forEach((btn) => {
    btn.onclick = (e) => guard(e.target,
      () => post('/api/delete_pose', { name: btn.dataset.name }));
  });
}

function renderExtrinsicResult() {
  const results = STATE.results.extrinsic;
  const host = $('extrinsic-result');
  const sig = JSON.stringify({ results: results || null, saved: STATE.saved_extrinsic || null });
  if (!stale(host, sig)) return;
  let html = '';
  if (results && results.cameras) {
    html += Object.values(results.cameras).map(cameraBlock).join('');
  } else if (STATE.saved_extrinsic) {
    const s = STATE.saved_extrinsic;
    html += `<div class="card"><h3>已保存的外参（${s.parent} → ${s.child}）</h3>
      <table>
        <tr><th>xyz (m)</th><td>${vec(s.translation)}</td></tr>
        <tr><th>rpy (°)</th><td>${degs(s.rpy)}</td></tr>
        <tr><th>quat xyzw</th><td>${vec(s.rotation)}</td></tr>
        <tr><th>标定方法 / 样本数</th><td>${s.method} / ${s.samples} 组（${s.stamp || ''}）</td></tr>
      </table></div>`;
  }
  host.innerHTML = html;
  host.querySelectorAll('.save-ext').forEach((btn) => {
    btn.onclick = (e) => guard(e.target, () => post('/api/save_extrinsic',
      { camera: btn.dataset.camera, method: btn.dataset.method }));
  });
}

function cameraBlock(entry) {
  const cross = entry.cross_check;
  const agree = cross && cross.angle_deg < 2 && cross.trans_mm < 10;
  const board = entry.board_stability || {};
  return `<div class="card">
    <h3>${entry.label}（${entry.parent} → ${entry.child}，共 ${entry.samples} 组姿态）</h3>
    <p class="${board.fixed ? 'muted' : 'warnish'}">
      各姿态反推标定板在基座系下的最大偏差：${num(board.angle_max_deg, 2)}° / ${num(board.trans_max_mm, 1)} mm ——
      ${board.fixed ? '板在采集期间保持固定，手眼标定有效'
        : '各姿态反推的板位姿差异较大，请检查采集过程中标定板是否被挪动或存在模糊帧'}
    </p>
    ${solutionTable('非线性联合最小二乘精化（SE(3) 闭环优化，推荐）', entry.joint, entry.camera, 'joint')}
    ${solutionTable('AX=XB 经典手眼（cv2.calibrateHandEye）', entry.handeye, entry.camera, 'handeye')}
    ${cross ? `<p class="${agree ? 'ok' : 'warnish'}">
      非线性精化 vs AX=XB 差异：${cross.angle_deg}° / ${cross.trans_mm} mm
      ${agree ? '—— 两种解法一致' : '—— 差异偏大，建议增加不同旋转轴方向的采集姿态'}</p>` : ''}
  </div>`;
}

function solutionTable(title, solution, camera, method) {
  if (!solution || !solution.ok) {
    return `<p><b>${title}</b> <span class="no">${solution ? solution.reason : '无解'}</span></p>`;
  }
  const t = solution.transform;
  const c = solution.consistency;
  return `<p><b>${title}</b>${solution.best ? `（最优算法 ${solution.best}）` : ''}
      <button class="small primary save-ext" data-camera="${camera}" data-method="${method}"
        data-done="外参已保存">保存这一组</button></p>
    <table>
      <tr><th>xyz 相对夹爪最高点 <code>gripper_base_top</code> (m)</th><td><b>${vec(t.translation)}</b></td></tr>
      ${solution.mesh_origins ? `<tr><th>折算至 <code>gripper_base_without_flange.STL</code> 底面原点 (+52.5 mm)</th><td>${vec(solution.mesh_origins.gripper_base_without_flange)} m</td></tr>
      <tr><th>折算至 <code>gripper_base.STL</code> 底面原点 (+63.0 mm)</th><td>${vec(solution.mesh_origins.gripper_base_with_flange)} m</td></tr>` : ""}
      <tr><th>rpy (°)</th><td>${degs(t.rpy)}</td></tr>
      <tr><th>quat xyzw</th><td>${vec(t.rotation)}</td></tr>
      ${solution.residual_mm !== undefined ? `<tr><th>最小二乘残差</th>
        <td class="${solution.residual_mm < 10 ? 'ok' : 'warnish'}">
          ${solution.residual_mm} mm rms（初值 ${solution.residual_mm_before} mm）${
            solution.well_posed === false ? ` · 条件数 ${solution.condition}（激励不足）` : ''}</td></tr>` : ''}
      <tr><th>板位姿一致性残差</th>
        <td class="${c.trans_rms_mm < 5 ? 'ok' : 'warnish'}">
          ${num(c.angle_rms_deg, 3)}° rms / ${num(c.trans_rms_mm, 2)} mm rms
          （最大 ${num(c.angle_max_deg, 3)}° / ${num(c.trans_max_mm, 2)} mm）</td></tr>
      ${solution.method_spread ? `<tr><th>四算法间离散（Tsai/Park/Horaud/Daniilidis）</th>
        <td>${num(solution.method_spread.angle_rms_deg, 3)}° rms / ${num(solution.method_spread.trans_rms_mm, 2)} mm rms</td></tr>` : ''}
    </table>`;
}

$('btn-capture-pose').onclick = (e) =>
  guard(e.target, () => post('/api/capture_pose', {}));
$('btn-solve-extrinsic').onclick = (e) =>
  guard(e.target, () => post('/api/solve_extrinsic', {}));
$('btn-clear-poses').onclick = (e) =>
  guard(e.target, () => post('/api/clear_poses', {}));

refresh();
setInterval(refresh, 1000);


const POSE_COACH_TIPS = [
  '第 1 组建议：把机械臂移到标定板正上方居中（距离约 40～55 cm），镜头大致正对标定板，停手静置 0.5 秒后点「📸 采集一组姿态」。',
  '第 2 组建议：标定板保持不动！将机械臂往左平移 10～15 cm，同时手腕向右歪斜（Roll 旋转约 +20°）斜看标定板，停稳后点「📸 采集一组姿态」。',
  '第 3 组建议：标定板保持不动！将机械臂往右平移 10～15 cm，同时手腕向左歪斜（Roll 旋转约 -20°）斜看标定板，停稳后点「📸 采集一组姿态」。',
  '第 4 组建议：将机械臂往前伸出约 10 cm，手腕向后仰（Pitch 倾斜约 +20°）斜看标定板，停稳后采集（已满 3 组可点「求解」，建议采满 6～8 组更稳）。',
  '第 5 组建议：将机械臂往后收回约 10 cm，手腕向前低头（Pitch 倾斜约 -20°）看标定板，停稳后点「📸 采集一组姿态」。',
  '第 6 组建议：将机械臂高度降低约 8 cm，末端第 6 轴顺时针旋转约 25° + 复合倾斜对准标定板，停稳后采集。',
  '第 7 组建议：将机械臂高度抬高约 8 cm，末端第 6 轴逆时针旋转约 25° + 复合倾斜对准标定板，停稳后采集。',
  '第 8+ 组建议：已采集充足姿态！可继续补充不同斜视角，或直接点击「🧮 求解外参」查看标定结果。',
];

function setStepState(cardId, tagId, mode, text) {
  const card = $(cardId);
  const tag = $(tagId);
  if (!card || !tag) return;
  card.classList.toggle('active', mode === 'active');
  card.classList.toggle('done', mode === 'ok');
  tag.className = 'step-tag' + (mode ? ' ' + mode : '');
  tag.textContent = text;
}

function renderCoach() {
  const cam = STATE.cameras[0];
  if (!cam) return;
  const coachCard = $('next-step-coach');
  const titleEl = $('coach-title');
  const bodyEl = $('coach-body');
  const activeTab = document.querySelector('.tab.active')?.dataset.tab || 'intrinsic';

  if (activeTab === 'intrinsic') {
    const picked = wh(cam.name);
    const pKey = profileKey(picked.width, picked.height);
    const shot = (cam.shots || []).find((s) => s.width === picked.width && s.height === picked.height) || {};
    const count = shot.shots || 0;
    const savedRms = shot.rms;
    const result = STATE.results[`${cam.name}:${pKey}`];
    const liveKey = cam.online ? profileKey(cam.width, cam.height) : '';
    const mismatch = cam.online && liveKey !== pKey;

    const step1Ok = cam.online && !mismatch && cam.corners >= 8;
    setStepState('int-step-1', 'int-tag-1', step1Ok ? 'ok' : 'active',
      !cam.online ? '相机离线' : (mismatch ? '需切档位' : (cam.corners >= 8 ? '识别正常 ✓' : `角点 ${cam.corners}/8`)));
    setStepState('int-step-2', 'int-tag-2', count >= 15 ? 'ok' : (count >= 6 ? 'active' : ''),
      `已拍 ${count}/15 张`);
    setStepState('int-step-3', 'int-tag-3', savedRms !== null && savedRms !== undefined ? 'ok' : (result && result.ok ? 'active' : ''),
      savedRms !== null && savedRms !== undefined ? `已保存 (RMS ${savedRms})` : (result && result.ok ? '待保存' : '待求解'));

    coachCard.className = 'coach-card';
    if (!cam.online) {
      coachCard.classList.add('warn');
      titleEl.textContent = `🧭 下一步：请先启动 ${STATE.arm} 的腕部相机节点`;
      bodyEl.textContent = `当前未收到 ${cam.topic} 图像。请在终端运行：ros2 launch camera_node camera.launch.py arm:=${STATE.arm}（或加 start_camera:=true 启动标定）。`;
    } else if (mismatch) {
      coachCard.classList.add('warn');
      titleEl.textContent = `🧭 下一步：点击相机右上角「切换」切换到 ${pKey} 档位`;
      bodyEl.textContent = `相机当前在推分辨率为 ${liveKey}，与下拉框选中的 ${pKey} 不一致，请先点「切换」等待画面变为 ${pKey}。`;
    } else if (savedRms !== null && savedRms !== undefined && (!result || !result.ok)) {
      coachCard.classList.add('good');
      titleEl.textContent = `✓ ${pKey} 档位内参已保存（RMS = ${savedRms} px）`;
      bodyEl.textContent = '你可以切换右上角下拉框标定另一个分辨率档位，或直接点击页面顶部「② 第二步：外参（手眼）标定」开始手眼标定。';
    } else if (result && result.ok) {
      const outliers = result.outliers || [];
      if (outliers.length > 0) {
        coachCard.classList.add('warn');
        titleEl.textContent = `🧭 下一步：检测到 ${outliers.length} 张误差偏大的离群图，建议点「删离群」或直接「保存」`;
        bodyEl.textContent = `当前求解 RMS = ${result.rms} px，整体覆盖率 ${(result.coverage * 100).toFixed(0)}%。点击「删离群」可自动删掉标红图像并重算；确认满意后务必点击「保存」。`;
      } else {
        coachCard.classList.add('good');
        titleEl.textContent = `🧭 下一步：内参求解完成（RMS = ${result.rms} px），请点击「保存」！`;
        bodyEl.textContent = `当前整体画面覆盖率 ${(result.coverage * 100).toFixed(0)}%，点击相机卡片上的「保存」按钮后内参才会正式写入 calibration.yaml。`;
      }
    } else if (count < 6) {
      titleEl.textContent = `🧭 下一步：换不同角度拍摄标定板（已拍 ${count} 张，至少需 6 张、推荐 15 张）`;
      bodyEl.textContent = cam.corners >= 8
        ? `当前画面已认出 ${cam.corners} 个角点，保持稳定点击「📸 拍摄当前帧」。记得把板子放到画面中心、四个角落并倾斜 15°～35° 各拍几张。`
        : `请将标定板对准镜头并保持光线清晰（当前仅识别 ${cam.corners} 个角点，至少需要 8 个角点才能拍摄）。`;
    } else if (count < 15) {
      titleEl.textContent = `🧭 下一步：已拍 ${count} 张（可点「求解」预览，推荐补拍四角至 15 张以上）`;
      bodyEl.textContent = '把标定板移到画面四个边缘角落、做上下俯仰和左右侧倾补拍几张能显著提升畸变标定精度；也可以现在点击「求解」查看当前误差。';
    } else {
      coachCard.classList.add('good');
      titleEl.textContent = `🧭 下一步：已拍满 ${count} 张，请点击相机卡片上的「求解」按钮！`;
      bodyEl.textContent = '求解后核对 RMS 重投影误差（小于 0.8 px 即佳），最后点击「保存」写入配置文件。';
    }
    return;
  }

  // 外参 tab 引导
  const poses = STATE.poses || [];
  const extRes = ((STATE.results.extrinsic || {}).cameras || {})[cam.name];
  const savedExt = STATE.saved_extrinsic;
  const link = (STATE.tf.links || {})[cam.name] || {};
  const motionOk = STATE.motion && STATE.motion.ok;
  const preflightOk = motionOk && link.ok && cam.online && cam.intrinsic_source !== 'none' && cam.corners >= 6;

  setStepState('ext-step-1', 'ext-tag-1', cam.intrinsic_source !== 'none' && cam.online ? 'ok' : 'active',
    cam.intrinsic_source === 'none' ? '缺内参' : (cam.online ? '就绪 ✓' : '相机离线'));
  setStepState('ext-step-2', 'ext-tag-2', poses.length >= 6 ? 'ok' : (poses.length >= 3 ? 'active' : ''),
    `已采 ${poses.length} 组`);
  setStepState('ext-step-3', 'ext-tag-3', savedExt ? 'ok' : (extRes ? 'active' : ''),
    savedExt ? '已保存 ✓' : (extRes ? '待保存' : '待求解'));

  coachCard.className = 'coach-card';
  if (cam.intrinsic_source === 'none') {
    coachCard.classList.add('warn');
    titleEl.textContent = '🧭 下一步：当前分辨率还没有内参，请先回「① 第一步：内参标定」完成标定并保存';
    bodyEl.textContent = '外参标定必须依赖当前分辨率的相机内参才能通过 PnP 算出标定板 3D 位姿。';
  } else if (extRes && extRes.joint && extRes.joint.ok) {
    coachCard.classList.add('good');
    titleEl.textContent = `🧭 下一步：外参求解成功（板位姿残差 ${num(extRes.joint.consistency.trans_rms_mm, 2)} mm rms），请点击「保存这一组」！`;
    bodyEl.textContent = '核对下方「非线性联合最小二乘精化（推荐）」与「AX=XB 经典手眼」结果一致后，点击「保存这一组」写入 calibration.yaml。';
  } else if (!preflightOk && poses.length < 3) {
    coachCard.classList.add('warn');
    titleEl.textContent = `🧭 下一步：请固定好标定板，移动 ${STATE.arm} 对准标定板并停稳（已采 ${poses.length} 组，至少需 3 组）`;
    bodyEl.textContent = '请查看下方「采集前检查」中带 ✗ 的项目：确保机械臂驱动已启动、标定板在画面内（角点 ≥ 6），且停手静置 0.5 秒后再点「📸 采集一组姿态」。';
  } else {
    const tipIdx = Math.min(poses.length, POSE_COACH_TIPS.length - 1);
    if (poses.length >= 6) coachCard.classList.add('good');
    titleEl.textContent = `🧭 手眼采集向导（当前已采 ${poses.length} 组${poses.length >= 3 ? '，已可点击「🧮 求解外参」' : '，至少需 3 组'}）`;
    bodyEl.textContent = POSE_COACH_TIPS[tipIdx];
  }
}


let REF_SIG = '';
function syncRefForm(ref) {
  if (!ref) return;
  const card = $('ref-point-card');
  if (card && card.contains(document.activeElement)) return;
  const sig = JSON.stringify([ref.variant, ref.link6_to_top_z_mm]);
  if (sig === REF_SIG) return;
  REF_SIG = sig;
  $('ref-variant').value = ref.variant || 'gripper_with_wrench';
  $('ref-z-mm').value = ref.link6_to_top_z_mm ?? 92.8;
}
$('ref-variant').onchange = () => {
  const v = $('ref-variant').value;
  if (v === 'gripper_with_wrench') $('ref-z-mm').value = 92.8;
  else if (v === 'with_flange') $('ref-z-mm').value = 63.0;
  else if (v === 'without_flange') $('ref-z-mm').value = 52.5;
};
$('btn-apply-ref').onclick = (e) => guard(e.target, async () => {
  const variant = $('ref-variant').value;
  const mm = Number($('ref-z-mm').value);
  const out = await post('/api/reference_point', { variant, link6_to_top_z_mm: mm });
  REF_SIG = '';
  out._banner = [`外参参考点已更新为 gripper_base_top（link6 +Z = ${out.link6_to_top_z_mm} mm）`, 'good'];
  return out;
});
