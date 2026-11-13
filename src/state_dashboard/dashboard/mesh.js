"use strict";

window.PiperMesh = (() => {
  const JOINT_ORIGINS = [
    {xyz: [0, 0, 0.123], rpy: [0, 0, 3.1415926]},
    {xyz: [0, 0, 0], rpy: [-1.5707963, -3.005806, -3.1415926]},
    {xyz: [0.28503, 0, 0], rpy: [0, 0, 2.8380798]},
    {xyz: [0.27364, 0, 0], rpy: [0, 0, 0.0806342]},
    {xyz: [0.07466, 0, 0], rpy: [-1.5707963, 1.5707963, 0]},
    {xyz: [0, -0.035, 0], rpy: [1.5707963, 0, 0]},
  ];

  const LINK_COLORS = [
    [0.22, 0.25, 0.30], // base_link: dark anodized graphite
    [0.78, 0.81, 0.86], // link1: metallic silver
    [0.82, 0.85, 0.90], // link2: brushed aluminum
    [0.78, 0.81, 0.86], // link3: metallic silver
    [0.74, 0.78, 0.84], // link4: metallic silver
    [0.26, 0.30, 0.36], // link5: dark wrist housing
    [0.20, 0.23, 0.28], // link6: flange dark graphite
    [0.25, 0.29, 0.35], // gripper_base / teach_base: dark housing
    [0.16, 0.42, 0.72], // link7: finger L accent
    [0.16, 0.42, 0.72], // link8: finger R accent
  ];

  const GHOST_COLOR = [0.15, 0.55, 0.95];
  const meshCache = new Map();
  const viewers = new Set();
  let sharedGL = null;

  function mat4Identity() {
    return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  }

  function mat4Mul(a, b) {
    const out = new Float32Array(16);
    for (let c = 0; c < 4; c++) {
      for (let r = 0; r < 4; r++) {
        out[c * 4 + r] =
          a[0 * 4 + r] * b[c * 4 + 0] +
          a[1 * 4 + r] * b[c * 4 + 1] +
          a[2 * 4 + r] * b[c * 4 + 2] +
          a[3 * 4 + r] * b[c * 4 + 3];
      }
    }
    return out;
  }

  function mat4FromOriginRpy(xyz, rpy, qz = 0, tz = 0) {
    const [rx, ry, rz] = rpy;
    const cx = Math.cos(rx), sx = Math.sin(rx);
    const cy = Math.cos(ry), sy = Math.sin(ry);
    const cz = Math.cos(rz), sz = Math.sin(rz);
    let r00 = cz * cy, r01 = cz * sy * sx - sz * cx, r02 = cz * sy * cx + sz * sx;
    let r10 = sz * cy, r11 = sz * sy * sx + cz * cx, r12 = sz * sy * cx - cz * sx;
    let r20 = -sy, r21 = cy * sx, r22 = cy * cx;
    const tx = xyz[0] + r02 * tz;
    const ty = xyz[1] + r12 * tz;
    const tzz = xyz[2] + r22 * tz;
    if (qz !== 0) {
      const cq = Math.cos(qz), sq = Math.sin(qz);
      const n00 = r00 * cq + r01 * sq, n01 = -r00 * sq + r01 * cq;
      const n10 = r10 * cq + r11 * sq, n11 = -r10 * sq + r11 * cq;
      const n20 = r20 * cq + r21 * sq, n21 = -r20 * sq + r21 * cq;
      r00 = n00; r01 = n01;
      r10 = n10; r11 = n11;
      r20 = n20; r21 = n21;
    }
    return new Float32Array([
      r00, r10, r20, 0,
      r01, r11, r21, 0,
      r02, r12, r22, 0,
      tx, ty, tzz, 1,
    ]);
  }

  function mat4Perspective(fovY, aspect, near, far) {
    const f = 1 / Math.tan(fovY / 2);
    const nf = 1 / (near - far);
    return new Float32Array([
      f / aspect, 0, 0, 0,
      0, f, 0, 0,
      0, 0, (far + near) * nf, -1,
      0, 0, 2 * far * near * nf, 0,
    ]);
  }

  function mat4LookAt(eye, target, up) {
    let zx = eye[0] - target[0], zy = eye[1] - target[1], zz = eye[2] - target[2];
    let len = Math.hypot(zx, zy, zz) || 1;
    zx /= len; zy /= len; zz /= len;
    let xx = up[1] * zz - up[2] * zy;
    let xy = up[2] * zx - up[0] * zz;
    let xz = up[0] * zy - up[1] * zx;
    len = Math.hypot(xx, xy, xz) || 1;
    xx /= len; xy /= len; xz /= len;
    const yx = zy * xz - zz * xy;
    const yy = zz * xx - zx * xz;
    const yz = zx * xy - zy * xx;
    return new Float32Array([
      xx, yx, zx, 0,
      xy, yy, zy, 0,
      xz, yz, zz, 0,
      -(xx * eye[0] + xy * eye[1] + xz * eye[2]),
      -(yx * eye[0] + yy * eye[1] + yz * eye[2]),
      -(zx * eye[0] + zy * eye[1] + zz * eye[2]),
      1,
    ]);
  }

  function computeFK(jointsRad, gripperM = 0.02, effector = "gripper") {
    const q = Array.isArray(jointsRad) && jointsRad.length === 6
      ? jointsRad.map(v => Number.isFinite(v) ? v : 0)
      : [0, 0, 0, 0, 0, 0];
    const width = Number.isFinite(gripperM) ? Math.max(0, Math.min(0.1, gripperM)) : 0.02;
    const links = [];
    let current = mat4Identity();
    links.push({mesh: "base_link.STL", matrix: current, color: LINK_COLORS[0]});
    const isWrench = effector === "gripper_with_wrench";
    for (let i = 0; i < 6; i++) {
      const qEff = (i === 5 && isWrench) ? q[i] + Math.PI / 4 : q[i];
      current = mat4Mul(current, mat4FromOriginRpy(JOINT_ORIGINS[i].xyz, JOINT_ORIGINS[i].rpy, qEff, 0));
      links.push({mesh: `link${i + 1}.STL`, matrix: current, color: LINK_COLORS[i + 1]});
    }
    if (isWrench) {
      const mArmFlange = current;
      const mSensor = mat4Mul(mArmFlange, mat4FromOriginRpy([0, 0, 0.0075], [0, 0, -Math.PI / 4], 0, 0));
      const mGripFlange = mat4Mul(mSensor, mat4FromOriginRpy([0, 0, 0.024], [0, 0, Math.PI / 2], 0, 0));
      const mGripBase = mat4Mul(mGripFlange, mat4FromOriginRpy([0, 0, 0.0088], [0, 0, 0], 0, 0));
      const m7 = mat4Mul(mGripBase, mat4FromOriginRpy([0, 0, 0.1253], [1.5708, 0, 0], 0, width / 2));
      const m8 = mat4Mul(mGripBase, mat4FromOriginRpy([0, 0, 0.1253], [1.5708, 0, -3.1416], 0, width / 2));
      links.push({mesh: "px6d75_arm_flange.stl", matrix: mArmFlange, color: [0.65, 0.72, 0.80]});
      links.push({mesh: "px6d75.stl", matrix: mSensor, color: [0.72, 0.74, 0.77]});
      links.push({mesh: "px6d75_gripper_flange.stl", matrix: mGripFlange, color: [0.86, 0.65, 0.33]});
      links.push({mesh: "gripper_base_without_flange.STL", matrix: mGripBase, color: LINK_COLORS[7]});
      links.push({mesh: "link7.STL", matrix: m7, color: LINK_COLORS[8]});
      links.push({mesh: "link8.STL", matrix: m8, color: LINK_COLORS[9]});
      return {links, flangeMatrix: current, gripperBaseMatrix: mGripBase, gripperTopZ: 0.0525};
    }
    const isTeach = effector === "teaching_handle";
    const baseMesh = isTeach ? "teach_base.STL" : "gripper_base.STL";
    const fingerL = isTeach ? "teach_link7.STL" : "link7.STL";
    const fingerR = isTeach ? "teach_link8.STL" : "link8.STL";
    const fingerZ = isTeach ? 0.0795 : 0.1358;
    const mTermBase = mat4Mul(current, mat4FromOriginRpy([0, 0, 0], [0, 0, Math.PI / 2], 0, 0));
    links.push({mesh: baseMesh, matrix: mTermBase, color: LINK_COLORS[7]});
    const m7 = mat4Mul(mTermBase, mat4FromOriginRpy([0, 0, fingerZ], [1.5708, 0, 0], 0, width / 2));
    const m8 = mat4Mul(mTermBase, mat4FromOriginRpy([0, 0, fingerZ], [1.5708, 0, -3.1416], 0, width / 2));
    links.push({mesh: fingerL, matrix: m7, color: LINK_COLORS[8]});
    links.push({mesh: fingerR, matrix: m8, color: LINK_COLORS[9]});
    return {links, flangeMatrix: current, gripperBaseMatrix: mTermBase, gripperTopZ: 0.063};
  }

  function parseBinaryStl(buffer) {
    if (buffer.byteLength < 84) return null;
    const view = new DataView(buffer);
    const triCount = view.getUint32(80, true);
    if (84 + triCount * 50 > buffer.byteLength) return null;
    const interleaved = new Float32Array(triCount * 18);
    let outIdx = 0;
    for (let i = 0; i < triCount; i++) {
      const offset = 84 + i * 50;
      let nx = view.getFloat32(offset, true);
      let ny = view.getFloat32(offset + 4, true);
      let nz = view.getFloat32(offset + 8, true);
      const v0x = view.getFloat32(offset + 12, true);
      const v0y = view.getFloat32(offset + 16, true);
      const v0z = view.getFloat32(offset + 20, true);
      const v1x = view.getFloat32(offset + 24, true);
      const v1y = view.getFloat32(offset + 28, true);
      const v1z = view.getFloat32(offset + 32, true);
      const v2x = view.getFloat32(offset + 36, true);
      const v2y = view.getFloat32(offset + 40, true);
      const v2z = view.getFloat32(offset + 44, true);
      if (nx * nx + ny * ny + nz * nz < 0.01) {
        const ux = v1x - v0x, uy = v1y - v0y, uz = v1z - v0z;
        const vx = v2x - v0x, vy = v2y - v0y, vz = v2z - v0z;
        nx = uy * vz - uz * vy;
        ny = uz * vx - ux * vz;
        nz = ux * vy - uy * vx;
        const len = Math.hypot(nx, ny, nz) || 1;
        nx /= len; ny /= len; nz /= len;
      }
      interleaved[outIdx++] = v0x; interleaved[outIdx++] = v0y; interleaved[outIdx++] = v0z;
      interleaved[outIdx++] = nx;  interleaved[outIdx++] = ny;  interleaved[outIdx++] = nz;
      interleaved[outIdx++] = v1x; interleaved[outIdx++] = v1y; interleaved[outIdx++] = v1z;
      interleaved[outIdx++] = nx;  interleaved[outIdx++] = ny;  interleaved[outIdx++] = nz;
      interleaved[outIdx++] = v2x; interleaved[outIdx++] = v2y; interleaved[outIdx++] = v2z;
      interleaved[outIdx++] = nx;  interleaved[outIdx++] = ny;  interleaved[outIdx++] = nz;
    }
    return {vertexCount: triCount * 3, interleaved, triCount};
  }

  function ensureMesh(name) {
    if (meshCache.has(name)) return meshCache.get(name);
    const entry = {status: "loading", data: null};
    meshCache.set(name, entry);
    fetch(`/meshes/${name}`)
      .then(res => res.ok ? res.arrayBuffer() : Promise.reject(new Error(`HTTP ${res.status}`)))
      .then(buf => {
        entry.data = parseBinaryStl(buf);
        entry.status = entry.data ? "ready" : "error";
        for (const viewer of viewers) viewer.scheduleDraw();
      })
      .catch(() => {
        entry.status = "error";
      });
    return entry;
  }

  function buildAxesLines(matrix, length = 0.075) {
    const ox = matrix[12], oy = matrix[13], oz = matrix[14];
    // Gripper local frame: +x left (matrix col 1), +y up (-matrix col 0), +z forward along gripper (matrix col 2)
    const xx = ox + matrix[4] * length, xy = oy + matrix[5] * length, xz = oz + matrix[6] * length;
    const yx = ox - matrix[0] * length, yy = oy - matrix[1] * length, yz = oz - matrix[2] * length;
    const zx = ox + matrix[8] * length, zy = oy + matrix[9] * length, zz = oz + matrix[10] * length;
    return new Float32Array([
      ox, oy, oz, xx, xy, xz,
      ox, oy, oz, yx, yy, yz,
      ox, oy, oz, zx, zy, zz,
    ]);
  }

  function buildStandardAxesLines(matrix, length = 0.055) {
    const ox = matrix[12], oy = matrix[13], oz = matrix[14];
    const xx = ox + matrix[0] * length, xy = oy + matrix[1] * length, xz = oz + matrix[2] * length;
    const yx = ox + matrix[4] * length, yy = oy + matrix[5] * length, yz = oz + matrix[6] * length;
    const zx = ox + matrix[8] * length, zy = oy + matrix[9] * length, zz = oz + matrix[10] * length;
    return new Float32Array([
      ox, oy, oz, xx, xy, xz,
      ox, oy, oz, yx, yy, yz,
      ox, oy, oz, zx, zy, zz,
    ]);
  }

  function transformPoint(m, x, y, z) {
    return [
      m[0] * x + m[4] * y + m[8] * z + m[12],
      m[1] * x + m[5] * y + m[9] * z + m[13],
      m[2] * x + m[6] * y + m[10] * z + m[14],
    ];
  }

  function resolveCameraPoses(fk, calib) {
    if (!fk || !calib || !Array.isArray(calib.translation) || !Array.isArray(calib.rpy)) return null;
    // Camera extrinsics are calibrated relative to PiPER-X link6 translated along +Z to gripper_base_top
    const topZ = Number.isFinite(calib.link6_to_top_z_m) ? calib.link6_to_top_z_m : (fk.gripperTopZ ?? 0.063);
    const topMatrix = mat4Mul(fk.flangeMatrix, mat4FromOriginRpy([0, 0, topZ], [0, 0, 0], 0, 0));
    const extMat = mat4FromOriginRpy(calib.translation, calib.rpy, 0, 0);
    const camMatrix = mat4Mul(topMatrix, extMat);
    let boardMatrix = null;
    if (calib.board_pose && Array.isArray(calib.board_pose.translation) && Array.isArray(calib.board_pose.rpy)) {
      boardMatrix = mat4FromOriginRpy(calib.board_pose.translation, calib.board_pose.rpy, 0, 0);
    }
    const intr = Array.isArray(calib.intrinsics) && calib.intrinsics.length ? calib.intrinsics[0] : null;
    const fovDeg = intr && Array.isArray(intr.fov_deg) && intr.fov_deg.length >= 2 ? intr.fov_deg : [50.0, 35.0];
    return {topMatrix, camMatrix, boardMatrix, fovDeg, intr, calib};
  }

  function buildCameraFrustumLines(topMatrix, camMatrix, fovDeg = [50.0, 35.0], depth = 0.14) {
    const o = [camMatrix[12], camMatrix[13], camMatrix[14]];
    const top = [topMatrix[12], topMatrix[13], topMatrix[14]];
    const hx = Math.tan((fovDeg[0] || 50) * Math.PI / 360) * depth;
    const hy = Math.tan((fovDeg[1] || 35) * Math.PI / 360) * depth;
    const c0 = transformPoint(camMatrix, -hx, -hy, depth);
    const c1 = transformPoint(camMatrix,  hx, -hy, depth);
    const c2 = transformPoint(camMatrix,  hx,  hy, depth);
    const c3 = transformPoint(camMatrix, -hx,  hy, depth);
    const up0 = transformPoint(camMatrix, -hx * 0.35, -hy * 1.05, depth);
    const up1 = transformPoint(camMatrix,  hx * 0.35, -hy * 1.05, depth);
    const upTip = transformPoint(camMatrix, 0, -hy * 1.32, depth);
    return new Float32Array([
      top[0], top[1], top[2], o[0], o[1], o[2],
      o[0], o[1], o[2], c0[0], c0[1], c0[2],
      o[0], o[1], o[2], c1[0], c1[1], c1[2],
      o[0], o[1], o[2], c2[0], c2[1], c2[2],
      o[0], o[1], o[2], c3[0], c3[1], c3[2],
      c0[0], c0[1], c0[2], c1[0], c1[1], c1[2],
      c1[0], c1[1], c1[2], c2[0], c2[1], c2[2],
      c2[0], c2[1], c2[2], c3[0], c3[1], c3[2],
      c3[0], c3[1], c3[2], c0[0], c0[1], c0[2],
      up0[0], up0[1], up0[2], upTip[0], upTip[1], upTip[2],
      upTip[0], upTip[1], upTip[2], up1[0], up1[1], up1[2],
    ]);
  }

  function buildGridGeometry() {
    const lines = [];
    const extent = 0.4;
    const step = 0.1;
    for (let i = -4; i <= 4; i++) {
      const p = i * step;
      lines.push(p, -extent, 0, p, extent, 0);
      lines.push(-extent, p, 0, extent, p, 0);
    }
    lines.push(0, 0, 0, 0.12, 0, 0, 0, 0, 0, 0, 0.12, 0, 0, 0, 0, 0, 0, 0.12);
    return new Float32Array(lines);
  }

  const GRID_DATA = buildGridGeometry();

  function getSharedGL() {
    if (sharedGL !== null) return sharedGL;
    const offscreen = document.createElement("canvas");
    offscreen.width = 640;
    offscreen.height = 520;
    const gl = offscreen.getContext("webgl", {antialias: true, alpha: false, preserveDrawingBuffer: true});
    if (!gl) {
      sharedGL = false;
      return false;
    }
    const vsSource = `
      attribute vec3 aPos;
      attribute vec3 aNorm;
      uniform mat4 uMVP;
      uniform mat4 uModel;
      varying vec3 vNorm;
      void main() {
        vNorm = mat3(uModel[0].xyz, uModel[1].xyz, uModel[2].xyz) * aNorm;
        gl_Position = uMVP * vec4(aPos, 1.0);
      }
    `;
    const fsSource = `
      precision mediump float;
      varying vec3 vNorm;
      uniform vec4 uColor;
      uniform float uUnlit;
      void main() {
        if (uUnlit > 0.5) {
          gl_FragColor = uColor;
          return;
        }
        vec3 n = normalize(vNorm);
        vec3 l1 = normalize(vec3(0.55, -0.55, 0.63));
        vec3 l2 = normalize(vec3(-0.60, 0.50, 0.45));
        vec3 halfVec = normalize(l1 + vec3(0.0, -0.6, 0.8));
        float d1 = abs(dot(n, l1));
        float d2 = max(dot(n, l2), 0.0);
        float hemi = 0.5 + 0.5 * n.z;
        float spec = pow(abs(dot(n, halfVec)), 28.0) * 0.22;
        vec3 base = uColor.rgb * (0.28 + 0.16 * hemi + 0.44 * d1 + 0.18 * d2) + vec3(spec);
        gl_FragColor = vec4(clamp(base, 0.0, 1.0), uColor.a);
      }
    `;
    function compile(type, src) {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      return gl.getShaderParameter(s, gl.COMPILE_STATUS) ? s : null;
    }
    const vs = compile(gl.VERTEX_SHADER, vsSource);
    const fs = compile(gl.FRAGMENT_SHADER, fsSource);
    if (!vs || !fs) {
      sharedGL = false;
      return false;
    }
    const prog = gl.createProgram();
    gl.attachShader(prog, vs);
    gl.attachShader(prog, fs);
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
      sharedGL = false;
      return false;
    }
    const gridBuf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, gridBuf);
    gl.bufferData(gl.ARRAY_BUFFER, GRID_DATA, gl.STATIC_DRAW);
    const dynBuf = gl.createBuffer();
    const rendererInfo = String(gl.getParameter(gl.RENDERER) || "");
    sharedGL = {
      offscreen,
      gl,
      prog,
      isSoftware: /SwiftShader|llvmpipe|Software/i.test(rendererInfo),
      aPos: gl.getAttribLocation(prog, "aPos"),
      aNorm: gl.getAttribLocation(prog, "aNorm"),
      uMVP: gl.getUniformLocation(prog, "uMVP"),
      uModel: gl.getUniformLocation(prog, "uModel"),
      uColor: gl.getUniformLocation(prog, "uColor"),
      uUnlit: gl.getUniformLocation(prog, "uUnlit"),
      gridBuf,
      dynBuf,
      gridVertexCount: GRID_DATA.length / 3,
      buffers: new Map(),
    };
    return sharedGL;
  }

  function hasGhostDifference(joints, ghost) {
    if (!Array.isArray(ghost) || ghost.length !== 6) return false;
    return ghost.some((v, i) => Number.isFinite(v) && Math.abs(v - (joints[i] || 0)) > 0.008);
  }

  function createViewer(canvas, options = {}) {
    const ctx2d = canvas.getContext("2d");
    const viewer = {
      canvas,
      ctx2d,
      yaw: options.yaw ?? -0.78,
      pitch: options.pitch ?? 0.36,
      distance: options.distance ?? 1.05,
      target: options.target ? [...options.target] : [0.04, 0.0, 0.26],
      joints: [0, 0, 0, 0, 0, 0],
      ghostJoints: null,
      targetPose: null,
      wrench: null,
      displayWrench: null,
      gripper: 0.02,
      effector: "gripper",
      cameraCalib: null,
      wireframe: false,
      showGhost: true,
      showCamera: true,
      _raf: 0,
      _signature: "",
      scheduleDraw() {
        if (this._raf) return;
        this._raf = requestAnimationFrame(() => {
          this._raf = 0;
          this.draw();
        });
      },
      setPreset(preset) {
        if (preset === "front") {
          this.yaw = 0; this.pitch = 0.15; this.distance = 1.05; this.target = [0.06, 0, 0.26];
        } else if (preset === "side") {
          this.yaw = -Math.PI / 2; this.pitch = 0.15; this.distance = 1.05; this.target = [0.06, 0, 0.26];
        } else if (preset === "top") {
          this.yaw = -Math.PI / 2; this.pitch = 1.48; this.distance = 1.15; this.target = [0.06, 0, 0.18];
        } else {
          this.yaw = -0.78; this.pitch = 0.36; this.distance = 1.05; this.target = [0.04, 0, 0.26];
        }
        this.scheduleDraw();
      },
      update(state) {
        if (Array.isArray(state.joints) && state.joints.length === 6) this.joints = state.joints;
        if ("ghostJoints" in state) this.ghostJoints = state.ghostJoints;
        if ("targetPose" in state) this.targetPose = state.targetPose;
        if ("wrench" in state) this.wrench = state.wrench;
        if (Number.isFinite(state.gripper)) this.gripper = state.gripper;
        if (state.effector) this.effector = state.effector;
        if ("cameraCalib" in state) this.cameraCalib = state.cameraCalib;
        const poseSig = this.targetPose ? this.targetPose.map(v => v.toFixed(3)).join(",") : "";
        const wrenchSig = Array.isArray(this.wrench) ? this.wrench.map(v => Number.isFinite(v) ? v.toFixed(2) : "0").join(",") : "";
        const camSig = this.cameraCalib ? (this.cameraCalib.stamp || "1") + ":" + (this.cameraCalib.translation || []).join(",") : "";
        const sig = `${this.effector}|${this.gripper.toFixed(4)}|${this.joints.map(v => v.toFixed(3)).join(",")}|${this.ghostJoints ? this.ghostJoints.map(v => v.toFixed(3)).join(",") : ""}|${poseSig}|${wrenchSig}|${camSig}|${this.showCamera}`;
        if (sig !== this._signature) {
          this._signature = sig;
          this.scheduleDraw();
        }
      },
      draw() {
        if (!ctx2d) return;
        const dialog = canvas.closest("dialog");
        if (dialog && !dialog.open) return;
        const rect = canvas.getBoundingClientRect();
        if (rect.width === 0 || rect.height === 0) return;
        const glState = getSharedGL();
        const isSoftware = glState && glState.isSoftware;
        const dpr = isSoftware ? 1 : Math.min(window.devicePixelRatio || 1, 2);
        const w = Math.max(1, Math.round(rect.width * dpr));
        const h = Math.max(1, Math.round(rect.height * dpr));
        if (canvas.width !== w || canvas.height !== h) {
          canvas.width = w;
          canvas.height = h;
        }
        const ex = this.target[0] + this.distance * Math.cos(this.pitch) * Math.cos(this.yaw);
        const ey = this.target[1] + this.distance * Math.cos(this.pitch) * Math.sin(this.yaw);
        const ez = this.target[2] + this.distance * Math.sin(this.pitch);
        const vp = mat4Mul(
          mat4Perspective(0.65, w / h, 0.02, 5.0),
          mat4LookAt([ex, ey, ez], this.target, [0, 0, 1]),
        );
        const fk = computeFK(this.joints, this.gripper, this.effector);
        const ghostFk = (this.showGhost && hasGhostDifference(this.joints, this.ghostJoints))
          ? computeFK(this.ghostJoints, this.gripper, this.effector)
          : null;
        const targetMat = (Array.isArray(this.targetPose) && this.targetPose.length === 6 && this.targetPose.every(Number.isFinite))
          ? mat4FromOriginRpy(this.targetPose.slice(0, 3), this.targetPose.slice(3, 6), 0, 0)
          : null;
        let wrenchToDraw = this.wrench;
        if (Array.isArray(this.wrench) && this.wrench.length >= 6 && this.wrench.every(Number.isFinite)) {
          if (!Array.isArray(this.displayWrench) || this.displayWrench.length < 6) {
            this.displayWrench = [...this.wrench];
          } else {
            let moving = false;
            for (let i = 0; i < 6; i++) {
              const diff = this.wrench[i] - this.displayWrench[i];
              if (Math.abs(diff) > 0.015) moving = true;
              this.displayWrench[i] += 0.42 * diff;
            }
            if (moving) this.scheduleDraw();
          }
          wrenchToDraw = this.displayWrench;
        } else {
          this.displayWrench = null;
        }
        if (glState) {
          const camPose = this.showCamera ? resolveCameraPoses(fk, this.cameraCalib) : null;
          drawSharedWebGL(glState, ctx2d, w, h, vp, fk, ghostFk, targetMat, wrenchToDraw, this.wireframe, camPose);
        } else {
          drawSkeletonFallback(ctx2d, w, h, vp, fk);
        }
      },
    };

    if (options.interactive !== false) {
      let dragging = false;
      let panMode = false;
      let lastX = 0, lastY = 0;
      canvas.addEventListener("pointerdown", e => {
        dragging = true;
        panMode = e.button === 2 || e.shiftKey;
        lastX = e.clientX;
        lastY = e.clientY;
        canvas.setPointerCapture(e.pointerId);
      });
      canvas.addEventListener("pointermove", e => {
        if (!dragging) return;
        const dx = e.clientX - lastX;
        const dy = e.clientY - lastY;
        lastX = e.clientX;
        lastY = e.clientY;
        if (panMode) {
          const scale = viewer.distance * 0.0018;
          viewer.target[0] -= (-Math.sin(viewer.yaw) * dx) * scale;
          viewer.target[1] -= (Math.cos(viewer.yaw) * dx) * scale;
          viewer.target[2] += dy * scale;
        } else {
          viewer.yaw -= dx * 0.008;
          viewer.pitch = Math.max(-1.2, Math.min(1.5, viewer.pitch + dy * 0.008));
        }
        viewer.draw();
      });
      const stopDrag = () => { dragging = false; };
      canvas.addEventListener("pointerup", stopDrag);
      canvas.addEventListener("pointercancel", stopDrag);
      canvas.addEventListener("contextmenu", e => e.preventDefault());
      canvas.addEventListener("wheel", e => {
        e.preventDefault();
        viewer.distance = Math.max(0.25, Math.min(2.2, viewer.distance * (1 + Math.sign(e.deltaY) * 0.09)));
        viewer.draw();
      }, {passive: false});
    }

    viewers.add(viewer);
    return viewer;
  }

  function drawSharedWebGL(ctx, target2d, w, h, vp, fk, ghostFk, targetMat, wrench, wireframe, camPose) {
    const {offscreen, gl, prog, aPos, aNorm, uMVP, uModel, uColor, uUnlit, gridBuf, dynBuf, gridVertexCount, buffers} = ctx;
    if (offscreen.width < w || offscreen.height < h) {
      offscreen.width = Math.max(offscreen.width, w);
      offscreen.height = Math.max(offscreen.height, h);
    }
    gl.viewport(0, 0, w, h);
    gl.scissor(0, 0, w, h);
    gl.enable(gl.SCISSOR_TEST);
    gl.clearColor(0.955, 0.968, 0.985, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.useProgram(prog);

    const ident = mat4Identity();
    gl.bindBuffer(gl.ARRAY_BUFFER, gridBuf);
    gl.enableVertexAttribArray(aPos);
    gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 12, 0);
    gl.disableVertexAttribArray(aNorm);
    gl.uniformMatrix4fv(uMVP, false, vp);
    gl.uniformMatrix4fv(uModel, false, ident);
    gl.uniform1f(uUnlit, 1.0);
    gl.uniform4f(uColor, 0.78, 0.83, 0.89, 1.0);
    gl.drawArrays(gl.LINES, 0, gridVertexCount - 6);
    gl.uniform4f(uColor, 0.85, 0.20, 0.25, 1.0);
    gl.drawArrays(gl.LINES, gridVertexCount - 6, 2);
    gl.uniform4f(uColor, 0.12, 0.62, 0.36, 1.0);
    gl.drawArrays(gl.LINES, gridVertexCount - 4, 2);
    gl.uniform4f(uColor, 0.15, 0.42, 0.82, 1.0);
    gl.drawArrays(gl.LINES, gridVertexCount - 2, 2);

    gl.uniform1f(uUnlit, 0.0);
    gl.enableVertexAttribArray(aNorm);
    const hasActiveForce = Array.isArray(wrench) && wrench.length >= 6 &&
      wrench.every(Number.isFinite) && Math.hypot(wrench[0], wrench[1], wrench[2]) > 0.3;
    const drawLinkList = (linkItems, alpha, overrideColor) => {
      for (const link of linkItems) {
        const entry = ensureMesh(link.mesh);
        if (entry.status !== "ready" || !entry.data) continue;
        let gpuBuf = buffers.get(link.mesh);
        if (!gpuBuf) {
          gpuBuf = gl.createBuffer();
          gl.bindBuffer(gl.ARRAY_BUFFER, gpuBuf);
          gl.bufferData(gl.ARRAY_BUFFER, entry.data.interleaved, gl.STATIC_DRAW);
          buffers.set(link.mesh, gpuBuf);
        } else {
          gl.bindBuffer(gl.ARRAY_BUFFER, gpuBuf);
        }
        gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 24, 0);
        gl.vertexAttribPointer(aNorm, 3, gl.FLOAT, false, 24, 12);
        const mvp = mat4Mul(vp, link.matrix);
        const c = overrideColor || link.color;
        gl.uniformMatrix4fv(uMVP, false, mvp);
        gl.uniformMatrix4fv(uModel, false, link.matrix);
        gl.uniform4f(uColor, c[0], c[1], c[2], alpha);
        gl.drawArrays(wireframe ? gl.LINES : gl.TRIANGLES, 0, entry.data.vertexCount);
      }
    };

    if (hasActiveForce && !wireframe) {
      drawLinkList(fk.links.slice(0, 7), 1.0, null);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      drawLinkList(fk.links.slice(7), 0.62, null);
      gl.disable(gl.BLEND);
    } else {
      drawLinkList(fk.links, 1.0, null);
    }
    if (ghostFk) {
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.depthMask(false);
      drawLinkList(ghostFk.links, 0.28, GHOST_COLOR);
      gl.depthMask(true);
      gl.disable(gl.BLEND);
    }
    const drawTriad = (mat, len) => {
      const axesData = buildAxesLines(mat, len);
      gl.bindBuffer(gl.ARRAY_BUFFER, dynBuf);
      gl.bufferData(gl.ARRAY_BUFFER, axesData, gl.STREAM_DRAW);
      gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 12, 0);
      gl.disableVertexAttribArray(aNorm);
      gl.uniformMatrix4fv(uMVP, false, vp);
      gl.uniformMatrix4fv(uModel, false, ident);
      gl.uniform1f(uUnlit, 1.0);
      gl.uniform4f(uColor, 0.92, 0.22, 0.26, 1.0);
      gl.drawArrays(gl.LINES, 0, 2);
      gl.uniform4f(uColor, 0.14, 0.72, 0.40, 1.0);
      gl.drawArrays(gl.LINES, 2, 2);
      gl.uniform4f(uColor, 0.18, 0.48, 0.92, 1.0);
      gl.drawArrays(gl.LINES, 4, 2);
    };
    drawTriad(fk.flangeMatrix, 0.065);
    if (camPose) {
      const frustumData = buildCameraFrustumLines(camPose.topMatrix, camPose.camMatrix, camPose.fovDeg, 0.14);
      gl.bindBuffer(gl.ARRAY_BUFFER, dynBuf);
      gl.bufferData(gl.ARRAY_BUFFER, frustumData, gl.STREAM_DRAW);
      gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 12, 0);
      gl.disableVertexAttribArray(aNorm);
      gl.uniformMatrix4fv(uMVP, false, vp);
      gl.uniformMatrix4fv(uModel, false, ident);
      gl.uniform1f(uUnlit, 1.0);
      gl.uniform4f(uColor, 0.95, 0.55, 0.08, 1.0);
      gl.drawArrays(gl.LINES, 0, 2);
      gl.uniform4f(uColor, 0.05, 0.68, 0.78, 1.0);
      gl.drawArrays(gl.LINES, 2, 20);
      const camAxes = buildStandardAxesLines(camPose.camMatrix, 0.05);
      gl.bufferData(gl.ARRAY_BUFFER, camAxes, gl.STREAM_DRAW);
      gl.uniform4f(uColor, 0.92, 0.22, 0.26, 1.0);
      gl.drawArrays(gl.LINES, 0, 2);
      gl.uniform4f(uColor, 0.14, 0.72, 0.40, 1.0);
      gl.drawArrays(gl.LINES, 2, 2);
      gl.uniform4f(uColor, 0.18, 0.48, 0.92, 1.0);
      gl.drawArrays(gl.LINES, 4, 2);
      const topAxes = buildStandardAxesLines(camPose.topMatrix, 0.035);
      gl.bufferData(gl.ARRAY_BUFFER, topAxes, gl.STREAM_DRAW);
      gl.uniform4f(uColor, 0.95, 0.55, 0.08, 1.0);
      gl.drawArrays(gl.LINES, 0, 6);
    }
    if (targetMat) {
      gl.disable(gl.DEPTH_TEST);
      drawTriad(targetMat, 0.085);
      gl.enable(gl.DEPTH_TEST);
    }
    gl.disable(gl.SCISSOR_TEST);
    target2d.clearRect(0, 0, w, h);
    target2d.drawImage(offscreen, 0, offscreen.height - h, w, h, 0, 0, w, h);
    if (camPose) {
      drawCameraOverlay2D(target2d, vp, camPose, w, h);
    }
    if (hasActiveForce) {
      drawForceOverlay2D(target2d, vp, fk.flangeMatrix, wrench, w, h);
    }
  }

  function drawCameraOverlay2D(ctx, vp, camPose, w, h) {
    const top = camPose.topMatrix;
    const cam = camPose.camMatrix;
    const pTop = projectPoint(vp, top[12], top[13], top[14], w, h);
    const pCam = projectPoint(vp, cam[12], cam[13], cam[14], w, h);
    const lookEnd = transformPoint(cam, 0, 0, 0.14);
    const pLook = projectPoint(vp, lookEnd[0], lookEnd[1], lookEnd[2], w, h);
    ctx.save();
    ctx.font = "600 11px ui-monospace, monospace";
    if (pTop) {
      ctx.beginPath();
      ctx.arc(pTop[0], pTop[1], 4, 0, Math.PI * 2);
      ctx.fillStyle = "#f28c18";
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = "#ffffff";
      ctx.stroke();
      ctx.fillStyle = "#b85d00";
      ctx.fillText("gripper_base_top", pTop[0] + 6, pTop[1] - 4);
    }
    if (pCam) {
      if (pLook) {
        ctx.beginPath();
        ctx.setLineDash([4, 3]);
        ctx.lineWidth = 2;
        ctx.strokeStyle = "#0c9ab0";
        ctx.moveTo(pCam[0], pCam[1]);
        ctx.lineTo(pLook[0], pLook[1]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.beginPath();
      ctx.arc(pCam[0], pCam[1], 5, 0, Math.PI * 2);
      ctx.fillStyle = "#0c9ab0";
      ctx.fill();
      ctx.lineWidth = 1.8;
      ctx.strokeStyle = "#ffffff";
      ctx.stroke();
      const name = camPose.calib?.camera || "camera";
      const t = camPose.calib?.translation || [0, 0, 0];
      const label = name + " [" + (t[0] * 1000).toFixed(1) + ", " + (t[1] * 1000).toFixed(1) + ", " + (t[2] * 1000).toFixed(1) + "]mm";
      const tw = ctx.measureText(label).width;
      const lx = pCam[0] + 8, ly = pCam[1] + 4;
      ctx.fillStyle = "rgba(255, 255, 255, 0.88)";
      ctx.fillRect(lx - 3, ly - 11, tw + 6, 15);
      ctx.strokeStyle = "#0c9ab0";
      ctx.lineWidth = 1;
      ctx.strokeRect(lx - 3, ly - 11, tw + 6, 15);
      ctx.fillStyle = "#075e6d";
      ctx.fillText(label, lx, ly);
    }
    ctx.restore();
  }

  function drawForceOverlay2D(ctx, vp, flangeMatrix, wrench, w, h) {
    const [fx, fy, fz] = wrench;
    const f2 = fx * fx + fy * fy + fz * fz;
    const fMag = Math.sqrt(f2);
    if (fMag <= 0.3) return;
    const m = flangeMatrix;
    const px = m[12], py = m[13], pz = m[14];
    const toWorldVec = (lx, ly, lz) => [
      m[4] * lx - m[0] * ly + m[8] * lz,
      m[5] * lx - m[1] * ly + m[9] * lz,
      m[6] * lx - m[2] * ly + m[10] * lz,
    ];
    const [fwx, fwy, fwz] = toWorldVec(fx, fy, fz);
    const arrowLen = Math.min(0.24, 0.045 + fMag * 0.008) / fMag;
    const qx = px + fwx * arrowLen, qy = py + fwy * arrowLen, qz = pz + fwz * arrowLen;

    const pAction = projectPoint(vp, px, py, pz, w, h);
    const pTip = projectPoint(vp, qx, qy, qz, w, h);
    if (!pAction || !pTip) return;

    ctx.save();
    // 1. X-ray force vector shaft from fixed wrench origin P -> Q with white contrast halo
    const dx = pTip[0] - pAction[0], dy = pTip[1] - pAction[1];
    const len2d = Math.hypot(dx, dy);
    if (len2d > 2) {
      const ux = dx / len2d, uy = dy / len2d;
      const headLen = Math.min(14, Math.max(8, len2d * 0.28));
      const headW = headLen * 0.52;
      const bx = pTip[0] - ux * headLen, by = pTip[1] - uy * headLen;
      for (const [width, color] of [[6, "rgba(255, 255, 255, 0.9)"], [3.2, "#e64815"]]) {
        ctx.beginPath();
        ctx.lineCap = "round";
        ctx.lineWidth = width;
        ctx.strokeStyle = color;
        ctx.moveTo(pAction[0], pAction[1]);
        ctx.lineTo(bx, by);
        ctx.stroke();
      }
      // Arrowhead with white border
      ctx.beginPath();
      ctx.moveTo(pTip[0], pTip[1]);
      ctx.lineTo(bx - uy * headW, by + ux * headW);
      ctx.lineTo(bx + uy * headW, by - ux * headW);
      ctx.closePath();
      ctx.fillStyle = "#e64815";
      ctx.lineWidth = 2;
      ctx.strokeStyle = "rgba(255, 255, 255, 0.92)";
      ctx.stroke();
      ctx.fill();
    }
    // 2. Bullseye marker at the fixed wrench origin P
    ctx.beginPath();
    ctx.arc(pAction[0], pAction[1], 5, 0, Math.PI * 2);
    ctx.fillStyle = "#ffffff";
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#e64815";
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(pAction[0], pAction[1], 2.2, 0, Math.PI * 2);
    ctx.fillStyle = "#e64815";
    ctx.fill();
    ctx.restore();
  }

  function projectPoint(mvp, x, y, z, w, h) {
    const cx = mvp[0] * x + mvp[4] * y + mvp[8] * z + mvp[12];
    const cy = mvp[1] * x + mvp[5] * y + mvp[9] * z + mvp[13];
    const cz = mvp[2] * x + mvp[6] * y + mvp[10] * z + mvp[14];
    const cw = mvp[3] * x + mvp[7] * y + mvp[11] * z + mvp[15];
    if (cw <= 0.01) return null;
    return [(cx / cw * 0.5 + 0.5) * w, (0.5 - cy / cw * 0.5) * h, cz / cw];
  }

  function drawSkeletonFallback(ctx, w, h, vp, fk) {
    ctx.fillStyle = "#f6f8fb";
    ctx.fillRect(0, 0, w, h);
    const joints2d = fk.links.slice(0, 7).map(l => projectPoint(vp, l.matrix[12], l.matrix[13], l.matrix[14], w, h)).filter(Boolean);
    if (joints2d.length >= 2) {
      ctx.strokeStyle = "#245cbe";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(joints2d[0][0], joints2d[0][1]);
      for (let i = 1; i < joints2d.length; i++) ctx.lineTo(joints2d[i][0], joints2d[i][1]);
      ctx.stroke();
    }
  }

  return {createViewer, computeFK};
})();
