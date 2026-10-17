// Live perception: the released HumanEgo pipeline (KptsSelector -> depth lift -> PCA pose -> VisualKpts),
// fed by renders of the three.js scene from the calibrated policy camera.
import * as THREE from 'three';
import * as M from './math.js';

const W = 640, H = 480;
// cfg/preprocess/base/VisualKpts.yaml (BGR in the yaml -> RGB here)
const VK = {
  wMax: 0.18, wMin: 0.05, finger: 0.08, root: 0.08, ppl: 6, r: 4, core: 1,
  open: [[255, 255, 0], [255, 160, 0], [255, 60, 0], [255, 0, 0]],
  closed: [[255, 255, 255], [255, 255, 255], [255, 0, 255], [255, 0, 255]],
  obj: [[0, 255, 255], [0, 255, 0], [255, 0, 255], [255, 0, 0]],
};
// cfg/preprocess/base/KptsSelector.yaml + cfg/inference/.../CamRS.yaml
const KS = { close: 7, erode: 5, erodeIters: 1, inner: 5, minEdge: 50, nPts: 20 };
const LIFT = { r: 6, zmin: 0.15, zmax: 2.5, madK: 3.5, absGate: 0.01, minValid: 5 };

// ------------------------------------------------------------------ binary morphology (OpenCV MORPH_ELLIPSE)
function ellipseKernel(k) {
  const r = k >> 1, c = r, ker = [];
  for (let i = 0; i < k; i++) {
    const dy = i - r;
    if (Math.abs(dy) <= r) {
      const dx = Math.round(c * Math.sqrt((r * r - dy * dy) / (r * r || 1)));
      for (let j = Math.max(c - dx, 0); j < Math.min(c + dx + 1, k); j++) ker.push([dy, j - c]);
    }
  }
  return ker;
}
function morph(m, k, op) {           // op: 'erode' | 'dilate'
  const ker = ellipseKernel(k), out = new Uint8Array(W * H);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    let v = op === 'erode' ? 1 : 0;
    for (const [dy, dx] of ker) {
      const yy = y + dy, xx = x + dx;
      const s = (yy < 0 || yy >= H || xx < 0 || xx >= W) ? (op === 'erode' ? 1 : 0) : m[yy * W + xx];
      if (op === 'erode' && !s) { v = 0; break; }
      if (op === 'dilate' && s) { v = 1; break; }
    }
    out[y * W + x] = v;
  }
  return out;
}
const count = (m) => { let n = 0; for (let i = 0; i < m.length; i++) n += m[i]; return n; };

// largest 8-connected component, then its outer boundary (Moore neighbour tracing)
function outerContour(m) {
  const lab = new Int32Array(W * H), sizes = [0];
  let best = 0;
  const st = [];
  for (let i = 0; i < W * H; i++) {
    if (!m[i] || lab[i]) continue;
    const id = sizes.length; sizes.push(0); lab[i] = id; st.push(i);
    while (st.length) {
      const p = st.pop(); sizes[id]++;
      const py = (p / W) | 0, px = p - py * W;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const y = py + dy, x = px + dx;
        if (y < 0 || y >= H || x < 0 || x >= W) continue;
        const q = y * W + x;
        if (m[q] && !lab[q]) { lab[q] = id; st.push(q); }
      }
    }
    if (sizes[id] > sizes[best]) best = id;
  }
  if (!best) return [];
  let start = -1;
  for (let i = 0; i < W * H; i++) if (lab[i] === best) { start = i; break; }   // top-most, then left-most
  const inside = (x, y) => x >= 0 && y >= 0 && x < W && y < H && lab[y * W + x] === best;
  const dirs = [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]];
  const sx = start % W, sy = (start / W) | 0;
  const c = [[sx, sy]];
  let x = sx, y = sy, d = 6;      // came from the north (pixel above is outside)
  for (let it = 0; it < 8 * W * H; it++) {
    let found = false;
    for (let k = 0; k < 8; k++) {
      const nd = (d + 6 + k) % 8, nx = x + dirs[nd][0], ny = y + dirs[nd][1];
      if (inside(nx, ny)) { x = nx; y = ny; d = nd; found = true; break; }
    }
    if (!found || (x === sx && y === sy)) break;
    c.push([x, y]);
  }
  return c;
}

export function selectPoints(mask) {            // preprocess.KptsSelector.select_points
  let m = mask;
  m = morph(morph(m, KS.close, 'dilate'), KS.close, 'erode');
  for (let i = 0; i < KS.erodeIters; i++) m = morph(m, KS.erode, 'erode');
  const me = morph(m, KS.inner, 'erode');
  if (count(me) > KS.minEdge) m = me;
  let c = outerContour(m);
  if (c.length < 10) return [];
  let top = 0;
  c.forEach((p, i) => { if (p[1] < c[top][1] || (p[1] === c[top][1] && p[0] < c[top][0])) top = i; });
  c = [...c.slice(top), ...c.slice(0, top)];
  const cum = [0];
  for (let i = 1; i < c.length; i++) cum.push(cum[i - 1] + Math.hypot(c[i][0] - c[i - 1][0], c[i][1] - c[i - 1][1]));
  const total = cum[cum.length - 1] + Math.hypot(c[0][0] - c[c.length - 1][0], c[0][1] - c[c.length - 1][1]);
  const out = [];
  for (let k = 0; k < KS.nPts; k++) {
    const td = total * k / KS.nPts;
    let i = 0; while (i + 1 < cum.length && cum[i + 1] <= td) i++;
    const p1 = c[i], p2 = i === c.length - 1 ? c[0] : c[i + 1], d1 = cum[i], d2 = i === c.length - 1 ? total : cum[i + 1];
    const a = d2 === d1 ? 0 : (td - d1) / (d2 - d1);
    out.push([Math.round(p1[0] + a * (p2[0] - p1[0])), Math.round(p1[1] + a * (p2[1] - p1[1]))]);
  }
  return out;
}

function lift1(uv, depth, mask, K) {              // CamRS.lift3d_for_single_frame
  const [u, v] = uv, ui = Math.round(u), vi = Math.round(v);
  if (ui < 0 || ui >= W || vi < 0 || vi >= H) return null;
  const roi = [];
  for (let y = Math.max(0, vi - LIFT.r); y <= Math.min(H - 1, vi + LIFT.r); y++)
    for (let x = Math.max(0, ui - LIFT.r); x <= Math.min(W - 1, ui + LIFT.r); x++) {
      if (!mask[y * W + x]) continue;
      const z = depth[y * W + x];
      if (Number.isFinite(z) && z > LIFT.zmin && z < LIFT.zmax) roi.push(z);
    }
  if (roi.length < LIFT.minValid) return null;
  const zm = M.median(roi), mad = M.median(roi.map((z) => Math.abs(z - zm)));
  const keep = roi.filter((z) => Math.abs(z - zm) <= LIFT.madK * mad + LIFT.absGate);
  if (keep.length < LIFT.minValid) return null;
  const z = M.median(keep);
  return [(u - K[2]) / K[0] * z, (v - K[5]) / K[4] * z, z];
}
function liftMulti(uv, depths, mask, K) {
  const xs = depths.map((d) => lift1(uv, d, mask, K)).filter(Boolean);
  if (!xs.length) return null;
  return [0, 1, 2].map((i) => M.median(xs.map((p) => p[i])));
}
function postprocess(pts) {                      // MAD gate + DBSCAN (eps 6 cm, min 3)
  pts = pts.filter(Boolean);
  if (pts.length < 3) return pts;
  const med = [0, 1, 2].map((i) => M.median(pts.map((p) => p[i])));
  const d = pts.map((p) => M.norm(M.sub(p, med))), md = M.median(d), mad = M.median(d.map((x) => Math.abs(x - md)));
  let P = pts.filter((_, i) => d[i] <= md + 4 * mad + 1e-6);
  if (P.length < 3) P = pts;
  const lab = new Array(P.length).fill(-2); let cid = -1;
  const nb = (i) => P.map((q, j) => (M.norm(M.sub(P[i], q)) <= 0.06 ? j : -1)).filter((j) => j >= 0);
  for (let i = 0; i < P.length; i++) {
    if (lab[i] !== -2) continue;
    const n = nb(i); if (n.length < 3) { lab[i] = -1; continue; }
    lab[i] = ++cid; const q = [...n];
    while (q.length) { const j = q.pop(); if (lab[j] === -1) lab[j] = cid; if (lab[j] !== -2) continue; lab[j] = cid; const nj = nb(j); if (nj.length >= 3) q.push(...nj); }
  }
  if (cid < 0) return P;
  const cnt = new Array(cid + 1).fill(0); lab.forEach((l) => { if (l >= 0) cnt[l]++; });
  const bestL = cnt.indexOf(Math.max(...cnt));
  return P.filter((_, i) => lab[i] === bestL);
}

export function pca1(pts, isAnchor, anchorCenter) {      // preprocess.OrientAnything.estimate_frame_pca1
  const n = pts.length, t = [0, 1, 2].map((i) => pts.reduce((s, p) => s + p[i], 0) / n);
  const C = new Array(9).fill(0);
  for (const p of pts) { const q = M.sub(p, t); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] += q[i] * q[j]; }
  const { vals, vecs } = M.eigSym3(C);
  const camUp = [0, -1, 0], camRight = [1, 0, 0];
  let y, x;
  if (isAnchor) {
    y = vecs[1].slice(); if (M.dot(y, camUp) > 0) y = M.scale(y, -1);
    const aniso = (vals[0] - vals[1]) / (vals[0] + 1e-12);
    if (aniso > 0.15) { x = vecs[0].slice(); if (M.dot(x, camRight) < 0) x = M.scale(x, -1); }
    else x = M.normalize(M.sub(camRight, M.scale(y, M.dot(camRight, y))));
  } else {
    y = vecs[2].slice(); if (M.dot(y, camUp) > 0) y = M.scale(y, -1);
    const v = M.sub(anchorCenter, t), xp = M.sub(v, M.scale(y, M.dot(v, y)));
    x = M.norm(xp) > 1e-4 ? M.normalize(xp) : M.normalize(M.sub(camRight, M.scale(y, M.dot(camRight, y))));
  }
  x = M.normalize(M.sub(x, M.scale(y, M.dot(x, y))));
  const z = M.normalize(M.cross(x, y));
  return M.fromRp([x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]], t);
}

const S = [-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
export const mirrorPose = (T) => M.mul4(M.mul4(S, T), S);

// ------------------------------------------------------------------ renderer from the policy camera
export class PolicyCamera {
  constructor(viewer, policyCam) {
    this.viewer = viewer;
    this.K = policyCam.K;
    this.r = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.r.setSize(W, H, false); this.r.setPixelRatio(1);
    this.r.outputColorSpace = THREE.SRGBColorSpace;
    this.r.toneMapping = viewer.renderer.toneMapping; this.r.toneMappingExposure = viewer.renderer.toneMappingExposure;
    this.r.shadowMap.enabled = false;            // the policy never sees arm shadows (robot removed); cheaper
    const fovy = 2 * Math.atan(H / 2 / this.K[4]) * 180 / Math.PI;
    this.cam = new THREE.PerspectiveCamera(fovy, W / H, 0.02, 10);
    const T = policyCam.T_cam_in_world;
    const R = [T[0], -T[1], -T[2], T[4], -T[5], -T[6], T[8], -T[9], -T[10]];   // OpenCV -> three (flip y, z)
    const m = new THREE.Matrix4().set(R[0], R[1], R[2], T[3], R[3], R[4], R[5], T[7], R[6], R[7], R[8], T[11], 0, 0, 0, 1);
    this.cam.matrixAutoUpdate = false; this.cam.matrix.copy(m); this.cam.matrixWorld.copy(m);
    this.cam.matrixWorldInverse.copy(m).invert();
    this.rtF = new THREE.WebGLRenderTarget(W, H, { type: THREE.FloatType, format: THREE.RGBAFormat });
    this.rtU = new THREE.WebGLRenderTarget(W, H);
    this.depthMat = new THREE.ShaderMaterial({
      vertexShader: 'varying float vz; void main(){ vec4 mv = modelViewMatrix * vec4(position,1.0); vz = -mv.z; gl_Position = projectionMatrix * mv; }',
      fragmentShader: 'varying float vz; void main(){ gl_FragColor = vec4(vz, 0.0, 0.0, 1.0); }', side: THREE.DoubleSide });
    this.black = new THREE.MeshBasicMaterial({ color: 0x000000, side: THREE.DoubleSide });
    this.idMats = {};
    this.canvas = document.createElement('canvas'); this.canvas.width = W; this.canvas.height = H;
    this.ctx = this.canvas.getContext('2d', { willReadFrequently: true });
    this.small = document.createElement('canvas'); this.small.width = 320; this.small.height = 240;
    this.sctx = this.small.getContext('2d', { willReadFrequently: true });
  }

  _robotGroups() { return this.viewer.bodies.filter((b) => /^(right|left)\//.test(b.name)).map((b) => b.grp); }
  _withVisibility(fn, hideRobot) {
    const v = this.viewer, hidden = [];
    const hide = (o) => { if (o && o.visible) { o.visible = false; hidden.push(o); } };
    hide(v.overlayRoot);
    for (const b of Object.values(v.objectBodies)) for (const c of b.grp.children) if (c.userData.kpts) hide(c);
    if (hideRobot) for (const g of this._robotGroups()) hide(g);
    try { return fn(); } finally { for (const o of hidden) o.visible = true; }
  }
  _withMaterials(map, fn) {
    const saved = [];
    this.viewer.root.traverse((o) => { if (o.isMesh) { saved.push([o, o.material]); o.material = map(o); } });
    try { return fn(); } finally { for (const [o, m] of saved) o.material = m; }
  }
  _readFloat(rt) {
    const buf = new Float32Array(W * H * 4); this.r.readRenderTargetPixels(rt, 0, 0, W, H, buf);
    const out = new Float32Array(W * H);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) out[y * W + x] = buf[((H - 1 - y) * W + x) * 4];
    return out;
  }

  /** Clean image (robot removed = perfect LaMa inpainting) as an ImageData in the real camera domain. */
  cleanImage() {
    const scene = this.viewer.scene, bg = this.viewer.renderer.domElement.style.background;
    this._withVisibility(() => { this.r.setRenderTarget(null); this.r.setClearColor(0xbdbcb8, 1); this.r.render(scene, this.cam); }, true);
    this.ctx.drawImage(this.r.domElement, 0, 0);
    return this.ctx.getImageData(0, 0, W, H);
  }

  /** Object masks (robot visible and occluding, like DINO-SAM on the real image). */
  masks(objectNames) {
    const ids = {};
    objectNames.forEach((n, i) => { ids[n] = i + 1; });
    const scene = this.viewer.scene;
    const bodyOf = (o) => { let p = o; while (p && !(p.name in ids)) p = p.parent; return p ? p.name : null; };
    const img = this._withVisibility(() => this._withMaterials((o) => {
      const b = bodyOf(o);
      if (!b) return this.black;
      const k = 'id' + ids[b];
      return (this.idMats[k] ||= new THREE.MeshBasicMaterial({ color: new THREE.Color(ids[b] / 10, 0, 0), side: THREE.DoubleSide }));
    }, () => { this.r.setRenderTarget(this.rtF); this.r.setClearColor(0x000000, 1); this.r.clear(); this.r.render(scene, this.cam); this.r.setRenderTarget(null); return this._readFloat(this.rtF); }), false);
    const out = {};
    for (const [n, id] of Object.entries(ids)) {
      const m = new Uint8Array(W * H);
      for (let i = 0; i < W * H; i++) if (Math.abs(img[i] * 10 - id) < 0.3) m[i] = 1;
      out[n] = m;
    }
    return out;
  }

  /** Metric depth along the optical axis (+ the camera's noise model). */
  depth(rng, noise) {
    const scene = this.viewer.scene;
    const z = this._withVisibility(() => this._withMaterials(() => this.depthMat, () => {
      this.r.setRenderTarget(this.rtF); this.r.setClearColor(0x000000, 1); this.r.clear(); this.r.render(scene, this.cam); this.r.setRenderTarget(null);
      return this._readFloat(this.rtF);
    }), false);
    if (noise) for (let i = 0; i < z.length; i++) { const s = noise[0] + noise[1] * z[i] * z[i]; z[i] += rng.normal() * s; if (z[i] < noise[2]) z[i] = 0; }
    return z;
  }

  /** Inference.do_object_centric_bimanual (shared_context): per-side object poses + local keypoints. */
  estimateObjects(sides, objectNames, rng, depthNoise, nFrames = 10) {
    const masks = this.masks(objectNames);
    const obj1 = new Uint8Array(W * H);
    for (const n of objectNames) if (n.startsWith('bread')) for (let i = 0; i < W * H; i++) obj1[i] |= masks[n][i];
    const full = { obj1, obj2: masks.plate };
    const depths = Array.from({ length: nFrames }, () => this.depth(rng, depthNoise));
    const out = {};
    for (const side of sides) {
      const flip = side === 'left', ok3d = {};
      for (const k of ['obj1', 'obj2']) {
        let mk = full[k];
        if (sides.length > 1 && k === 'obj1') mk = splitByX(mk, W >> 1)[side];
        const k2d = selectPoints(mk);
        let k3d = postprocess(k2d.map((uv) => liftMulti(uv, depths, mk, this.K)));
        if (flip) k3d = k3d.map((p) => [-p[0], p[1], p[2]]);
        ok3d[k] = k3d;
      }
      const meta = {};
      const Ta = ok3d.obj1.length >= 3 ? pca1(ok3d.obj1, true) : M.eye4();
      const To = ok3d.obj2.length >= 3 ? pca1(ok3d.obj2, false, M.getP(Ta)) : M.eye4();
      for (const [k, T] of [['obj1', Ta], ['obj2', To]]) {
        const Ti = M.inv4(T);
        meta[k] = { T_static: T, kpts_local: ok3d[k].map((p) => M.xform(Ti, p)) };
      }
      out[side] = { meta, dynamic: { obj1: Ta.slice(), obj2: To.slice() }, T_anchor_static: Ta };
    }
    return out;
  }

  /** Policy input for one side: clean background + object keypoints + virtual gripper (VisualKpts). */
  sideImage(clean, side, objs, eePInCam, isGrasp) {
    const flip = side === 'left', ctx = this.ctx, K = this.K;
    const Kf = [K[0], 0, W - K[2], 0, K[4], K[5], 0, 0, 1];
    const aK = flip ? Kf : K;
    // background in this side's domain
    ctx.putImageData(clean, 0, 0);
    if (flip) this._flip();
    let ci = 0;
    for (const k of Object.keys(objs.meta)) {
      const kl = objs.meta[k].kpts_local; if (!kl.length) continue;
      const Td = objs.dynamic[k] || objs.meta[k].T_static, col = VK.obj[ci++ % VK.obj.length];
      for (const p of kl) {
        const q = M.xform(Td, p), z = q[2] + 1e-6, u = aK[0] * q[0] / z + aK[2], v = aK[4] * q[1] / z + aK[5];
        if (u >= 0 && u < W && v >= 0 && v < H && !(u === 0 && v === 0)) this._kpt(Math.trunc(u), Math.trunc(v), col, VK.r + 1);
      }
    }
    if (flip) this._flip();
    this._gripper(eePInCam, isGrasp);
    if (flip) this._flip();
    this.sctx.drawImage(this.canvas, 0, 0, 320, 240);
    const d = this.sctx.getImageData(0, 0, 320, 240).data, chw = new Float32Array(3 * 320 * 240), n = 320 * 240;
    for (let i = 0; i < n; i++) { chw[i] = d[4 * i] / 255; chw[n + i] = d[4 * i + 1] / 255; chw[2 * n + i] = d[4 * i + 2] / 255; }
    return { chw, preview: this.canvas };
  }
  _flip() { const c = this.ctx; const tmp = c.getImageData(0, 0, W, H); this.small2 ||= Object.assign(document.createElement('canvas'), { width: W, height: H });
    const t = this.small2.getContext('2d'); t.putImageData(tmp, 0, 0); c.save(); c.setTransform(-1, 0, 0, 1, W, 0); c.drawImage(this.small2, 0, 0); c.restore(); }
  _kpt(u, v, col, r) {
    const c = this.ctx;
    c.fillStyle = `rgb(${col[0]},${col[1]},${col[2]})`; c.beginPath(); c.arc(u, v, r, 0, 2 * Math.PI); c.fill();
    c.fillStyle = '#fff'; c.beginPath(); c.arc(u, v, VK.core, 0, 2 * Math.PI); c.fill();
  }
  _gripper(p, closed) {
    const T = M.p_to_T(p), hw = (closed ? VK.wMin : VK.wMax) / 2, fl = VK.finger, rd = VK.root, K = this.K;
    const P = { lt: [-hw, 0, 0], rt: [hw, 0, 0], lb: [-hw, 0, -fl], rb: [hw, 0, -fl], root: [0, 0, -(fl + rd)] };
    const C = closed ? VK.closed : VK.open, [c1, c2, c3, c4] = C;
    const segs = [[P.root, P.lb, c4, c3], [P.root, P.rb, c4, c3], [P.lb, P.lt, c2, c1], [P.rb, P.rt, c2, c1]];
    for (const [a, b, s, e] of segs) for (let i = 0; i < VK.ppl; i++) {
      const f = i / (VK.ppl - 1), q = M.xform(T, a.map((x, j) => x + f * (b[j] - x)));
      if (q[2] <= 0.001) continue;
      const u = K[0] * q[0] / q[2] + K[2], v = K[4] * q[1] / q[2] + K[5];
      if (u >= 0 && u < W && v >= 0 && v < H) this._kpt(u, v, s.map((x, j) => Math.trunc(x + (e[j] - x) * f)), VK.r);
    }
  }
}

function splitByX(m, sx) {                        // Inference.split_mask_by_x
  const lab = new Int32Array(W * H), comps = []; let id = 0;
  for (let i = 0; i < W * H; i++) {
    if (!m[i] || lab[i]) continue;
    id++; let n = 0, cx = 0; const st = [i]; lab[i] = id;
    while (st.length) { const p = st.pop(); n++; cx += p % W; const py = (p / W) | 0, px = p % W;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) { const y = py + dy, x = px + dx; if (y < 0 || y >= H || x < 0 || x >= W) continue; const q = y * W + x; if (m[q] && !lab[q]) { lab[q] = id; st.push(q); } } }
    if (n >= 50) comps.push([id, cx / n]);
  }
  const right = new Uint8Array(W * H), left = new Uint8Array(W * H);
  if (comps.length === 1) { for (let i = 0; i < W * H; i++) if (lab[i] === comps[0][0]) ((i % W) >= sx ? right : left)[i] = 1; }
  else for (const [cid, cx] of comps) { const t = cx >= sx ? right : left; for (let i = 0; i < W * H; i++) if (lab[i] === cid) t[i] = 1; }
  return { right, left };
}
