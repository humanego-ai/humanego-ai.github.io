// Small linear-algebra helpers for the live simulator (row-major 4x4 / 3x3, like numpy).
export const eye4 = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

export function mul4(A, B) {
  const C = new Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    C[4 * i + j] = A[4 * i] * B[j] + A[4 * i + 1] * B[4 + j] + A[4 * i + 2] * B[8 + j] + A[4 * i + 3] * B[12 + j];
  }
  return C;
}

export function inv4(T) {   // rigid transform inverse
  const R = getR(T), p = getP(T);
  const Rt = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const q = mul3v(Rt, p);
  return fromRp(Rt, [-q[0], -q[1], -q[2]]);
}

export const getR = (T) => [T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]];
export const getP = (T) => [T[3], T[7], T[11]];
export const fromRp = (R, p) => [R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1];
export const setP = (T, p) => { const o = T.slice(); o[3] = p[0]; o[7] = p[1]; o[11] = p[2]; return o; };
export const mul3 = (A, B) => {
  const C = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
  return C;
};
export const mul3v = (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2]];
export const T3 = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
export const xform = (T, v) => [T[0] * v[0] + T[1] * v[1] + T[2] * v[2] + T[3], T[4] * v[0] + T[5] * v[1] + T[6] * v[2] + T[7], T[8] * v[0] + T[9] * v[1] + T[10] * v[2] + T[11]];
export const add = (a, b) => a.map((x, i) => x + b[i]);
export const sub = (a, b) => a.map((x, i) => x - b[i]);
export const scale = (a, s) => a.map((x) => x * s);
export const dot = (a, b) => a.reduce((s, x, i) => s + x * b[i], 0);
export const norm = (a) => Math.sqrt(dot(a, a));
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const normalize = (a) => { const n = norm(a) + 1e-12; return a.map((x) => x / n); };

// ---- rotations ----
export function rotvecToMat(r) {
  const th = norm(r);
  if (th < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const [x, y, z] = r.map((v) => v / th), c = Math.cos(th), s = Math.sin(th), C = 1 - c;
  return [c + x * x * C, x * y * C - z * s, x * z * C + y * s,
          y * x * C + z * s, c + y * y * C, y * z * C - x * s,
          z * x * C - y * s, z * y * C + x * s, c + z * z * C];
}
export function matToQuat(R) {   // -> [x, y, z, w]
  const tr = R[0] + R[4] + R[8];
  let x, y, z, w;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (R[7] - R[5]) / s; y = (R[2] - R[6]) / s; z = (R[3] - R[1]) / s; }
  else if (R[0] > R[4] && R[0] > R[8]) { const s = Math.sqrt(1 + R[0] - R[4] - R[8]) * 2; w = (R[7] - R[5]) / s; x = 0.25 * s; y = (R[1] + R[3]) / s; z = (R[2] + R[6]) / s; }
  else if (R[4] > R[8]) { const s = Math.sqrt(1 + R[4] - R[0] - R[8]) * 2; w = (R[2] - R[6]) / s; x = (R[1] + R[3]) / s; y = 0.25 * s; z = (R[5] + R[7]) / s; }
  else { const s = Math.sqrt(1 + R[8] - R[0] - R[4]) * 2; w = (R[3] - R[1]) / s; x = (R[2] + R[6]) / s; y = (R[5] + R[7]) / s; z = 0.25 * s; }
  const n = Math.hypot(x, y, z, w);
  return [x / n, y / n, z / n, w / n];
}
export function quatToMat([x, y, z, w]) {
  return [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
          2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
          2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
}
export function matToRotvec(R) {
  const q = matToQuat(R);
  let [x, y, z, w] = q;
  if (w < 0) { x = -x; y = -y; z = -z; w = -w; }
  const s = Math.hypot(x, y, z);
  if (s < 1e-12) return [0, 0, 0];
  const th = 2 * Math.atan2(s, w);
  return [x / s * th, y / s * th, z / s * th];
}
export function quatSlerp(a, b, t) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let bb = b;
  if (d < 0) { d = -d; bb = b.map((v) => -v); }
  if (d > 0.9995) { const r = a.map((v, i) => v + t * (bb[i] - v)); const n = Math.hypot(...r); return r.map((v) => v / n); }
  const th0 = Math.acos(d), th = th0 * t, s0 = Math.cos(th) - d * Math.sin(th) / Math.sin(th0), s1 = Math.sin(th) / Math.sin(th0);
  return a.map((v, i) => s0 * v + s1 * bb[i]);
}
export const T_to_p = (T) => [...getP(T), ...matToRotvec(getR(T))];
export const p_to_T = (p) => fromRp(rotvecToMat(p.slice(3, 6)), p.slice(0, 3));
export const rotAngle = (Ra, Rb) => norm(matToRotvec(mul3(Rb, T3(Ra))));   // |R_b R_a^T|
export function slerpMat(Ra, Rb, t) { return quatToMat(quatSlerp(matToQuat(Ra), matToQuat(Rb), t)); }

// ---- small dense solvers ----
export function solve(A, b, n) {          // Gaussian elimination with partial pivoting, A row-major n x n
  const M = A.slice(), x = b.slice();
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r * n + c]) > Math.abs(M[p * n + c])) p = r;
    if (p !== c) { for (let k = 0; k < n; k++) [M[c * n + k], M[p * n + k]] = [M[p * n + k], M[c * n + k]]; [x[c], x[p]] = [x[p], x[c]]; }
    const d = M[c * n + c] || 1e-12;
    for (let r = c + 1; r < n; r++) {
      const f = M[r * n + c] / d;
      if (!f) continue;
      for (let k = c; k < n; k++) M[r * n + k] -= f * M[c * n + k];
      x[r] -= f * x[c];
    }
  }
  for (let r = n - 1; r >= 0; r--) { let s = x[r]; for (let k = r + 1; k < n; k++) s -= M[r * n + k] * x[k]; x[r] = s / (M[r * n + r] || 1e-12); }
  return x;
}

export function eigSym3(C) {               // Jacobi; returns {vals (desc), vecs: columns}
  const a = C.slice(); const v = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  for (let sweep = 0; sweep < 50; sweep++) {
    const off = Math.abs(a[1]) + Math.abs(a[2]) + Math.abs(a[5]);
    if (off < 1e-15) break;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      const apq = a[3 * p + q]; if (Math.abs(apq) < 1e-18) continue;
      const theta = (a[3 * q + q] - a[3 * p + p]) / (2 * apq);
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < 3; k++) { const akp = a[3 * k + p], akq = a[3 * k + q]; a[3 * k + p] = c * akp - s * akq; a[3 * k + q] = s * akp + c * akq; }
      for (let k = 0; k < 3; k++) { const apk = a[3 * p + k], aqk = a[3 * q + k]; a[3 * p + k] = c * apk - s * aqk; a[3 * q + k] = s * apk + c * aqk; }
      for (let k = 0; k < 3; k++) { const vkp = v[3 * k + p], vkq = v[3 * k + q]; v[3 * k + p] = c * vkp - s * vkq; v[3 * k + q] = s * vkp + c * vkq; }
    }
  }
  const vals = [a[0], a[4], a[8]];
  const order = [0, 1, 2].sort((i, j) => vals[j] - vals[i]);
  return { vals: order.map((i) => vals[i]), vecs: order.map((i) => [v[i], v[3 + i], v[6 + i]]) };
}

export function median(arr) {
  const a = Float64Array.from(arr).sort(); const n = a.length;
  if (!n) return NaN;
  return n % 2 ? a[(n - 1) >> 1] : 0.5 * (a[n / 2 - 1] + a[n / 2]);
}

// deterministic Gaussian noise
export function makeRng(seed = 1) {
  let s = seed >>> 0 || 1;
  const u = () => { s ^= s << 13; s >>>= 0; s ^= s >> 17; s ^= s << 5; s >>>= 0; return (s + 0.5) / 4294967296; };
  return { uniform: u, normal: () => Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u()) };
}
