// Released HumanEgo policy in the browser: ONNX Runtime Web (WebGPU, or WASM int8 fallback).
// ICT construction / decoding mirror inference/policy.py (ICTPolicy) for single-hand camera-frame checkpoints.
import * as M from './math.js';

export const T_ALIGN = [-1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1];
export const T_ALIGN_INV = M.inv4(T_ALIGN);
const TYPE_HAND_R = 2, TYPE_OBJ_ANCHOR = 3, TYPE_OBJ_OTHER = 4;

export class LivePolicy {
  static async create(base, onStatus) {
    const p = new LivePolicy();
    p.meta = await (await fetch(base + 'serve_bread_meta.json')).json();
    // The whole flow-ODE solve is one ONNX graph (context + cross-attention K/V computed once,
    // Euler steps unrolled), exported by scripts/export_onnx.py and checked against ICTPolicy.infer.
    //   WebGPU: fp32, 20 Euler steps (as on the real robot)   ~0.1 s / re-plan
    //   CPU   : fp16 weights, 10 Euler steps (README: 10-20)   ~0.4 s / re-plan
    const gpu = !!navigator.gpu && !new URLSearchParams(location.search).has('cpu');
    const pick = (dev) => dev === 'webgpu' ? { file: 'serve_bread_rollout20.onnx', mb: 62, steps: 20 }
                                           : { file: 'serve_bread_rollout10_fp16.onnx', mb: 32, steps: 10 };
    const load = async (dev) => {
      const c = pick(dev);
      onStatus?.(`Downloading the released policy (${c.mb} MB)…`);
      const buf = await fetchWithProgress(base + c.file, (f) => onStatus?.(`Downloading the released policy… ${Math.round(100 * f)}% of ${c.mb} MB`));
      onStatus?.(`Compiling the policy for ${dev === 'webgpu' ? 'WebGPU' : 'CPU'}…`);
      p.session = await ort.InferenceSession.create(buf, { executionProviders: [dev], graphOptimizationLevel: 'all' });
      p.device = dev; p.steps = c.steps;
    };
    ort.env.wasm.numThreads = crossOriginIsolated ? Math.min(4, navigator.hardwareConcurrency || 1) : 1;
    try { await load(gpu ? 'webgpu' : 'wasm'); }
    catch (e) { if (!gpu) throw e; console.warn('WebGPU failed, using CPU', e); await load('wasm'); }
    p.K = p.meta.pred_horizon; p.D = p.meta.action_dim;
    p.mean = p.meta.pos_mean; p.std = p.meta.pos_std;
    return p;
  }

  encode(T) {   // 4x4 -> normalized [pos(3), o6d(6)]
    const pos = M.getP(T).map((v, i) => (v - this.mean[i]) / (this.std[i] + 1e-8));
    const R = M.getR(T);
    let a1 = [R[0], R[3], R[6]], a2 = [R[1], R[4], R[7]];
    const b1 = M.normalize(a1); a2 = M.sub(a2, M.scale(b1, M.dot(b1, a2))); const b2 = M.normalize(a2);
    return [...pos, b1[0], b2[0], b1[1], b2[1], b1[2], b2[2]];    // R[:, :2].reshape(-1) row-major
  }

  buildICT(Thand, grasp, objs, anchorKey = 'obj1') {
    // camera_frame: the reference frame is the camera itself
    const rel = (Tent) => this.encode(M.mul4(M.inv4(Tent), Thand));
    const tokens = [[TYPE_HAND_R, ...this.encode(Thand), ...rel(Thand), grasp]];
    const keys = [anchorKey, ...Object.keys(objs).filter((k) => k !== anchorKey)];
    for (const k of keys) {
      if (!objs[k]) continue;
      tokens.push([k === anchorKey ? TYPE_OBJ_ANCHOR : TYPE_OBJ_OTHER, ...this.encode(objs[k]), ...rel(objs[k]), -1]);
    }
    const x = new Float32Array(this.meta.max_ict * this.meta.ict_dim), mask = new Float32Array(this.meta.max_ict);
    tokens.slice(0, this.meta.max_ict).forEach((tok, i) => { x.set(tok, i * this.meta.ict_dim); mask[i] = 1; });
    return { x, mask };
  }

  anchorUV(Tanchor, K, W, H) {
    const p = M.getP(Tanchor);
    if (p[2] < 1e-4) return [0.5, 0.5];
    return [Math.min(Math.max((K[0] * p[0] / p[2] + K[2]) / W, 0), 1), Math.min(Math.max((K[4] * p[1] / p[2] + K[5]) / H, 0), 1)];
  }

  async infer(rgbCHW, ict, uv) {
    const T = (data, shape) => new ort.Tensor('float32', data, shape);
    const out = await this.session.run({
      x_rgb: T(rgbCHW, [1, 3, ...this.meta.img_size]), x_ict: T(ict.x, [1, this.meta.max_ict, this.meta.ict_dim]),
      ict_mask: T(ict.mask, [1, this.meta.max_ict]), anchor_uv: T(Float32Array.from(uv), [1, 2]),
      noise: T(Float32Array.from(this.meta.noise), [1, this.K, this.D]) });
    const x = out.actions.data, done = out.done_logit.data[0];
    const sig = (z) => 1 / (1 + Math.exp(-z));
    const pos = [], o6d = [], g = [];
    for (let k = 0; k < this.K; k++) {
      const a = x.subarray(k * this.D, (k + 1) * this.D);
      pos.push([a[0], a[1], a[2]]); o6d.push([a[3], a[4], a[5], a[6], a[7], a[8]]); g.push(sig(a[9]));
    }
    return { pos, o6d, grasp: g, done: sig(done) };
  }

  // one predicted step -> EE target in the camera frame (hand pose @ inv(T_align))
  decode(pos, o6d) {
    const p = pos.map((v, i) => v * this.std[i] + this.mean[i]);
    const a1 = [o6d[0], o6d[2], o6d[4]], a2r = [o6d[1], o6d[3], o6d[5]];
    const b1 = M.normalize(a1), b2 = M.normalize(M.sub(a2r, M.scale(b1, M.dot(b1, a2r)))), b3 = M.cross(b1, b2);
    const R = [b1[0], b2[0], b3[0], b1[1], b2[1], b3[1], b1[2], b2[2], b3[2]];
    return M.mul4(M.fromRp(R, p), T_ALIGN_INV);
  }
}

async function fetchWithProgress(url, onProgress) {
  const res = await fetch(url);
  const total = +res.headers.get('content-length') || 0;
  if (!res.body || !total) return new Uint8Array(await res.arrayBuffer());
  const reader = res.body.getReader(), chunks = []; let got = 0;
  for (;;) { const { done, value } = await reader.read(); if (done) break; chunks.push(value); got += value.length; onProgress?.(got / total); }
  const out = new Uint8Array(got); let o = 0; for (const c of chunks) { out.set(c, o); o += c.length; }
  return out;
}
