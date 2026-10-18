// HumanEgo Sim — rollout loading. Packed rollouts (`<id>.bin`, humanego_sim/web_pack.py) hold every
// number of the exported JSON as an integer at the precision it was rounded to, delta-coded over
// time; decoding divides by the same power of ten, which gives back exactly the doubles JSON.parse
// would produce. Unpacked builds still load `<id>.json`.

const DT = { 1: Int8Array, 2: Int16Array, 4: Int32Array };

export async function fetchRollout(id, fmt) {
  if (fmt === 'bin') {
    const res = await fetch(`data/rollouts/${id}.bin`);
    if (!res.ok) throw new Error(`rollout ${id}: HTTP ${res.status}`);
    return decodeRollout(await res.arrayBuffer());
  }
  return (await fetch(`data/rollouts/${id}.json`)).json();
}

export function decodeRollout(buf) {
  const dv = new DataView(buf);
  const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
  if (magic !== 'HEGR') throw new Error('not a packed rollout');
  const ver = dv.getUint32(4, true), jl = dv.getUint32(8, true);
  if (ver !== 1) throw new Error(`packed rollout version ${ver}`);
  const ro = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 12, jl)));
  const P0 = 12 + jl;
  const view = (T, off, n) => new T(buf, P0 + off, n);
  const pack = ro.pack || {};
  delete ro.pack;

  if (pack.frames) {
    const f = pack.frames, T = f.n, C = f.cols.length;
    const blk = { 1: view(Int8Array, ...f.i8), 2: view(Int16Array, ...f.i16), 4: view(Int32Array, ...f.i32) };
    const off = { 1: 0, 2: 0, 4: 0 };
    const rows = Array.from({ length: T }, () => new Float64Array(C));
    f.cols.forEach((c, j) => {
      const src = blk[c.w], s = 10 ** c.d;
      let o = off[c.w], v = c.base;
      rows[0][j] = v / s;
      for (let i = 1; i < T; i++) { v += src[o++]; rows[i][j] = v / s; }
      off[c.w] = o;
    });
    ro.frames = rows;
  }

  if (pack.deform) {
    ro.deform = {};
    for (const [name, d] of Object.entries(pack.deform)) {
      const t = view(Int32Array, d.t, d.n), mask = view(Uint8Array, d.mask, d.n);
      const D = view(DT[d.w], d.data, d.nd * d.len), s = 10 ** d.dt;
      const acc = new Int32Array(d.len), r = [];
      let k = 0;
      for (let i = 0; i < d.n; i++) {
        if (!mask[i]) { r.push(0); continue; }
        const row = new Int32Array(d.len);
        for (let c = 0; c < d.len; c++) { acc[c] += D[k * d.len + c]; row[c] = acc[c]; }
        r.push(row); k++;
      }
      ro.deform[name] = { t: Array.from(t, (x) => x / s), r };
    }
  }

  if (pack.policy) {
    const p = pack.policy, n = p.n, ts = view(Int32Array, p.t, n), s = 10 ** p.dt;
    const pol = Array.from(ts, (x) => ({ t: x / s, sides: {} }));
    for (const [side, d] of Object.entries(p.sides)) {
      const m = d.m, present = view(Uint8Array, d.present, n), tl = view(Uint8Array, d.traj_len, m);
      const dq = view(DT[d.traj_w], d.traj, 3 * d.npts), step = view(Uint8Array, d.traj_step, m);
      const gl = view(Uint8Array, d.grasp_len, m), g = view(Uint8Array, d.grasp, d.ng + m);
      const done = view(Int32Array, d.done, m), gr = view(Uint8Array, d.grasping, m);
      const sT = 10 ** d.traj_dec, sG = 10 ** d.grasp_dec, sD = 10 ** d.done_dec;
      let k = 0, op = 0, og = 0;
      for (let i = 0; i < n; i++) {
        if (!present[i]) continue;
        const traj = [], L = tl[k];
        let x = 0, y = 0, z = 0;
        for (let j = 0; j < L; j++, op++) {
          if (j === 0) { x = dq[3 * op]; y = dq[3 * op + 1]; z = dq[3 * op + 2]; }
          else { x += dq[3 * op]; y += dq[3 * op + 1]; z += dq[3 * op + 2]; }
          traj.push([x / sT, y / sT, z / sT]);
        }
        const grasp = [];
        for (let j = 0; j < gl[k]; j++) grasp.push(g[og++] / sG);
        pol[i].sides[side] = { traj, traj_step: step[k], grasp, gmax: g[d.ng + k] / sG, done: done[k] / sD,
                               grasping: d.grasping_bool ? !!gr[k] : gr[k] };
        k++;
      }
    }
    ro.policy = pol;
  }
  return ro;
}
