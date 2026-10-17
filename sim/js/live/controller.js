// SimController (JS port of humanego_sim/controller.py = production AsyncRobotController on a sim clock).
import * as M from './math.js';

export class SimController {
  constructor(arms, c) {
    this.arms = arms;
    this.hz = c.control_freq ?? 10; this.hist = c.max_history_len ?? 1; this.stride = c.step_stride ?? 1;
    this.look = c.look_ahead_offset ?? 30; this.aPos = c.alpha ?? 0.8; this.aRot = c.alpha_rot ?? this.aPos;
    this.durK = c.duration_smooth_param ?? 1; this.gth = c.grasp_threshold ?? 0.5; this.method = c.should_grasp_method ?? 'first';
    this.kAny = c.grasp_k_any ?? 50; this.cage = c.use_safety_cage ?? true; this.maxPos = c.max_pos_step ?? 0.018;
    this.maxRot = c.max_rot_step ?? 0.1; this.safeZ = c.safe_z_min ?? -0.2; this.closeT = c.close_gripper_goal_time ?? 1;
    this.openT = c.open_gripper_goal_time ?? 1; this.latchOn = c.grasp_latch ?? false;
    const sides = Object.keys(arms);
    const o = (v) => Object.fromEntries(sides.map((s) => [s, typeof v === 'function' ? v() : v]));
    this.shared = o(() => []); this.grip = o(0); this.histBuf = o(() => []); this.active = o(() => []);
    this.idx = o(this.look); this.last = o(null); this.muted = o(false); this.everOpen = o(false); this.locked = o(false);
  }
  shouldGrasp(g) {
    const th = this.gth;
    if (this.method === 'any') return g.some((x) => x > th);
    if (this.method === 'any_first_k') return g.slice(0, Math.max(1, this.kAny)).some((x) => x > th);
    return g[0] > th;
  }
  updateTarget(traj, grasp) {
    for (const [side, chunk] of Object.entries(traj)) {
      if (!this.arms[side] || !chunk.length) continue;
      let sg = this.shouldGrasp(grasp[side]);
      const hb = this.histBuf[side]; hb.push(chunk); if (hb.length > this.hist) hb.shift();
      const ens = [];
      for (let k = 0; k < chunk.length; k++) {
        let sp = [0, 0, 0], ws = 0, qs = [];
        hb.forEach((ht, hi) => { const off = hb.length - 1 - hi, kk = k + off; if (kk < ht.length) { const w = Math.exp(-0.25 * off); sp = M.add(sp, M.scale(ht[kk].slice(0, 3), w)); ws += w; qs.push([M.matToQuat(M.rotvecToMat(ht[kk].slice(3))), w]); } });
        if (!ws) break;
        let q = qs[0][0];                                   // weighted quaternion mean (ok for 1-2 entries)
        for (let i = 1; i < qs.length; i++) q = M.quatSlerp(q, qs[i][0], qs[i][1] / (qs[i][1] + qs[0][1]));
        ens.push([...M.scale(sp, 1 / ws), ...M.matToRotvec(M.quatToMat(q))]);
      }
      if (this.latchOn) { if (!sg) this.everOpen[side] = true; else if (this.everOpen[side]) this.locked[side] = true; if (this.locked[side]) sg = true; }
      if (!this.muted[side]) { this.shared[side] = ens; this.grip[side] = sg ? 1 : 0; }
    }
  }
  clear(side) { this.shared[side] = []; this.histBuf[side] = []; this.active[side] = []; this.last[side] = null; }
  tick() {
    const dt = 1 / this.hz;
    for (const [side, arm] of Object.entries(this.arms)) {
      if (this.shared[side].length) { this.active[side] = this.shared[side]; this.idx[side] = Math.min(this.look, this.active[side].length - 1); this.shared[side] = []; }
      if (this.muted[side]) { this.active[side] = []; this.last[side] = null; continue; }
      const tr = this.active[side], i = this.idx[side];
      if (!(tr.length && i < tr.length)) continue;
      const tgt = tr[i].slice(), act = arm.getPInCam();
      if (this.cage) {
        const dp = M.sub(tgt.slice(0, 3), act.slice(0, 3)), dist = M.norm(dp);
        if (dist > this.maxPos) { const p = M.add(act.slice(0, 3), M.scale(dp, this.maxPos / dist)); tgt[0] = p[0]; tgt[1] = p[1]; tgt[2] = p[2]; }
        const Ra = M.rotvecToMat(act.slice(3)), Rt = M.rotvecToMat(tgt.slice(3)), ang = M.rotAngle(Ra, Rt);
        if (ang > this.maxRot) { const r = M.matToRotvec(M.slerpMat(Ra, Rt, this.maxRot / ang)); tgt[3] = r[0]; tgt[4] = r[1]; tgt[5] = r[2]; }
      }
      let cmd;
      const last = this.last[side];
      if (!last) cmd = tgt;
      else {
        const p = last.slice(0, 3).map((v, k) => this.aPos * v + (1 - this.aPos) * tgt[k]);
        const r = M.matToRotvec(M.slerpMat(M.rotvecToMat(last.slice(3)), M.rotvecToMat(tgt.slice(3)), 1 - this.aRot));
        cmd = [...p, ...r];
      }
      this.last[side] = cmd.slice();
      const ok = arm.movePInCam(cmd, dt * this.durK, this.safeZ);
      if (!ok) { this.shared[side] = []; this.histBuf[side] = []; this.last[side] = act; }
      if (this.grip[side] > this.gth) arm.closeGripper(this.closeT); else arm.openGripper(this.openT);
      this.idx[side] += this.stride;
    }
  }
}
