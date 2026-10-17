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
    // grasp skill (controller.py): refine the pinch point with the perceived bread, close on arrival,
    // check the grip from the gripper reading, re-grasp lower / more centrally if it came up empty
    const ga = c.grasp_assist || {};
    this.ga = { on: !!ga.enabled, refine: ga.refine ?? true, lat: ga.lateral_gain ?? 0.7, alongMax: ga.along_max ?? 0.045,
      upMax: ga.height_max ?? 0.004, upMin: ga.height_min ?? -0.008, tol: ga.arrive_tol ?? 0.01, timeout: ga.approach_timeout ?? 1.5,
      hold: ga.close_hold ?? 0.6, maxTries: ga.regrasp_max ?? 2, qEmpty: ga.q_empty ?? 0.1, gapEmpty: ga.gap_empty ?? 0.015, cool: ga.regrasp_cooldown ?? 1.2,
      down: ga.retry_lower ?? 0.005, inward: ga.retry_inward ?? 0.01 };
    this.t = 0; this.geom = {}; this.Tcw = null;
    this.gpShared = o(null); this.gpActive = o(null);
    this.gs = o(() => ({ state: 'free', target: null, t0: 0, tries: 0, emptyT: null, coolUntil: 0 }));
    this.events = o(() => []);
  }
  setGeometry(side, g, Tcw) { this.geom[side] = g; this.Tcw = Tcw; }
  _refine(side, p) {
    const g = this.geom[side]; if (!this.ga.refine || !g || !this.Tcw) return p;
    const cl = (x, lo, hi) => Math.min(Math.max(x, lo), hi);
    const pw = M.xform(this.Tcw, p.slice(0, 3)), a = g.a, w = [-a[1], a[0], 0], d = M.sub(pw, g.c);
    const n = this.gs[side].tries, alongMax = Math.max(0, this.ga.alongMax - n * this.ga.inward), upMax = this.ga.upMax - n * this.ga.down;
    const sa = cl(M.dot(d, a), -alongMax, alongMax), sw = M.dot(d, w) * Math.pow(1 - this.ga.lat, 1 + n);
    const su = cl(d[2], Math.min(this.ga.upMin, upMax), upMax);
    const pw2 = [g.c[0] + sa * a[0] + sw * w[0], g.c[1] + sa * a[1] + sw * w[1], g.c[2] + su];
    return [...M.xform(M.inv4(this.Tcw), pw2), ...p.slice(3)];
  }
  _graspPoint(side, tr, i) {
    const gp = this.gpActive[side]; let k = i;
    if (gp) { const h = gp.findIndex((x) => x > this.gth); if (h >= 0) k = h; }
    k = Math.min(Math.max(k, 0), tr.length - 1);
    return this._refine(side, tr[k].slice());
  }
  _regrasp(side, arm, why) {
    const g = this.gs[side];
    g.tries++; g.state = 'free'; g.target = null; g.emptyT = null; g.coolUntil = this.t + this.ga.cool;
    this.locked[side] = false; this.everOpen[side] = false; this.grip[side] = 0;
    arm.openGripper(this.openT); this.events[side].push([this.t, 'regrasp:' + why]);
  }
  _skill(side, arm, want, tr, i, act) {
    const g = this.gs[side], G = this.ga;
    if (g.state === 'free') {
      if (want && this.t >= g.coolUntil && g.tries <= G.maxTries) { g.state = 'approach'; g.target = this._graspPoint(side, tr, i); g.t0 = this.t; this.events[side].push([this.t, 'approach']); }
      else return [null, false];
    }
    if (g.state === 'approach') {
      if (!want) { g.state = 'free'; g.target = null; return [null, false]; }
      if (M.norm(M.sub(act.slice(0, 3), g.target.slice(0, 3))) < G.tol || this.t - g.t0 > G.timeout) { g.state = 'closing'; g.t0 = this.t; this.events[side].push([this.t, 'close']); }
      else return [g.target, false];
    }
    if (g.state === 'closing') { if (this.t - g.t0 >= G.hold) { g.state = 'holding'; g.t0 = this.t; g.emptyT = null; } return [g.target, true]; }
    if (g.state === 'holding') {
      if (!want) { g.state = 'free'; g.target = null; return [null, false]; }
      if (arm.holdingNothing(G.qEmpty, G.gapEmpty)) {
        if (g.emptyT === null) g.emptyT = this.t;
        if (this.t - g.emptyT > 0.3 && g.tries < G.maxTries) { this._regrasp(side, arm, 'empty'); return [null, false]; }
      } else g.emptyT = null;
      return [null, true];
    }
    return [null, want];
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
      if (!this.muted[side]) { this.shared[side] = ens; this.grip[side] = sg ? 1 : 0; this.gpShared[side] = (grasp[side] || []).slice(0, ens.length); }
    }
  }
  clear(side) { this.shared[side] = []; this.histBuf[side] = []; this.active[side] = []; this.last[side] = null; }
  tick() {
    const dt = 1 / this.hz;
    this.t += dt;
    for (const [side, arm] of Object.entries(this.arms)) {
      if (this.shared[side].length) { this.active[side] = this.shared[side]; this.gpActive[side] = this.gpShared[side]; this.idx[side] = Math.min(this.look, this.active[side].length - 1); this.shared[side] = []; }
      if (this.muted[side]) { this.active[side] = []; this.last[side] = null; continue; }
      const tr = this.active[side], i = this.idx[side];
      if (!(tr.length && i < tr.length)) continue;
      let tgt = tr[i].slice();
      const act = arm.getPInCam();
      let want = this.grip[side] > this.gth, ovr = null;
      if (this.ga.on) { [ovr, want] = this._skill(side, arm, want, tr, i, act); if (ovr) tgt = ovr.slice(); }
      if (this.cage) {
        const dp = M.sub(tgt.slice(0, 3), act.slice(0, 3)), dist = M.norm(dp);
        if (dist > this.maxPos) { const p = M.add(act.slice(0, 3), M.scale(dp, this.maxPos / dist)); tgt[0] = p[0]; tgt[1] = p[1]; tgt[2] = p[2]; }
        const Ra = M.rotvecToMat(act.slice(3)), Rt = M.rotvecToMat(tgt.slice(3)), ang = M.rotAngle(Ra, Rt);
        if (ang > this.maxRot) { const r = M.matToRotvec(M.slerpMat(Ra, Rt, this.maxRot / ang)); tgt[3] = r[0]; tgt[4] = r[1]; tgt[5] = r[2]; }
      }
      let cmd;
      const last = this.last[side];
      if (!last || ovr) cmd = tgt;            // the grasp skill servoes straight to its point
      else {
        const p = last.slice(0, 3).map((v, k) => this.aPos * v + (1 - this.aPos) * tgt[k]);
        const r = M.matToRotvec(M.slerpMat(M.rotvecToMat(last.slice(3)), M.rotvecToMat(tgt.slice(3)), 1 - this.aRot));
        cmd = [...p, ...r];
      }
      this.last[side] = cmd.slice();
      const ok = arm.movePInCam(cmd, dt * this.durK, this.safeZ);
      if (!ok) { this.shared[side] = []; this.histBuf[side] = []; this.last[side] = act; }
      if (want) arm.closeGripper(this.closeT); else arm.openGripper(this.openT);
      this.idx[side] += this.stride;
    }
  }
}
