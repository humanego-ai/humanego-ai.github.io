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
      down: ga.retry_lower ?? 0.005, inward: ga.retry_inward ?? 0.01,
      near: ga.timeout_near ?? 0, approachMax: ga.approach_max ?? 4,
      // re-perceive before a re-grasp (controller.py): 'off' | 'skill' | 'policy'
      reperceive: ga.reperceive === true ? 'policy' : (!ga.reperceive ? 'off' : String(ga.reperceive)), rpTimeout: ga.reperceive_timeout ?? 1.5,
      // grip check while holding (controller.py): 'q' | 'touch' | 'effort'
      check: ga.grip_check ?? 'q', fMin: ga.touch_min ?? 0.3, onePad: ga.one_pad_empty ?? false, effEmpty: ga.effort_empty ?? 3,
      slipDrop: ga.slip_drop ?? 0, slipTime: ga.slip_time ?? 0.1, emptyTime: ga.empty_time ?? 0.3,
      // wrist yaw at the pinch point ('off' | 'align' | 'fit') and slow final approach (controller.py)
      yaw: ga.yaw ?? 'off', yawMax: (ga.yaw_max_deg ?? 60) * Math.PI / 180, yawMargin: ga.yaw_margin ?? 0.01,
      open: ga.max_opening ?? Object.values(arms)[0]?.R?.max_opening ?? 0.08, rotTol: (ga.arrive_rot_deg ?? 6) * Math.PI / 180,
      slowDist: ga.slow_dist ?? 0, slowSpeed: ga.slow_speed ?? 0.05 };
    this.reperceive = o(null);
    this.t = 0; this.geom = {}; this.Tcw = null;
    this.gpShared = o(null); this.gpActive = o(null);
    this.gs = o(() => ({ state: 'free', target: null, t0: 0, tries: 0, emptyT: null, coolUntil: 0 }));
    this.events = o(() => []);
  }
  setGeometry(side, g, Tcw) { this.geom[side] = g; this.Tcw = Tcw; }
  reperceived(side, g) {
    const old = this.geom[side]; this.geom[side] = g; this.reperceive[side] = null;
    this.events[side].push([this.t, 'reperceive', old ? Math.hypot(g.c[0] - old.c[0], g.c[1] - old.c[1]) : 0]);
  }
  _refine(side, p) {
    const g = this.geom[side]; if (!this.ga.refine || !g || !this.Tcw) return p;
    const cl = (x, lo, hi) => Math.min(Math.max(x, lo), hi);
    const pw = M.xform(this.Tcw, p.slice(0, 3)), a = g.a, w = [-a[1], a[0], 0], d = M.sub(pw, g.c);
    const n = this.gs[side].tries, alongMax = Math.max(0, this.ga.alongMax - n * this.ga.inward), upMax = this.ga.upMax - n * this.ga.down;
    const sa = cl(M.dot(d, a), -alongMax, alongMax), sw = M.dot(d, w) * Math.pow(1 - this.ga.lat, 1 + n);
    const su = cl(d[2], Math.min(this.ga.upMin, upMax), upMax);
    const pw2 = [g.c[0] + sa * a[0] + sw * w[0], g.c[1] + sa * a[1] + sw * w[1], g.c[2] + su];
    return [...M.xform(M.inv4(this.Tcw), pw2), ...(this.ga.yaw !== 'off' ? this._yawFix(g, p.slice(3)) : p.slice(3))];
  }
  /** controller.py _yaw_fix: turn the hand about the world vertical so its fingers close across the bread. */
  _yawFix(g, rv) {
    const G = this.ga, Rc = M.getR(this.Tcw), Rw = M.mul3(Rc, M.rotvecToMat(rv)), sp = [Rw[0], Rw[3]];
    if (Math.hypot(sp[0], sp[1]) < 1e-6) return rv;
    const phi = Math.atan2(sp[1], sp[0]) - Math.atan2(g.a[1], g.a[0]);
    const wrap = (x) => ((((x + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) - Math.PI;
    const c0 = wrap(Math.PI / 2 - phi), c1 = wrap(-Math.PI / 2 - phi);
    let d = Math.abs(c0) <= Math.abs(c1) ? c0 : c1;
    if (G.yaw === 'fit') {
      const L = g.half_len ?? 0.07, W = g.half_wid ?? 0.035, room = 0.5 * (G.open - G.yawMargin);
      const width = (an) => Math.hypot(L * Math.cos(an), W * Math.sin(an));
      if (width(phi) <= room) return rv;
      if (W < room) {
        const angFit = Math.acos(Math.sqrt(Math.max(0, (room * room - W * W) / Math.max(L * L - W * W, 1e-9))));
        d = Math.sign(d) * Math.max(0, Math.abs(d) - (Math.PI / 2 - angFit));
      }
    }
    d = Math.min(Math.max(d, -G.yawMax), G.yawMax);
    const cz = Math.cos(d), sz = Math.sin(d), Rz = [cz, -sz, 0, sz, cz, 0, 0, 0, 1];
    return M.matToRotvec(M.mul3(M.T3(Rc), M.mul3(Rz, Rw)));
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
    if (this.ga.reperceive !== 'off') this.reperceive[side] = this.t + this.ga.rpTimeout;
  }
  _skill(side, arm, want, tr, i, act) {
    const g = this.gs[side], G = this.ga;
    let rp = this.reperceive[side];
    if (rp !== null && this.t >= rp) { this.reperceive[side] = rp = null; this.events[side].push([this.t, 'reperceive:timeout']); }
    if (g.state === 'free') {
      if (want && this.t >= g.coolUntil && g.tries <= G.maxTries && rp === null) {
        g.state = 'approach'; g.target = this._graspPoint(side, tr, i); g.t0 = this.t; g.nearT = null; this.events[side].push([this.t, 'approach']);
      } else return [null, false];
    }
    if (g.state === 'approach') {
      if (!want) { g.state = 'free'; g.target = null; return [null, false]; }
      const err = M.norm(M.sub(act.slice(0, 3), g.target.slice(0, 3)));
      let timedOut;
      if (G.near > 0) {      // the timeout only counts once the arm is near the pinch point (approach_max: hard limit)
        if (g.nearT === null && err < G.near) g.nearT = this.t;
        timedOut = (g.nearT !== null && this.t - g.nearT > G.timeout) || this.t - g.t0 > G.approachMax;
      } else timedOut = this.t - g.t0 > G.timeout;
      let arrived = err < G.tol;
      if (arrived && G.yaw !== 'off') arrived = M.rotAngle(M.rotvecToMat(act.slice(3)), M.rotvecToMat(g.target.slice(3))) < G.rotTol;
      if (arrived || timedOut) { g.state = 'closing'; g.t0 = this.t; this.events[side].push([this.t, 'close']); }
      else {
        let tgt = g.target;
        if (G.slowDist > 0 && err < G.slowDist) {          // slow final approach
          const step = G.slowSpeed / this.hz;
          if (err > step) { tgt = tgt.slice(); for (let k = 0; k < 3; k++) tgt[k] = act[k] + (g.target[k] - act[k]) * (step / err); }
        }
        return [tgt, false];
      }
    }
    if (g.state === 'closing') { if (this.t - g.t0 >= G.hold) { g.state = 'holding'; g.t0 = this.t; g.emptyT = null; g.fPeak = null; } return [g.target, true]; }
    if (g.state === 'holding') {
      if (!want) { g.state = 'free'; g.target = null; return [null, false]; }
      const why = this._gripStatus(arm, g);
      if (why) {
        if (g.emptyT === null) g.emptyT = this.t;
        if (this.t - g.emptyT > (why === 'slip' ? G.slipTime : G.emptyTime) && g.tries < G.maxTries) { this._regrasp(side, arm, why); return [null, false]; }
      } else g.emptyT = null;
      return [null, true];
    }
    return [null, want];
  }
  /** '' while the grip holds, else 'empty' | 'one_pad' | 'slip' (controller.py _grip_status). */
  _gripStatus(arm, g) {
    const G = this.ga;
    if (G.check === 'touch') {
      const f = arm.padForces().sort((a, b) => a - b);
      if (f[1] < G.fMin) return 'empty';
      if (G.onePad && f[0] < G.fMin) return 'one_pad';
      g.fPeak = Math.max(g.fPeak || 0, f[0]);
      if (G.slipDrop > 0 && g.fPeak >= 2 * G.fMin && f[0] < G.slipDrop * g.fPeak) return 'slip';
      return '';
    }
    if (G.check === 'effort') {
      const e = arm.gripEffort();
      if (e < G.effEmpty) return 'empty';
      g.fPeak = Math.max(g.fPeak || 0, e);
      if (G.slipDrop > 0 && e < G.slipDrop * g.fPeak) return 'slip';
      return '';
    }
    return arm.holdingNothing(G.qEmpty, G.gapEmpty) ? 'empty' : '';
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
