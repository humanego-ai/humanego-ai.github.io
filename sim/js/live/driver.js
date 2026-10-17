// SimArm (JS port of humanego_sim/driver.py): Trossen-driver behaviour on MuJoCo WASM.
import * as M from './math.js';

function quinticCoeffs(x0, v0, a0, x1, T) {
  return x0.map((_, i) => {
    const d = x1[i] - x0[i];
    return [x0[i], v0[i], a0[i] / 2,
      (20 * d - 12 * v0[i] * T - 3 * a0[i] * T * T) / (2 * T ** 3),
      (-30 * d + 16 * v0[i] * T + 3 * a0[i] * T * T) / (2 * T ** 4),
      (12 * d - 6 * v0[i] * T - a0[i] * T * T) / (2 * T ** 5)];
  });
}
function quinticEval(C, tau) {
  const p = [], v = [], a = [];
  for (const c of C) {
    p.push(c[0] + tau * (c[1] + tau * (c[2] + tau * (c[3] + tau * (c[4] + tau * c[5])))));
    v.push(c[1] + tau * (2 * c[2] + tau * (3 * c[3] + tau * (4 * c[4] + tau * 5 * c[5]))));
    a.push(2 * c[2] + tau * (6 * c[3] + tau * (12 * c[4] + tau * 20 * c[5])));
  }
  return [p, v, a];
}

class CartInterp {
  constructor(T0) { this.reset(T0); }
  reset(T0) {
    this.p = M.getP(T0); this.v = [0, 0, 0]; this.a = [0, 0, 0];
    this.R = M.getR(T0); this.w = [0, 0, 0]; this.al = [0, 0, 0];
    this.active = false; this.Rg = this.R.slice();
  }
  setGoal(Tg, dur, t) {
    dur = Math.max(dur, 1e-3);
    this.Rg = M.getR(Tg);
    const RgT = M.T3(this.Rg);
    const r0 = M.matToRotvec(M.mul3(RgT, this.R));
    this.Cp = quinticCoeffs(this.p, this.v, this.a, M.getP(Tg), dur);
    this.Cr = quinticCoeffs(r0, M.mul3v(RgT, this.w), M.mul3v(RgT, this.al), [0, 0, 0], dur);
    this.t0 = t; this.T = dur; this.active = true;
  }
  update(t) {
    if (this.active) {
      const tau = Math.min(Math.max(t - this.t0, 0), this.T);
      [this.p, this.v, this.a] = quinticEval(this.Cp, tau);
      const [r, dr, ddr] = quinticEval(this.Cr, tau);
      this.R = M.mul3(this.Rg, M.rotvecToMat(r));
      this.w = M.mul3v(this.Rg, dr); this.al = M.mul3v(this.Rg, ddr);
      if (t - this.t0 >= this.T) { this.active = false; this.v = [0, 0, 0]; this.a = [0, 0, 0]; this.w = [0, 0, 0]; this.al = [0, 0, 0]; }
    }
    return M.fromRp(this.R, this.p);
  }
}

class JointInterp {
  constructor(q) { this.q = q.slice(); this.active = false; }
  setGoal(qg, dur, t) { const z = this.q.map(() => 0); this.C = quinticCoeffs(this.q, z, z, qg, Math.max(dur, 1e-3)); this.t0 = t; this.T = Math.max(dur, 1e-3); this.active = true; }
  update(t) {
    if (this.active) { this.q = quinticEval(this.C, Math.min(Math.max(t - this.t0, 0), this.T))[0]; if (t - this.t0 >= this.T) this.active = false; }
    return this.q;
  }
}

export class SimArm {
  constructor(mj, model, data, side, R, T_base_in_world, T_base_in_cam, tcpOffset = 0) {
    Object.assign(this, { mj, model, data, side, R });
    this.pre = side + '/';
    this.Tbw = T_base_in_world; this.Twb = M.inv4(T_base_in_world); this.Tbc = T_base_in_cam;
    const Rt = R.R_tool_to_ee;   // 3x3 row-major
    const off = M.mul3v(Rt, [0, 0, tcpOffset]);
    this.C = M.fromRp(Rt, off); this.Ci = M.inv4(this.C);
    const jid = (n) => model.jnt(this.pre + n).id;
    this.jids = R.arm_joints.map(jid);
    this.qadr = this.jids.map((j) => model.jnt_qposadr[j]);
    this.dadr = this.jids.map((j) => model.jnt_dofadr[j]);
    this.qlo = this.jids.map((j) => model.jnt_range[2 * j]); this.qhi = this.jids.map((j) => model.jnt_range[2 * j + 1]);
    this.aids = R.arm_actuators.map((n) => model.actuator(this.pre + n).id);
    this.gid = model.actuator(this.pre + R.gripper_actuator).id;
    this.sid = model.site(this.pre + R.tool_site).id;
    const gj = jid(R.gripper_joint); this.gq = model.jnt_qposadr[gj]; this.gd = model.jnt_dofadr[gj];
    this.adh = (R.finger_bodies || []).map((b) => { try { return model.actuator(`${this.pre}adhesion_${b}`).id; } catch (e) { return -1; } }).filter((i) => i >= 0);
    this.scratch = new mj.MjData(model);
    this.jacp = new mj.DoubleBuffer(3 * model.nv); this.jacr = new mj.DoubleBuffer(3 * model.nv);
    this.mode = 'joint'; this.t = 0; this.qcmd = this.getJ();
    this.ji = new JointInterp(this.qcmd); this.ci = new CartInterp(this.fkRaw(this.qcmd));
    this.gcmd = R.gripper_open; this.gFrom = this.gcmd; this.gTo = this.gcmd; this.gT0 = 0; this.gT = 1e-3;
  }
  getJ() { return this.qadr.map((a) => this.data.qpos[a]); }
  setJInstant(q, gripper) {
    q.forEach((v, i) => { this.data.qpos[this.qadr[i]] = v; this.data.qvel[this.dadr[i]] = 0; this.data.ctrl[this.aids[i]] = v; });
    this.qcmd = q.slice(); this.ji = new JointInterp(q); this.ci.reset(this.fkRaw(q)); this.mode = 'joint';
    if (gripper !== undefined) {
      this.gcmd = this.gFrom = this.gTo = gripper; this.data.ctrl[this.gid] = gripper;
      for (const [jn, v] of Object.entries(this.R.gripper_reset_qpos || {})) this.data.qpos[this.model.jnt_qposadr[this.model.jnt(this.pre + jn).id]] = v;
    }
  }
  _siteT(d) {
    const s = this.sid, x = d.site_xpos, m = d.site_xmat;
    return M.fromRp([m[9 * s], m[9 * s + 1], m[9 * s + 2], m[9 * s + 3], m[9 * s + 4], m[9 * s + 5], m[9 * s + 6], m[9 * s + 7], m[9 * s + 8]], [x[3 * s], x[3 * s + 1], x[3 * s + 2]]);
  }
  fkRaw(q) {
    const d = this.scratch; d.qpos.set(this.data.qpos); q.forEach((v, i) => { d.qpos[this.qadr[i]] = v; });
    this.mj.mj_kinematics(this.model, d);
    return M.mul4(this.Twb, this._siteT(d));
  }
  rawInBase() { return M.mul4(this.Twb, this._siteT(this.data)); }
  getTeeInBase() { return M.mul4(this.rawInBase(), this.C); }
  getTeeInCam() { return M.mul4(this.Tbc, this.getTeeInBase()); }
  getP() { return M.T_to_p(this.getTeeInBase()); }
  getPInCam() { return M.T_to_p(this.getTeeInCam()); }
  pInCamToBase(p) { return M.T_to_p(M.mul4(M.inv4(this.Tbc), M.p_to_T(p))); }
  pInBaseToCam(p) { return M.T_to_p(M.mul4(this.Tbc, M.p_to_T(p))); }

  ik(Traw, q0, iters = 3, tol = 2e-4) {
    const d = this.scratch, n = this.qadr.length, nv = this.model.nv;
    let q = q0.slice();
    const Tw = M.mul4(this.Tbw, Traw), pt = M.getP(Tw), Rt = M.getR(Tw);
    const W = [1, 1, 1, 0.5, 0.5, 0.5], lam2 = 0.02 * 0.02;
    for (let it = 0; it < iters; it++) {
      d.qpos.set(this.data.qpos); q.forEach((v, i) => { d.qpos[this.qadr[i]] = v; });
      this.mj.mj_kinematics(this.model, d); this.mj.mj_comPos(this.model, d);
      const T = this._siteT(d);
      const ep = M.sub(pt, M.getP(T)), er = M.matToRotvec(M.mul3(Rt, M.T3(M.getR(T))));
      if (M.norm(ep) < tol && M.norm(er) < 2e-3) break;
      this.mj.mj_jacSite(this.model, d, this.jacp, this.jacr, this.sid);
      const Jp = this.jacp.GetView(), Jr = this.jacr.GetView();
      const J = [];   // 6 x n (weighted)
      for (let r = 0; r < 3; r++) J.push(this.dadr.map((c) => W[r] * Jp[r * nv + c]));
      for (let r = 0; r < 3; r++) J.push(this.dadr.map((c) => W[3 + r] * Jr[r * nv + c]));
      const e = [...ep, ...er.map((x) => 0.5 * x)].map((x, i) => W[i] * x);
      const A = new Array(36).fill(0);
      for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) { let s = 0; for (let k = 0; k < n; k++) s += J[i][k] * J[j][k]; A[6 * i + j] = s + (i === j ? lam2 : 0); }
      const y = M.solve(A, e, 6);
      let dq = new Array(n).fill(0);
      for (let k = 0; k < n; k++) for (let i = 0; i < 6; i++) dq[k] += J[i][k] * y[i];
      const nn = M.norm(dq); if (nn > 0.3) dq = dq.map((x) => x * 0.3 / nn);
      q = q.map((v, i) => Math.min(Math.max(v + dq[i], this.qlo[i]), this.qhi[i]));
    }
    d.qpos.set(this.data.qpos); q.forEach((v, i) => { d.qpos[this.qadr[i]] = v; });
    this.mj.mj_kinematics(this.model, d);
    return [q, M.norm(M.sub(pt, M.getP(this._siteT(d))))];
  }

  moveP(pose, dur = 3, safeZ = -0.12) {
    let Traw = M.mul4(M.p_to_T(pose), this.Ci);
    if (Traw[11] < safeZ) Traw[11] = safeZ;
    const [, err] = this.ik(Traw, this.qcmd, 30);
    if (err > 0.01) return false;
    if (this.mode !== 'cartesian') { this.ci.reset(this.fkRaw(this.qcmd)); this.mode = 'cartesian'; }
    this.ci.setGoal(Traw, dur, this.t);
    return true;
  }
  movePInCam(p, dur, safeZ) { return this.moveP(this.pInCamToBase(p), dur, safeZ); }
  moveJ(q, dur = 3) { this.ji = new JointInterp(this.qcmd); this.ji.setGoal(q.map((v, i) => Math.min(Math.max(v, this.qlo[i]), this.qhi[i])), dur, this.t); this.mode = 'joint'; }
  goHome(dur = 3) { this.moveJ(this.R.home_joints || this.qcmd.map(() => 0), dur); }
  setGripper(pos, goal = 1) {
    const lo = Math.min(this.R.gripper_closed, this.R.gripper_open), hi = Math.max(this.R.gripper_closed, this.R.gripper_open);
    pos = Math.min(Math.max(pos, lo), hi);
    if (Math.abs(pos - this.gTo) < 1e-6) return;
    this.gFrom = this.gcmd; this.gTo = pos; this.gT0 = this.t; this.gT = Math.max(goal, 1e-3);
  }
  closeGripper(goal = 1) { this.setGripper(this.R.gripper_closed, goal); }
  openGripper(goal = 1) { this.setGripper(this.R.gripper_open, goal); }
  gripperQ() { const r = this.R, g = this.data.qpos[this.gq]; return Math.min(Math.max((g - r.gripper_joint_closed) / (r.gripper_joint_open - r.gripper_joint_closed), 0), 1); }
  cmdClosed() { return Math.abs(this.gTo - this.R.gripper_closed) < Math.abs(this.gTo - this.R.gripper_open); }
  settledClosed() {
    if (!this.cmdClosed() || this.t - this.gT0 < this.gT) return false;
    const r = this.R, v = Math.abs(this.data.qvel[this.gd]) / Math.max(Math.abs(r.gripper_joint_open - r.gripper_joint_closed), 1e-6);
    return v < 0.004 / 0.044;
  }
  update(t) {
    this.t = t;
    if (this.mode === 'joint') this.qcmd = this.ji.update(t);
    else if (this.mode === 'cartesian') this.qcmd = this.ik(this.ci.update(t), this.qcmd)[0];
    if (this.mode !== 'idle') this.qcmd.forEach((v, i) => { this.data.ctrl[this.aids[i]] = v; });
    const a = Math.min(Math.max((t - this.gT0) / this.gT, 0), 1);
    this.gcmd = this.gFrom + (this.gTo - this.gFrom) * a;
    this.data.ctrl[this.gid] = this.gcmd;
    const on = this.cmdClosed() && a > 0.6 ? 1 : 0;
    for (const i of this.adh) this.data.ctrl[i] = on;
  }
}
