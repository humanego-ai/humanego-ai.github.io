// Live episode: the HumanEgo bimanual loop (runner.py) on MuJoCo WASM + the ONNX policy, in lock-step sim time.
import * as M from './math.js';
import { SimArm } from './driver.js';
import { SimController } from './controller.js';
import { mirrorPose } from './perception.js';
import { T_ALIGN } from './policy.js';

const yawQuat = (deg) => { const h = deg * Math.PI / 360; return [Math.cos(h), 0, 0, Math.sin(h)]; };   // wxyz about +z

export class LiveEpisode {
  constructor({ mj, model, data, spec, policy, cam }) {
    Object.assign(this, { mj, model, data, spec, policy, cam });
    this.cc = spec.control;
    this.sides = spec.sides;
    this.dt = model.opt.timestep;
    this.objNames = Object.keys(spec.objects);
    this.stats = { ik: 0, phys: 0, steps: 0, render: 0, infer: 0, img: 0, plans: 0 };
    globalThis.__liveStats = this.stats;
  }

  // ---------------------------------------------------------------- reset
  reset(scene, seed = 0) {
    const { mj, model, data, spec } = this;
    mj.mj_resetData(model, data);
    this.rng = M.makeRng(seed + 1);
    this.t = 0;
    for (const [name, [x, y, yaw]] of Object.entries(scene)) {
      const o = spec.objects[name]; if (!o) continue;
      if (o.soft) {     // soft bread: place the deformation grid rigidly (short drop-in)
        const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180), z = o.half_height + 0.0008 + 0.05;
        for (let i = 0; i < o.nodes.length; i++) {
          const lx = o.local[3 * i], ly = o.local[3 * i + 1], lz = o.local[3 * i + 2];
          const w = [x + c * lx - s * ly, y + s * lx + c * ly, z + lz];
          for (let k = 0; k < 3; k++) data.qpos[o.qidx[3 * i + k]] = w[k] - o.p0[3 * i + k];
        }
        continue;
      }
      const a = o.qadr, z = name === 'plate' ? 0.0005 : o.half_height + 0.0008;
      const q = yawQuat(yaw);
      data.qpos[a] = x; data.qpos[a + 1] = y; data.qpos[a + 2] = z + (name === 'plate' ? 0.02 : 0.05);   // short drop-in
      for (let i = 0; i < 4; i++) data.qpos[a + 3 + i] = q[i];
    }
    this.arms = {};
    for (const s of this.sides) {
      const arm = new SimArm(mj, model, data, s, spec.robot, spec.T_base_in_world[s], spec.T_base_in_cam[s], this.cc.tcp_offset || 0,
                            this.cc.tool_pitch_deg || 0);
      arm.setJInstant(spec.robot.initial_joints[s], spec.robot.gripper_open);
      this.arms[s] = arm;
    }
    mj.mj_forward(model, data);
    this.phase = 'settle';
    this.done = false;
    this.entries = {};
  }

  _step() {
    const t0 = performance.now();
    for (const a of Object.values(this.arms)) a.update(this.t);
    const t1 = performance.now();
    this.mj.mj_step(this.model, this.data);
    const t2 = performance.now();
    this.stats.ik += t1 - t0; this.stats.phys += t2 - t1; this.stats.steps++;
    this.t += this.dt;
  }

  // ---------------------------------------------------------------- perception (t = 0)
  _startPolicy() {
    const c = this.cc, sp = this.spec;
    this.objs = this.cam.estimateObjects(this.sides, this.objNames, this.rng, sp.camera.depth_noise, sp.perception?.n_init_frames ?? 10);
    this.ctx = {};
    for (const s of this.sides) this.ctx[s] = { side: s, flip: s === 'left', o: this.objs[s], active: s === 'right',
      hand: { grasping: false, latched: null, lock: null, released: false, ever: false }, done: false, wrap: null, wrapT: 0,
      graspT: null, lift: 0, liftAnchor: null, doneReason: '' };
    this.ctl = new SimController(this.arms, c);
    this.geoms = {};
    for (const s of this.sides) {
      this.geoms[s] = this._perceivedGeometry(s, this.objs[s]);
      if (this.geoms[s].bread) this.ctl.setGeometry(s, this.geoms[s].bread, sp.T_cam_in_world);
    }
    this.P = c.policy_period ?? 0.3; this.Tc = 1 / (c.control_freq ?? 10);
    this.nextPol = 0; this.nextCtl = 0; this.pending = null;
    this.t = 0; for (const a of Object.values(this.arms)) { a.t = 0; a.gT0 = 0; }
    this.phase = 'run';
  }

  /** runner.py _perceived_geometry: bread centre / long axis and a place height, from perception only. */
  _perceivedGeometry(side, o) {
    const Tcw = this.spec.T_cam_in_world, pts = {}, out = {};
    for (const [k, m] of Object.entries(o.meta || {})) {
      let Ts = m.T_static, kl = m.kpts_local;
      if (side === 'left') { Ts = mirrorPose(Ts); kl = kl.map((q) => [-q[0], q[1], q[2]]); }
      pts[k] = kl.map((q) => M.xform(Tcw, M.xform(Ts, q)));
    }
    const b = pts.obj1;
    if (b && b.length >= 3) {
      const c = [0, 1, 2].map((i) => b.reduce((acc, q) => acc + q[i], 0) / b.length);
      let sxx = 0, syy = 0, sxy = 0;
      for (const q of b) { const x = q[0] - c[0], y = q[1] - c[1]; sxx += x * x; syy += y * y; sxy += x * y; }
      const th = 0.5 * Math.atan2(2 * sxy, sxx - syy), a = [Math.cos(th), Math.sin(th), 0];
      out.bread = { c, a };
      const pl = pts.obj2;
      const floor = pl ? pl.reduce((acc, q) => acc + q[2], 0) / pl.length - 0.0135 : 0.0085;
      out.placeZ = c[2] + Math.max(floor, 0);
    }
    return out;
  }

  /** Lower (dz < 0) or raise the end effector by dz in world, keeping its orientation. */
  _moveWorldZ(arm, z, dur) {
    const Tw = M.mul4(arm.Tbw, arm.getTeeInBase()), T2 = M.setP(Tw, [Tw[3], Tw[7], z]);
    arm.moveP(M.T_to_p(M.mul4(arm.Twb, T2)), dur, this.ctl.safeZ);
  }

  graspVal(arm) {
    let closed = arm.gripperQ() < 0.5;
    if ((this.cc.grasp_state_source ?? 'measured_or_stalled') === 'measured_or_stalled') closed = closed || arm.settledClosed();
    return closed ? 1 : 0;
  }

  async _runSide(ctx, clean) {
    const arm = this.arms[ctx.side], st = ctx.hand, c = this.cc, pol = this.policy;
    const Tr = M.mul4(arm.getTeeInCam(), T_ALIGN), Tv = ctx.flip ? mirrorPose(Tr) : Tr;
    const gv = this.graspVal(arm), isG = gv > 0.5;
    if (!isG) st.released = true;
    if (isG && !st.grasping && st.released) {
      st.grasping = true; st.ever = true;
      ctx.lift = c.forced_lift_frames ?? 15; ctx.liftAnchor = arm.getP().slice(0, 3);
      let best = null, bd = 1e9;
      for (const [k, To] of Object.entries(ctx.o.dynamic)) { const d = M.norm(M.sub(M.getP(Tv), M.getP(To))); if (d < bd) { bd = d; best = k; } }
      if (best && bd < (this.spec.perception?.latch_dist_thresh ?? 0.2)) { st.latched = best; st.lock = M.mul4(M.inv4(Tv), ctx.o.dynamic[best]); } else st.latched = null;
    } else if (!isG && st.grasping) { st.grasping = false; st.latched = null; st.lock = null; }
    if (st.grasping && st.latched) ctx.o.dynamic[st.latched] = M.mul4(Tv, st.lock);
    if (c.use_idle_done && st.grasping) {
      if (ctx.graspT === null) ctx.graspT = this.t;
      if (this.t - ctx.graspT >= (c.idle_time_limit ?? 3)) for (const [k, To] of Object.entries(ctx.o.dynamic)) {
        if (k === 'obj1') continue;
        if (M.norm(M.sub(M.getP(Tv), M.getP(To))) < (c.idle_dist_threshold ?? 0.05)) { ctx.done = true; ctx.doneReason = 'idle_over_plate'; }
      }
    } else if (!st.grasping) ctx.graspT = null;
    // policy input + inference
    let tt = performance.now();
    const img = this.cam.sideImage(clean, ctx.side, ctx.o, arm.getPInCam(), isG);
    this.stats.img += performance.now() - tt;
    const objs = { obj1: ctx.o.dynamic.obj1, obj2: ctx.o.dynamic.obj2 };
    const ict = pol.buildICT(Tv, gv, objs);
    const K = this.cam.K, aK = ctx.flip ? [K[0], 0, 640 - K[2], 0, K[4], K[5], 0, 0, 1] : K;
    const uv = pol.anchorUV(ctx.o.dynamic.obj1, aK, 640, 480);
    tt = performance.now();
    const out = await pol.infer(img.chw, ict, uv);
    this.stats.infer += performance.now() - tt; this.stats.plans++;
    if (out.done > (c.done_threshold ?? 0.8) && (!c.done_requires_grasp || st.ever)) { ctx.done = true; ctx.doneReason = 'policy_done'; }
    // decode -> camera-frame EE targets (+ forced lift), mirrored back for the left arm
    const he = Math.min(c.exec_horizon ?? pol.K, pol.K), raw = [], world = [];
    for (let k = 0; k < he; k++) {
      const Tvk = pol.decode(out.pos[k], out.o6d[k]), Treal = ctx.flip ? mirrorPose(Tvk) : Tvk;
      let p = M.T_to_p(Treal);
      if (ctx.lift > 0) {
        const el = (c.forced_lift_frames ?? 15) - ctx.lift + 1, b = (c.forced_lift_height ?? 0.05) * Math.min(1, el / (c.forced_lift_ramp_frames ?? 5));
        const pb = arm.pInCamToBase(p); pb[2] += b; p = arm.pInBaseToCam(pb);
      }
      raw.push(p); world.push(M.xform(this.spec.T_cam_in_world, p.slice(0, 3)));
    }
    if (ctx.lift > 0) ctx.lift--;
    this.entries[ctx.side] = { traj: world, grasp: out.grasp.slice(0, 12), gmax: Math.max(...out.grasp), done: out.done, grasping: st.grasping, image: img.preview };
    return [raw, out.grasp];
  }

  async _policyTick() {
    if (this.pending) { this.ctl.updateTarget(...this.pending); this.pending = null; }
    if (this.sides.length > 1 && !this.ctx.left.active && this.t >= (this.cc.bimanual_start_delay ?? 6)) this.ctx.left.active = true;
    let tt = performance.now();
    const clean = this.cam.cleanImage();
    this.stats.render += performance.now() - tt;
    const tr = {}, gr = {};
    for (const s of ['right', 'left']) {
      const ctx = this.ctx[s]; if (!ctx || !ctx.active || ctx.wrap) continue;
      [tr[s], gr[s]] = await this._runSide(ctx, clean);
      this.onPolicy?.(s, this.entries[s]);
    }
    this.pending = [tr, gr];
    this.nextPol += this.P;
    for (const [s, ctx] of Object.entries(this.ctx)) {
      if (ctx.active && ctx.done && !ctx.wrap) {
        ctx.wrapT = this.t; this.ctl.muted[s] = true; this.ctl.clear(s);
        const a = this.arms[s], rel = this.cc.release || {}, g = this.geoms[s] || {};
        a.moveP(a.getP(), 0.05);
        if (rel.place_down && a.cmdClosed() && g.placeZ !== undefined) {     // set the bread down before opening
          const zg = g.placeZ + (rel.margin ?? 0.012), Tw = M.mul4(a.Tbw, a.getTeeInBase());
          if (Tw[11] > zg + 0.005) { this._moveWorldZ(a, zg, rel.place_time ?? 0.9); ctx.wrap = 'place'; }
        }
        if (!ctx.wrap) { ctx.wrap = 'open'; a.openGripper(0.5); }
        delete this.entries[s];
      }
    }
  }

  /** Advance the simulation to sim time `tTarget` (awaits the policy whenever a re-plan is due). */
  async advanceTo(tTarget) {
    if (this.done) return;
    while (this.t < tTarget && !this.done) {
      if (this.phase === 'settle') {
        this._step();
        if (this.t >= 0.6) this._startPolicy();
        continue;
      }
      if (this.t >= this.nextPol - 1e-9) { await this._policyTick(); continue; }
      for (const [s, ctx] of Object.entries(this.ctx)) {
        const rel = this.cc.release || {}, a = this.arms[s];
        if (ctx.wrap === 'place' && this.t - ctx.wrapT >= (rel.place_time ?? 0.9) + 0.15) { ctx.wrap = 'open'; ctx.wrapT = this.t; a.openGripper(0.5); }
        else if (ctx.wrap === 'open' && this.t - ctx.wrapT >= 0.8) {
          ctx.wrapT = this.t;
          if ((rel.lift ?? 0) > 0) {                      // lift the open hand straight up before going home
            const Tw = M.mul4(a.Tbw, a.getTeeInBase()); this._moveWorldZ(a, Tw[11] + rel.lift, 0.8); ctx.wrap = 'retreat';
          } else { ctx.wrap = 'home'; a.goHome(3); }
        } else if (ctx.wrap === 'retreat' && this.t - ctx.wrapT >= 0.9) { ctx.wrap = 'home'; ctx.wrapT = this.t; a.goHome(3); }
        else if (ctx.wrap === 'home' && this.t - ctx.wrapT >= 3.1) ctx.wrap = 'finished';
      }
      if (this.t >= this.nextCtl - 1e-9) { this.ctl.tick(); this.nextCtl += this.Tc; }
      this._step();
      if (Object.values(this.ctx).every((c) => c.wrap === 'finished') || this.t > (this.cc.max_episode_time ?? 40)) {
        this.done = true; this.onDone?.(this.evaluate());
      }
    }
  }

  /** Object centre in world: a rigid body's origin, or a soft bread's grid centroid. */
  objectPos(name) {
    const d = this.data, o = this.spec.objects[name];
    if (!o.soft) return [d.xpos[3 * o.body], d.xpos[3 * o.body + 1], d.xpos[3 * o.body + 2]];
    const c = [0, 0, 0];
    for (const b of o.nodes) for (let k = 0; k < 3; k++) c[k] += d.xpos[3 * b + k] / o.nodes.length;
    return c;
  }

  /** Lowest surface point of an object (soft bread: lowest flex vertex; else grid / body estimate). */
  lowestZ(name) {
    const d = this.data, m = this.model, o = this.spec.objects[name];
    if (o.soft && d.flexvert_xpos && m.flex_vertadr && o.flex !== undefined) {
      const a = m.flex_vertadr[o.flex], n = m.flex_vertnum[o.flex];
      let z = Infinity;
      for (let i = 0; i < n; i++) z = Math.min(z, d.flexvert_xpos[3 * (a + i) + 2]);
      return z;
    }
    if (o.soft) { let z = Infinity; for (const b of o.nodes) z = Math.min(z, d.xpos[3 * b + 2]); return z; }
    return this.objectPos(name)[2] - o.half_height;
  }

  /** Served = released and resting on the plate: centre over the plate, no part touching the table. */
  evaluate() {
    const sp = this.spec, out = {}, pl = this.objectPos('plate'), pp = [pl[0], pl[1]];
    for (const s of this.sides) {
      const n = `bread_${s}`, bp = this.objectPos(n), dxy = Math.hypot(bp[0] - pp[0], bp[1] - pp[1]);
      out[s] = { success: dxy < sp.objects.plate.R && this.lowestZ(n) > 0.003 && bp[2] < 0.12 && !this.arms[s].cmdClosed(),
                 dist_to_plate_center: dxy };
    }
    return out;
  }
}
