// Python <-> browser parity harness (scripts/parity_test.py). Runs the Live stack (runner.js, controller.js,
// driver.js on MuJoCo WASM) from the Python run's physics state at t = 0 with the Python run's perception, and
// logs what the Python trace logs: every policy call, every control tick's commands, the breads.
//   mode 'sync'  : before every control tick the physics state is overwritten with Python's, and the policy
//                  outputs are Python's: any difference in commands is a logic difference (controller /
//                  driver / runner), and the state just before each re-sync measures one tick of physics drift.
//   mode 'free'  : free-running WASM physics from the same t = 0 state; the policy outputs are Python's,
//                  replayed by (side, time).
//   mode 'full'  : the browser stack on its own: perception from the three.js renders of the same t = 0 state,
//                  the ONNX policy on the same noise, free-running physics. First, the ONNX graph is run on
//                  Python's exact inputs of the first re-plans (image, ICT, anchor uv) to check the policy alone.
import { LiveEpisode } from './runner.js';
import { LivePolicy } from './policy.js';

const flat = (T) => (Array.isArray(T[0]) ? T.flat() : T);

function objsFromTrace(per) {
  const out = {};
  for (const [side, o] of Object.entries(per)) {
    const meta = {};
    for (const [k, m] of Object.entries(o.meta)) meta[k] = { T_static: flat(m.T_static), kpts_local: m.kpts_local };
    const dynamic = {};
    for (const [k, T] of Object.entries(o.dynamic)) dynamic[k] = flat(T);
    out[side] = { meta, dynamic, T_anchor_static: flat(o.T_anchor_static) };
  }
  return out;
}

export async function runParity({ mj, model, data, spec, trace, mode = 'sync', policyBase = 'assets/policy/', cam = null,
                                  tMax = null, onProgress = null }) {
  const meta = await (await fetch(policyBase + 'serve_bread_meta.json')).json();
  let pol;
  const log = { mode, ticks: [], policy: [], pre: [], calls: [] };
  const toArr = (x) => Array.from(x);
  if (mode === 'full') {
    pol = await LivePolicy.create(policyBase);
    pol.noise = trace.noise;
    log.device = pol.device;
    log.onnx = [];
    for (const r of trace.policy.filter((r) => r.img)) {           // the policy alone, on Python's exact inputs
      const out = await pol.infer(Float32Array.from(r.img, (v) => v / 255), { x: Float32Array.from(r.ict), mask: Float32Array.from(r.mask) }, r.uv);
      log.onnx.push({ t: r.t, side: r.side, pos: out.pos.map(toArr), o6d: out.o6d.map(toArr), grasp: toArr(out.grasp), done: out.done });
    }
  } else {
    pol = Object.create(LivePolicy.prototype);
    Object.assign(pol, { meta, K: meta.pred_horizon, D: meta.action_dim, mean: meta.pos_mean, std: meta.pos_std });
  }
  pol.noise = trace.noise;
  const recs = { right: [], left: [] };
  for (const r of trace.policy) recs[r.side].push(r);
  if (mode !== 'full') {
    pol.infer = async (chw, ict, uv, side, t) => {
      // Python's network output for this side at this time (or the nearest earlier one if Python stopped re-planning)
      const rs = recs[side]; let best = null;
      for (const r of rs) if (r.t <= t + 1e-6) best = r;
      best = best || rs[0];
      log.calls.push({ side, t, py_t: best.t });
      return { pos: best.pos, o6d: best.o6d, grasp: best.grasp, done: best.done };
    };
  } else {
    // keep the browser's own first policy-input images, to compare with Python's
    const inner = pol.infer.bind(pol), seen = { right: 0, left: 0 };
    log.images = [];
    pol.infer = async (chw, ict, uv, side, t) => {
      if (seen[side]++ < 2) log.images.push({ side, t, img: Array.from(chw, (v) => Math.round(255 * v)) });
      return inner(chw, ict, uv);
    };
  }
  cam = cam || { K: spec.camera.K, cleanImage: () => null, sideImage: () => ({ chw: new Float32Array(3 * 240 * 320), preview: null }) };
  const ep = new LiveEpisode({ mj, model, data, spec, policy: pol, cam });
  let k = 0;
  const sideState = (ep, s) => {
    const a = ep.arms[s], c = ep.ctl, g = c.gs[s], x = ep.ctx[s];
    return { cmd: c.last[s] ? toArr(c.last[s]) : null, ee: toArr(a.getPInCam()), g_to: a.gTo, g_cmd: a.gcmd, skill: g.state,
             tries: g.tries, wrap: x.wrap, grasping: x.hand.grasping, n_events: c.events[s].length,
             bread: toArr(ep.objectPos(`bread_${s}`)), goal: toArr(a.ci.pGoal), mode: a.mode };
  };
  ep.hooks = {
    preTick: (ep) => {
      const py = trace.pre[k];
      const jsb = {};
      for (const s of ep.sides) jsb[s] = { bread: toArr(ep.objectPos(`bread_${s}`)), ee: toArr(ep.arms[s].getPInCam()) };
      log.pre.push({ t: ep.t, sides: jsb });
      if (mode === 'sync' && py) ep.loadState(py);
      k++;
    },
    postTick: (ep, t) => {
      const sides = {};
      for (const s of ep.sides) sides[s] = sideState(ep, s);
      log.ticks.push({ t, sides });
    },
    policy: (ctx, e) => {
      log.policy.push({ t: e.t, side: ctx.side, gv: e.gv, Tv: toArr(e.Tv), ict: toArr(e.ict.x), uv: e.uv,
                        latched: ctx.hand.latched, lift: ctx.lift, grasp: toArr(e.out.grasp), done: e.out.done,
                        pos: e.out.pos.map(toArr), o6d: e.out.o6d.map(toArr) });
    },
  };
  if (mode !== 'full') {        // re-perception before a re-grasp: Python's camera observations, replayed
    log.reperceive = [];
    ep.reperceiveSource = (side, t) => {
      const r = (trace.reperceive || []).find((q) => q.side === side && Math.abs(q.t - t) < 1e-6);
      log.reperceive.push({ side, t, py: r ? (r.so ? 'objs' : 'none') : 'missing' });
      return r && r.so ? objsFromTrace({ [side]: r.so })[side] : null;
    };
  }
  ep.reset({}, 0, { state: trace.init_state, objs: mode === 'full' ? null : objsFromTrace(trace.perception) });
  log.geoms = ep.geoms;
  if (mode === 'full') {
    log.perception = {};
    for (const [s, o] of Object.entries(ep.objs)) {
      log.perception[s] = { meta: {} };
      for (const [k, m] of Object.entries(o.meta)) log.perception[s].meta[k] = { T_static: toArr(m.T_static), kpts_local: m.kpts_local };
    }
  }
  const T = tMax ?? (spec.control.max_episode_time ?? 40);
  let res = null;
  ep.onDone = (r) => { res = r; };
  while (!ep.done && ep.t < T) {
    await ep.advanceTo(Math.min(ep.t + 1.0, T + 1e-9));
    onProgress?.(ep.t);
  }
  log.result = res || ep.evaluate();
  log.t_end = ep.t;
  log.events = ep.ctl.events;
  return log;
}
