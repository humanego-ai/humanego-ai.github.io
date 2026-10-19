// HumanEgo Real2Sim page: interactive replays of physics-verified DexAgent episodes on the Tianji + SharpaWave rig,
// plus the pipeline's artifacts, the agent's record and the generated dataset.
import { Viewer } from './viewer.js';

const $ = s => document.querySelector(s);
const el = (tag, cls, html) => { const e = document.createElement(tag); if (cls) e.className = cls; if (html != null) e.innerHTML = html; return e; };
const fmt = (x, d = 1) => (x == null || Number.isNaN(x)) ? '—' : Number(x).toFixed(d);

const PHASES = [
  ['pull', /lift spray|pull|undock/i, '#f472b6'],
  ['turn', /open faucet|open_joint|turn the lever|swing/i, '#facc15'],
  ['hold', /\bhold\b|\| right: wait/i, '#a3e635'],
  ['via', /return home #\d+\.\d/i, '#6b7586'],
  ['reach', /reach|approach|pre-?grasp/i, '#8ea3c0'],
  ['descend', /descend|lower to/i, '#5fb3d9'],
  ['grasp', /close|grip|grasp|squeeze/i, '#c084fc'],
  ['lift', /lift|raise/i, '#ff9a3c'],
  ['carry', /carry|transport|move/i, '#ffd36b'],
  ['place', /set down|place|lower|down\)/i, '#3ddc97'],
  ['release', /let go|release|open/i, '#22d3ee'],
  ['retreat', /retreat|withdraw|home|settle|rest/i, '#6b7586'],
];
function phaseOf(label) {
  for (const [k, re, c] of PHASES) if (re.test(label)) return { k, c };
  return { k: 'other', c: '#4b5563' };
}
function cleanLabel(l) {
  return l.replace(/^(right|left|both):\s*/i, '').replace(/croissant_croissant/g, 'croissant').replace(/blue_plate_blue_plate/g, 'plate')
    .replace(/spray_head_spray_head/g, 'spray head').replace(/faucet_faucet_lever\w*/g, 'lever').replace(/flower_pot_flower_pot_pot/g, 'pot')
    .replace(/\s*#\d+\s*$/, '');
}

const state = { run: 'right', ep: null, cam: 'orbit' };
let MAN, SUM, viewer;

async function main() {
  [MAN, SUM] = await Promise.all([fetch('data/manifest.json').then(r => r.json()), fetch('data/summary.json').then(r => r.json())]);
  let PHOTO = null;
  try { PHOTO = await fetch('data/photoreal.json').then(r => r.ok ? r.json() : null); } catch (e) { PHOTO = null; }
  renderStats();
  viewer = new Viewer($('#viewport'));
  viewer.loop = true;
  viewer.onTime = onTime;
  viewer.onUserCamera = () => setCamPill('orbit');
  wireControls();
  const qs = new URLSearchParams(location.search);
  await selectRun(MAN.runs[qs.get('run')] ? qs.get('run') : 'right');
  if (qs.get('ep')) await loadEpisode(qs.get('ep'));
  if (qs.get('cam')) setCam(qs.get('cam'));
  if (qs.get('t') != null) { setPlaying(false); viewer.applyTime(+qs.get('t')); }
  $('#loading').classList.add('hide');
  renderPipeline(); renderAgent(); renderTools(); renderData(); renderPhoto(PHOTO);
  document.body.dataset.ready = '1';
}

// ------------------------------------------------------------------ hero
function renderStats() {
  const r = MAN.runs, d = SUM.dataset || {}, T = SUM.totals;
  const verified = T ? T.physics_samples : Object.values(r).reduce((a, x) => a + (x.n_verified_total || 0), 0);
  const chips = [
    ['2', 'human videos · 2 tasks'],
    ['3 × 3', 'DexAgent stages passed'],
    [verified, 'physics-verified robot episodes'],
    [T ? T.rendered_episodes : (d.episodes ?? '—'), 'rendered robot-camera episodes'],
    [T ? (T.frames / 1000).toFixed(0) + 'k' : (d.frames ? (d.frames / 1000).toFixed(1) + 'k' : '—'), 'frames'],
    [(SUM.new_tools?.length ?? 0) + (SUM.water_tools ?? 0), 'tools the agent created'],
    ['7 + 22', 'DOF per arm + hand'],
  ];
  $('#stats').innerHTML = chips.map(([v, k]) => `<span class="chip"><b>${v}</b> ${k}</span>`).join('');
}

// ------------------------------------------------------------------ controls
function wireControls() {
  const runs = $('#pills-run');
  for (const [k, r] of Object.entries(MAN.runs)) {
    const b = el('button', 'pill' + (k === state.run ? ' on' : ''), r.label); b.dataset.run = k;
    b.onclick = () => selectRun(k); runs.appendChild(b);
  }
  const cams = $('#pills-cam');
  for (const [k, l] of [['orbit', 'Orbit'], ['zed', 'Robot ZED'], ['wrist', 'Wrist'], ['human', 'Human eye'], ['top', 'Top']]) {
    const b = el('button', 'pill' + (k === 'orbit' ? ' on' : ''), l); b.dataset.cam = k; b.onclick = () => setCam(k); cams.appendChild(b);
  }
  document.querySelectorAll('.hud-br .icon-btn').forEach(b => b.onclick = () => setCam(b.dataset.cam));
  $('#play').onclick = () => setPlaying(!viewer.playing);
  $('#scrub').oninput = e => { setPlaying(false); viewer.applyTime(viewer.duration * e.target.value / 1000); };
  document.querySelectorAll('#speed button').forEach(b => b.onclick = () => {
    document.querySelectorAll('#speed button').forEach(x => x.classList.toggle('on', x === b));
    viewer.speed = +b.dataset.s; const v = $('#pip-video'); v.playbackRate = Math.max(0.25, viewer.speed * pipRate());
  });
  $('#btn-shuffle').onclick = () => {
    const eps = MAN.runs[state.run].episodes.filter(e => e.id !== state.ep);
    loadEpisode(eps[Math.floor(Math.random() * eps.length)].id);
  };
  $('#btn-orig').onclick = () => loadEpisode('original');
  window.addEventListener('keydown', e => { if (e.code === 'Space' && e.target === document.body) { e.preventDefault(); setPlaying(!viewer.playing); } });
}

function setCamPill(k) {
  state.cam = k;
  document.querySelectorAll('#pills-cam .pill').forEach(b => b.classList.toggle('on', b.dataset.cam === k));
  document.querySelectorAll('.hud-br .icon-btn').forEach(b => b.classList.toggle('on', b.dataset.cam === k));
}
function setCam(k) { setCamPill(k); viewer.setCamera(k); }

function setPlaying(p) {
  viewer.playing = p;
  $('#play-ico').innerHTML = p ? '<path d="M6 5h4v14H6zM14 5h4v14h-4z"/>' : '<path d="M8 5v14l11-7z"/>';
  const v = $('#pip-video');
  if (p) { v.play().catch(() => {}); } else v.pause();
}

async function selectRun(k) {
  state.run = k;
  document.querySelectorAll('#pills-run .pill').forEach(b => b.classList.toggle('on', b.dataset.run === k));
  const run = MAN.runs[k];
  $('#loading').classList.remove('hide'); $('#loading-msg').textContent = 'Loading the Tianji + SharpaWave rig…';
  await viewer.loadModel(`data/models/${run.model}.json`);
  viewer.wristCam = run.wrist || `wrist_${run.side}`;
  const v = $('#pip-video'); v.src = `media/human_${k}.mp4`; v.load();
  $('#pip-cap').textContent = k === 'left' ? 'Aria · mirrored · time-normalized' : k === 'water' ? 'Aria · both hands · time-normalized' : 'Aria · time-normalized';
  drawMinimap();
  const help = document.querySelector('.mm-help');
  if (help) help.innerHTML = run.task === 'water_flowers'
    ? 'Each dot is one <b>physics-verified episode</b> (the pot\'s start inside the sink; the faucet and its docked spray head stay mounted). The ★ is the agent\'s episode retargeted from the video; the rest are DexAgent Stage 4 randomizations, every one re-simulated in contact physics. Click to load.'
    : 'Each pair is one <b>physics-verified episode</b> (● croissant, ○ plate start). The ★ is the agent\'s episode retargeted from the video; the rest are DexAgent Stage 4 randomizations, every one re-simulated in contact physics. Click to load.';
  await loadEpisode('original');
  $('#loading').classList.add('hide');
  $('#count').textContent = `${run.episodes.length} episodes shown · ${run.n_verified_total} verified in this run`;
}

async function loadEpisode(id) {
  const res = await fetch(`data/episodes/${state.run}/${id}.json`);
  if (!res.ok) { if (id !== 'original') return loadEpisode('original'); throw new Error('episode not found'); }
  const ep = await res.json();
  state.ep = id; state.epData = ep;
  viewer.setEpisode(ep);
  const meta = MAN.runs[state.run].episodes.find(e => e.id === id);
  $('#hud-title').innerHTML = `<span class="dot"></span><span>${MAN.runs[state.run].label} · ${id === 'original' ? '★ agent episode' : id.replace('shard_', 'sample ').replace('_', '/')}</span>`;
  renderResult(ep.metrics, meta);
  drawTrack(); renderPhaseLegend();
  document.querySelectorAll('#minimap .ep').forEach(g => g.classList.toggle('on', g.dataset.id === id));
  $('#mm-meta').textContent = id === 'original' ? 'retargeted from the video' : 'Stage 4 randomization';
  setPlaying(true);
}

function pipRate() {
  const v = $('#pip-video');
  return v.duration && viewer.duration ? v.duration / viewer.duration : 1;
}

function onTime(t) {
  const d = viewer.duration || 1;
  $('#scrub').value = Math.round(1000 * t / d);
  $('#tlabel').textContent = `${t.toFixed(1)} / ${d.toFixed(1)} s`;
  const f = Math.round(t * viewer.fps), ph = state.epData?.phases || [];
  let cur = ph[0]; for (const p of ph) { if (p[0] <= f) cur = p; else break; }
  if (cur) { const P = phaseOf(cur[1]); $('#hud-phase').innerHTML = `<b style="color:${P.c}">${P.k}</b>${cleanLabel(cur[1])}`; }
  const v = $('#pip-video');
  if (v.duration) {
    const want = (t / d) * v.duration;
    if (Math.abs(v.currentTime - want) > 0.35) v.currentTime = want;
    v.playbackRate = Math.max(0.25, Math.min(4, viewer.speed * pipRate()));
  }
  drawPlayhead(t / d);
}

// ------------------------------------------------------------------ timeline
function drawTrack() {
  const c = $('#track-canvas'), r = c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  c.width = r.width * dpr; c.height = r.height * dpr;
  state.trackBase = null;
  const g = c.getContext('2d'); g.scale(dpr, dpr);
  const W = r.width, H = r.height, ph = state.epData.phases, n = state.epData.frames.length;
  g.fillStyle = 'rgba(255,255,255,0.08)'; g.beginPath(); g.roundRect(0, H / 2 - 4, W, 8, 4); g.fill();
  ph.forEach((p, i) => {
    const a = p[0] / (n - 1), b = (i + 1 < ph.length ? ph[i + 1][0] : n - 1) / (n - 1);
    g.fillStyle = phaseOf(p[1]).c; g.globalAlpha = 0.9;
    g.fillRect(a * W, H / 2 - 4, Math.max(1, (b - a) * W - 1), 8);
  });
  g.globalAlpha = 1;
  state.trackBase = g.getImageData(0, 0, c.width, c.height);
}
function drawPlayhead(u) {
  const c = $('#track-canvas'); if (!state.trackBase) return;
  const g = c.getContext('2d'), dpr = window.devicePixelRatio || 1;
  g.putImageData(state.trackBase, 0, 0);
  g.save(); g.setTransform(1, 0, 0, 1, 0, 0);
  const x = u * c.width;
  g.fillStyle = '#fff'; g.shadowColor = 'rgba(255,140,43,.9)'; g.shadowBlur = 8 * dpr;
  g.beginPath(); g.arc(x, c.height / 2, 6 * dpr, 0, Math.PI * 2); g.fill(); g.restore();
}
function renderPhaseLegend() {
  const seen = new Map();
  for (const p of state.epData.phases) { const P = phaseOf(p[1]); if (!seen.has(P.k)) seen.set(P.k, P.c); }
  $('#phase-legend').innerHTML = [...seen].map(([k, c]) => `<span class="ph"><i style="background:${c}"></i>${k}</span>`).join('')
    + '<span>· drag to orbit, buttons switch to the robot\'s ZED, the wrist camera or the human\'s eye</span>';
}
window.addEventListener('resize', () => { if (state.epData) drawTrack(); });

// ------------------------------------------------------------------ minimap + outcome
function drawMinimap() {
  if (MAN.runs[state.run].task === 'water_flowers') return drawMinimapWater();
  const svg = $('#minimap'), W = 340, H = 260, eps = MAN.runs[state.run].episodes;
  const xs = [-0.32, 0.38], ys = [0.27, 0.65];
  const X = x => 20 + (x - xs[0]) / (xs[1] - xs[0]) * (W - 40), Y = y => H - 20 - (y - ys[0]) / (ys[1] - ys[0]) * (H - 40);
  let s = `<rect class="tbl" x="8" y="8" width="${W - 16}" height="${H - 16}" rx="10"/>`;
  for (let gx = -0.3; gx <= 0.381; gx += 0.1) s += `<line class="grid" x1="${X(gx)}" y1="12" x2="${X(gx)}" y2="${H - 12}"/>`;
  for (let gy = 0.3; gy <= 0.651; gy += 0.1) s += `<line class="grid" x1="12" y1="${Y(gy)}" x2="${W - 12}" y2="${Y(gy)}"/>`;
  s += `<text x="${X(0) - 22}" y="${H - 11}">robot ↓</text>`;
  const r = 0.1267 / (xs[1] - xs[0]) * (W - 40);
  const ordered = [...eps.filter(e => e.id !== 'original'), ...eps.filter(e => e.id === 'original')];   // the star on top
  for (const e of ordered) {
    const m = e.metrics, c = m.croissant_start, p = m.plate_start, orig = e.id === 'original';
    s += `<g class="ep" data-id="${e.id}"><circle class="p" cx="${X(p[0])}" cy="${Y(p[1])}" r="${orig ? r : 3}" fill="none" stroke="#3ddc97" opacity="${orig ? 0.9 : 0.35}"/>`
      + (orig ? `<text x="${X(c[0]) - 5}" y="${Y(c[1]) + 4}" style="fill:#ffd36b;font-size:15px">★</text><circle class="c" cx="${X(c[0])}" cy="${Y(c[1])}" r="9" fill="transparent"/>`
        : `<circle class="c" cx="${X(c[0])}" cy="${Y(c[1])}" r="4" fill="${m.success ? '#ff9a3c' : '#ff5d6c'}" opacity="0.85"/>`) + '</g>';
  }
  svg.innerHTML = s;
  svg.querySelectorAll('.ep').forEach(g => g.onclick = () => loadEpisode(g.dataset.id));
}

function drawMinimapWater() {
  const svg = $('#minimap'), W = 340, H = 260, eps = MAN.runs[state.run].episodes;
  const xs = [-0.2, 0.42], ys = [0.2, 0.68];
  const X = x => 20 + (x - xs[0]) / (xs[1] - xs[0]) * (W - 40), Y = y => H - 20 - (y - ys[0]) / (ys[1] - ys[0]) * (H - 40);
  const sx = 0.48 / (xs[1] - xs[0]) * (W - 40), sy = 0.266 / (ys[1] - ys[0]) * (H - 40), r = 0.078 / (xs[1] - xs[0]) * (W - 40);
  let s = `<rect class="tbl" x="8" y="8" width="${W - 16}" height="${H - 16}" rx="10"/>`
    + `<rect x="${X(0.106) - sx / 2}" y="${Y(0.395) - sy / 2}" width="${sx}" height="${sy}" rx="8" fill="none" stroke="#5b6573" stroke-dasharray="4 3"/>`
    + `<text x="${X(0.106) + sx / 2 - 30}" y="${Y(0.395) - sy / 2 - 5}">sink</text>`
    + `<circle cx="${X(0.111)}" cy="${Y(0.61)}" r="6" fill="#9aa3ad"/><text x="${X(0.111) + 9}" y="${Y(0.61) + 4}">faucet</text>`
    + `<text x="${X(0) - 22}" y="${H - 11}">robot ↓</text>`;
  const ordered = [...eps.filter(e => e.id !== 'original'), ...eps.filter(e => e.id === 'original')];
  for (const e of ordered) {
    const m = e.metrics, p = m.pot_start, orig = e.id === 'original';
    s += `<g class="ep" data-id="${e.id}"><circle class="p" cx="${X(p[0])}" cy="${Y(p[1])}" r="${orig ? r : 3}" fill="none" stroke="#3ddc97" opacity="${orig ? 0.9 : 0.35}"/>`
      + (orig ? `<text x="${X(p[0]) - 5}" y="${Y(p[1]) + 4}" style="fill:#ffd36b;font-size:15px">★</text><circle class="c" cx="${X(p[0])}" cy="${Y(p[1])}" r="9" fill="transparent"/>`
        : `<circle class="c" cx="${X(p[0])}" cy="${Y(p[1])}" r="4" fill="${m.success ? '#ff9a3c' : '#ff5d6c'}" opacity="0.85"/>`) + '</g>';
  }
  svg.innerHTML = s;
  svg.querySelectorAll('.ep').forEach(g => g.onclick = () => loadEpisode(g.dataset.id));
}

function renderResult(m, meta) {
  if (MAN.runs[state.run].task === 'water_flowers') {
    const cell = (k, v, s, cls = '') => `<div class="res-arm"><div class="k">${k}</div><div class="v ${cls}">${v}</div><div class="s">${s}</div></div>`;
    $('#result').innerHTML =
      cell('Task', m.success ? '✓ watering' : '✗ failed', 'head over the pot, lever on', m.success ? 'ok' : 'bad') +
      cell('Head lowered', fmt(m.head_pulled_cm) + ' cm', 'from the dock to over the pot') +
      cell('Over the pot', fmt(m.head_over_pot_cm, 2) + ' cm', 'head to pot axis, at the end') +
      cell('Lever', fmt(m.lever_deg) + '°', 'on from 17.5°') +
      cell('Pot moved', fmt(m.pot_moved_mm) + ' mm', 'never touched') +
      cell('Duration', fmt(meta?.duration_s ?? viewer.duration) + ' s', 'simulated');
    $('#res-meta').textContent = 'MuJoCo · 500 Hz · both arms';
    return;
  }
  const cell = (k, v, s, cls = '') => `<div class="res-arm"><div class="k">${k}</div><div class="v ${cls}">${v}</div><div class="s">${s}</div></div>`;
  $('#result').innerHTML =
    cell('Task', m.success ? '✓ served' : '✗ failed', 'croissant on plate, released', m.success ? 'ok' : 'bad') +
    cell('Lift', fmt(m.lift_cm) + ' cm', 'max height gained') +
    cell('Final error', fmt(m.final_dist_cm, 2) + ' cm', 'to plate centre') +
    cell('In-hand drift', fmt(m.slip_mm) + ' mm', 'while lifted, hand frame') +
    cell('Plate moved', fmt(m.plate_moved_mm) + ' mm', 'never touched') +
    cell('Duration', fmt(meta?.duration_s ?? viewer.duration) + ' s', 'simulated');
  $('#res-meta').textContent = 'MuJoCo · 500 Hz';
}

// ------------------------------------------------------------------ sections
function renderPipeline() {
  const R = SUM.runs.right, sem = R.semantics;
  const tot = Math.max(...sem.subgoals.map(s => (s.window || [0, 0])[1]), 15);
  $('#subgoals').innerHTML = sem.subgoals.map(s => `<div class="sg"><div class="m">${s.motion}<small>${(s.window || []).map(x => x.toFixed(1)).join('–')} s</small></div><div class="g">${s.goal}</div></div>`).join('')
    + `<div class="sgbar">${sem.subgoals.map((s, i) => { const w = s.window || [0, 0]; return `<span style="left:${100 * w[0] / tot}%;width:${100 * (w[1] - w[0]) / tot}%;background:${['#c084fc', '#ffd36b', '#3ddc97'][i % 3]}"></span>`; }).join('')}</div>`;
  $('#objtab').innerHTML = R.objects.map(o => {
    const p = o.params || {};
    const dims = o.id === 'croissant' ? `${fmt(100 * (p.length || 0))} × ${fmt(100 * (p.width || 0))} × ${fmt(100 * (p.height || 0))} cm` : (p.rim_radius ? `⌀ ${fmt(200 * p.rim_radius)} cm, ${fmt(1000 * p.height, 0)} mm rim` : '');
    return `<div class="obj"><span class="iou">IoU ${fmt(o.iou, 2)}</span><b>${o.id.replace('_', ' ')}</b><br><code>${o.skill}</code><br>${dims}</div>`;
  }).join('');
  const r = MAN.runs, d = SUM.dataset || {};
  $('#s4-kpis').innerHTML = [
    ['Verified · right arm', r.right.n_verified_total, 'physics-replayed layouts'],
    ['Verified · left arm', r.left.n_verified_total, 'mirrored, real-rig arm'],
    ['Acceptance', '29 %', 'of drawn layouts pass physics'],
    ['Rendered', d.episodes ?? '—', `${d.frames ?? '—'} frames`],
  ].map(([k, v, s]) => `<div class="kpi"><div class="k">${k}</div><div class="v">${v}</div><div class="s">${s}</div></div>`).join('');
}

function renderAgent() {
  const rows = [['Stage', 'Run', 'Rounds', 'Tool calls', 'Verifier verdicts', 'Time', 'New tools']];
  const rounds = { left: { stage3: 11 } };   // the left run's Stage 3 resumed after a host OOM: 8 + 3 rounds
  for (const [rk, R] of Object.entries(SUM.runs)) {
    for (const [st, p] of Object.entries(R.stages)) {
      const vp = (p.verifier_passes || []).map(x => x ? '<span style="color:var(--ok)">✓</span>' : '<span style="color:var(--bad)">✗</span>').join(' ');
      const nt = (p.new_tools || []).map(t => `<code>${t.name}</code>`).join(' ') || '—';
      rows.push([st.replace('stage', 'Stage '), rk === 'right' ? 'right arm' : 'left arm (mirror)',
        `${rounds[rk]?.[st] ?? p.round} / ${p.max_rounds}`, p.tool_calls ?? '—', vp || '✓', p.elapsed_s ? `${Math.round(p.elapsed_s / 60)} min` : '—', nt]);
    }
  }
  $('#agent-table').innerHTML = rows.map((r, i) => `<tr>${r.map(c => i ? `<td>${c}</td>` : `<th>${c}</th>`).join('')}</tr>`).join('');
  const tu = {};
  for (const R of Object.values(SUM.runs)) for (const p of Object.values(R.stages)) for (const [k, v] of Object.entries(p.tools_used || {})) tu[k] = (tu[k] || 0) + v;
  const top = Object.entries(tu).sort((a, b) => b[1] - a[1]).slice(0, 10).map(([k, v]) => `<code>${k}</code>×${v}`).join(' · ');
  $('#agent-note').innerHTML = `<b>Most used tools:</b> ${top}. A ✗ is a verifier rejection the agent then fixed (e.g. Stage 1's first goal phrasing was not measurable).`;
}

function renderTools() {
  $('#tools').innerHTML = (SUM.new_tools || []).map(t => `<div class="tool"><span class="tc">${t.stage} · ${t.category}</span><div class="tn">${t.name}</div><p>${t.summary}</p></div>`).join('');
}

function renderData() {
  const d = SUM.dataset; if (!d) return;
  $('#ds-kpis').innerHTML = [
    ['Episodes', d.episodes, `${d.distinct_physics_samples} physics samples × 2 looks`],
    ['Frames', (d.frames / 1000).toFixed(1) + 'k', `${fmt(d.hours_at_15fps, 2)} h @ 15 fps`],
    ['Passed checks', `${d.ok}/${d.episodes}`, 'croissant on plate, lifted, finite'],
    ['Final error', fmt(d.final_dist_to_plate_cm_mean, 2) + '<small>cm</small>', 'mean, to plate centre'],
    ['Lift', fmt(d.max_lift_cm_mean, 1) + '<small>cm</small>', 'mean max'],
    ['Action tracking', fmt(d.track_err_arm_deg_p95, 2) + '<small>°</small>', 'p95 |action − next state|'],
  ].map(([k, v, s]) => `<div class="kpi"><div class="k">${k}</div><div class="v">${v}</div><div class="s">${s}</div></div>`).join('');
  $('#clips').innerHTML = (d.clips || []).map(c => `<div class="clip"><div class="pair"><video src="${c.zed}" muted loop playsinline autoplay></video><video src="${c.wrist}" muted loop playsinline autoplay></video></div>
    <div class="cap"><span><b>ZED</b> · wrist</span><span>${c.id.replace('ep_shard_', 'sample ').replace(/_r(\d)$/, ' · look $1')}</span></div></div>`).join('');
  scatter($('#ch-layout'), d.croissant_start, d.plate_start);
  hist($('#ch-lift'), d.croissant_dyaw_deg || d.max_lift_cm, '#ff9a3c', '°');
  hist($('#ch-final'), d.final_dist_cm, '#3ddc97', 'cm');
  hist($('#ch-track'), d.track_err_deg, '#22d3ee', '°');
}

function renderPhoto(P) {
  const box = $('#photo');
  if (!P || !P.clips?.length) { box.outerHTML = '<div class="empty">The Cycles renders are still being produced; this section fills in when they finish.</div>'; return; }
  box.innerHTML = P.clips.map(c => `<div class="clip"><video src="${c.src}" muted loop playsinline autoplay></video><div class="cap"><b>${c.title}</b><span>${c.note || ''}</span></div></div>`).join('');
}

// ------------------------------------------------------------------ tiny charts
function ctx2d(c) {
  const r = c.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  c.width = r.width * dpr; c.height = r.height * dpr; const g = c.getContext('2d'); g.scale(dpr, dpr);
  return [g, r.width, r.height];
}
function axes(g, W, H, xl, xh, yl, yh, xt, yt) {
  g.strokeStyle = '#242a35'; g.fillStyle = '#6b7586'; g.font = '11px Inter, sans-serif'; g.lineWidth = 1;
  const P = { l: 40, r: 12, t: 12, b: 26 };
  const X = x => P.l + (x - xl) / (xh - xl) * (W - P.l - P.r), Y = y => H - P.b - (y - yl) / (yh - yl) * (H - P.t - P.b);
  for (const t of xt) { g.beginPath(); g.moveTo(X(t), P.t); g.lineTo(X(t), H - P.b); g.stroke(); g.fillText(t, X(t) - 8, H - 8); }
  for (const t of yt) { g.beginPath(); g.moveTo(P.l, Y(t)); g.lineTo(W - P.r, Y(t)); g.stroke(); g.fillText(t, 6, Y(t) + 4); }
  return [X, Y];
}
function niceTicks(lo, hi, n = 5) {
  const step = Math.pow(10, Math.floor(Math.log10((hi - lo) / n))); const s = [1, 2, 5, 10].map(m => m * step).find(m => (hi - lo) / m <= n) || step;
  const out = []; for (let v = Math.ceil(lo / s) * s; v <= hi + 1e-9; v += s) out.push(+v.toFixed(6)); return out;
}
function scatter(c, A, B) {
  const [g, W, H] = ctx2d(c);
  const all = [...A, ...B], xs = all.map(p => p[0]), ys = all.map(p => p[1]);
  const xl = Math.min(...xs) - 0.03, xh = Math.max(...xs) + 0.03, yl = Math.min(...ys) - 0.03, yh = Math.max(...ys) + 0.03;
  const [X, Y] = axes(g, W, H, xl, xh, yl, yh, niceTicks(xl, xh), niceTicks(yl, yh));
  for (const [pts, col] of [[B, '#3ddc97'], [A, '#ff9a3c']]) { g.fillStyle = col; g.globalAlpha = 0.75; for (const p of pts) { g.beginPath(); g.arc(X(p[0]), Y(p[1]), 2.6, 0, 7); g.fill(); } }
  g.globalAlpha = 1; g.fillStyle = '#ff9a3c'; g.fillText('● croissant', W - 150, 20); g.fillStyle = '#3ddc97'; g.fillText('● plate', W - 70, 20);
}
function hist(c, vals, col, unit) {
  const [g, W, H] = ctx2d(c);
  if (!vals?.length) return;
  const lo = Math.min(...vals), hi = Math.max(...vals) + 1e-6, nb = 28, w = (hi - lo) / nb;
  const bins = new Array(nb).fill(0); for (const v of vals) bins[Math.min(nb - 1, Math.floor((v - lo) / w))]++;
  const ym = Math.max(...bins);
  const [X, Y] = axes(g, W, H, lo, hi, 0, ym * 1.1, niceTicks(lo, hi), niceTicks(0, ym * 1.1, 4));
  g.fillStyle = col; g.globalAlpha = 0.85;
  bins.forEach((b, i) => { const x0 = X(lo + i * w), x1 = X(lo + (i + 1) * w); g.fillRect(x0 + 0.5, Y(b), Math.max(1, x1 - x0 - 1), Y(0) - Y(b)); });
  g.globalAlpha = 1;
  const mean = vals.reduce((a, b) => a + b, 0) / vals.length;
  g.strokeStyle = '#fff'; g.setLineDash([4, 4]); g.beginPath(); g.moveTo(X(mean), 12); g.lineTo(X(mean), H - 26); g.stroke(); g.setLineDash([]);
  g.fillStyle = '#eef1f6'; g.fillText(`mean ${mean.toFixed(2)} ${unit}`, Math.min(X(mean) + 6, W - 100), 24);
}

main().catch(e => { console.error(e); $('#loading-msg').textContent = 'Failed to load: ' + e.message; });
