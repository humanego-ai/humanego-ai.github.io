// HumanEgo · Real2Sim — the photoreal-data and agent-designed-diversity sections (data/quality.json, data/diversity.json)
const $ = s => document.querySelector(s);
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const kpi = (k, v, s) => `<div class="kpi"><div class="k">${k}</div><div class="v">${v}</div>${s ? `<div class="s">${s}</div>` : ''}</div>`;
const pct = (a, b) => b ? Math.round(100 * a / b) : 0;

async function getJSON(p) { try { const r = await fetch(p); return r.ok ? r.json() : null; } catch (e) { return null; } }

function lazyVideos(root) {
  const io = new IntersectionObserver(es => es.forEach(e => {
    const v = e.target;
    if (e.isIntersecting) v.play().catch(() => {}); else v.pause();
  }), { rootMargin: '200px' });
  root.querySelectorAll('video[data-src]').forEach(v => { v.preload = 'metadata'; v.src = v.dataset.src + '#t=0.5'; io.observe(v); });
}

function renderQuality(Q) {
  const box = $('#quality');
  if (!Q) { box.querySelector('.kpis').outerHTML = '<div class="empty">Photoreal renders are being produced.</div>'; return; }
  const k = Q.kpis;
  $('#q-kpis').innerHTML = [
    kpi('Render time', `${k.s_per_frame_view_idle_gpu}<small>s</small>`, `per frame per view (640×480, OptiX denoised); ${k.s_per_frame_view_median} s median on the shared GPU`),
    kpi('Robot alignment', `${k.robot_silhouette_iou}`, 'silhouette IoU vs. the physics state, ZED view'),
    kpi('Depth error', `${k.depth_err_mm_median}<small>mm</small>`, 'median vs. MuJoCo depth from the recorded camera'),
    kpi('Episodes', `${k.ok}/${k.episodes}<small>+${k.ok_left}/${k.episodes_left}</small>`, 'right arm (+ mirrored left arm) pass the dataset checks'),
    kpi('Environments', `${k.hdris}<small>HDRI</small>`, `CC0 indoor rooms + studio · ${k.table_textures} table textures`),
  ].join('');
  $('#q-hero').innerHTML = [['scene', 'Scene mode — real indoor lighting'], ['studio', 'Studio mode — cyclorama and area lights']]
    .map(([m, t]) => `<div class="clip"><video data-src="${Q.hero[m]}" muted loop playsinline preload="none"></video><div class="cap"><b>${t}</b><span>1280×720 showcase render</span></div></div>`).join('');
  $('#q-compare').src = Q.compare;
  $('#q-clips').innerHTML = Q.clips.map(c => `<div class="clip"><div class="pair"><video data-src="${c.zed}" muted loop playsinline preload="none"></video><video data-src="${c.wrist}" muted loop playsinline preload="none"></video></div>
    <div class="cap"><b>${c.mode === 'scene' ? 'Scene' : 'Studio'}</b><span>${esc(c.id.replace('ep_', '').replace(/_(scene|studio)_r\d/, ''))} · ZED + wrist</span></div></div>`).join('');
  $('#q-stills').innerHTML = Q.stills.map(s => `<figure><img src="${s.src}" alt="${esc(s.title)}" loading="lazy"><figcaption>${esc(s.title)}</figcaption></figure>`).join('');
  lazyVideos(box);
}

const AXIS_NAMES = { 'croissant.variant': 'Bread', 'blue_plate.variant': 'Plate', 'table_height_m': 'Table height', 'grasp': 'Grasp', 'tempo': 'Speed' };

function fmtVals(v) {
  if (!Array.isArray(v)) return esc(v);
  if (v.length === 2 && v.every(x => typeof x === 'number')) return `${v[0]} – ${v[1]}`;
  return v.map(x => `<span class="chip">${esc(x)}</span>`).join(' ');
}

function renderDiversity(D) {
  const box = $('#diversity');
  if (!D) { box.querySelector('.kpis').outerHTML = '<div class="empty">The diversity run is still in progress.</div>'; return; }
  if (D.designer) {
    const sent = D.designer.split(/(?<=\.)\s+/);
    $('#d-designer').textContent = sent.slice(0, 2).join(' ') + (sent.length > 2 ? ' …' : '');
    $('#d-designer').title = D.designer;
  }
  const T = D.totals, ds = D.dataset || {};
  const breads = Object.keys(D.acceptance['croissant.variant'] || {}).length;
  $('#d-kpis').innerHTML = [
    kpi('Scene variants', `${T.stage3_ok}/${T.variants}`, 'rebuilt, re-grasped and executed in contact physics'),
    kpi('Verified samples', `${T.samples}`, `${ds.episodes || 2 * T.samples} rendered episodes, ${ds.ok ?? '—'} pass the checks`),
    kpi('Layout draws kept', `${pct(T.stage4_draws.accepted, T.stage4_draws.attempts)}<small>%</small>`, `${T.stage4_draws.accepted}/${T.stage4_draws.attempts} · 29% for the earlier ±7 cm / ±35° run`),
    kpi('Bread types', `${breads}`, 'croissant, crescent, roll, bun, demi-baguette · 9–21 cm, 30–79 g'),
    kpi('Speeds', '0.8–1.3<small>×</small>', 'every speed re-checked against the 1.571 rad/s arm limit'),
  ].join('');
  $('#d-axes').innerHTML = '<tr><th>Axis</th><th>Values</th><th>Why (the designer’s note)</th></tr>' +
    D.axes.map(r => `<tr><td><b>${esc(r.axis.replace(/_m$/, ' (m)').replace(/_/g, ' '))}</b></td><td>${fmtVals(r.values)}</td><td class="why">${esc((r.why || '').slice(0, 230))}${(r.why || '').length > 230 ? '…' : ''}</td></tr>`).join('');
  const ACC = { ...D.acceptance, ...(D.tempo ? { tempo: Object.fromEntries(Object.entries(D.tempo).map(([k, v]) => [`${k}×`, v])) } : {}) };
  $('#d-acc').innerHTML = Object.entries(ACC).filter(([a]) => AXIS_NAMES[a]).map(([a, vals]) => {
    const rows = Object.entries(vals).sort((x, y) => (parseFloat(x[0]) - parseFloat(y[0])) || 0).map(([v, s]) => {
      const n = s.variants ?? s.attempts ?? s.s4_attempts ?? 0, ok = s.stage3_ok ?? s.accepted ?? s.s4_accepted ?? 0;
      return `<div class="acc"><span class="l">${esc(v.replace(/_/g, ' '))}</span><span class="bar"><i style="width:${pct(ok, n)}%"></i></span><span class="n">${ok}/${n}${s.samples != null ? ` · ${s.samples}` : ''}</span></div>`;
    }).join('');
    return `<div class="acc-g"><h5>${AXIS_NAMES[a]}</h5>${rows}</div>`;
  }).join('');
  $('#d-clips').innerHTML = D.clips.map(c => `<div class="clip"><div class="pair"><video data-src="${c.zed}" muted loop playsinline preload="none"></video><video data-src="${c.wrist}" muted loop playsinline preload="none"></video></div>
    <div class="cap col"><b>“${esc(c.instruction)}”</b><span>${esc((c.label || '').replace('croissant=', '').replace('blue_plate=', '').replace(/, grasp /, ' · ').replace(/, /g, ' · '))} · ${c.tempo}× · ${esc(c.table)} · ${c.light_k} K</span></div></div>`).join('');
  $('#d-sheet').src = D.sheet; $('#d-sheet-a').href = D.sheet;
  if (D.hq) {
    $('#d-hq-clips').innerHTML = D.hq.clips.map(c => `<div class="clip"><div class="pair"><video data-src="${c.zed}" muted loop playsinline preload="none"></video><video data-src="${c.wrist}" muted loop playsinline preload="none"></video></div>
      <div class="cap col"><b>“${esc(c.instruction)}”</b><span>photoreal · ZED + wrist</span></div></div>`).join('');
    $('#d-hq-stills').innerHTML = D.hq.stills.map(s => `<figure><img src="${s.src}" alt="${esc(s.title)}" loading="lazy"><figcaption>${esc(s.title)}</figcaption></figure>`).join('');
    $('#d-hq-grid').src = D.hq.grid; $('#d-hq-grid-a').href = D.hq.grid;
    if (D.hq.dataset) $('#d-hq-note').textContent += ` ${D.hq.dataset.ok}/${D.hq.dataset.episodes} episodes pass the dataset checks.`;
  } else { ['#d-hq-clips', '#d-hq-stills'].forEach(q => $(q).remove()); }
  lazyVideos(box);
}

function renderEfficiency(E) {
  if (!E) { $('#efficiency').style.display = 'none'; return; }
  const a = E.acceptance, s = E.s_per_sample, u = E.usd_per_1000_samples;
  $('#e-kpis').innerHTML = [
    kpi('CPU time per sample', `${s.after}<small>s</small>`, `was ${s.before} s · ${(s.before / s.after).toFixed(1)}× less, end to end per verified sample`),
    kpi('Layouts kept', `${Math.round(100 * a.after.rate)}<small>%</small>`, `was ${Math.round(100 * a.before.rate)}% (${a.before.accepted}/${a.before.attempts}) · now ${a.after.accepted}/${a.after.attempts}`),
    kpi('Physics rejections', `${Math.round(100 * a.after.physics_rejected / a.after.attempts)}<small>%</small>`, `of attempts · was ${Math.round(100 * a.before.physics_rejected / a.before.attempts)}%`),
    kpi('Compute per 1000 samples', `$${u.after}`, `was $${u.before} · at $${u.vcpu_hour_usd}/vCPU-h`),
    kpi('Memory per process', '2.5<small>GiB</small>', 'was 5.1 GiB · 11–12 shards fit in 30 GB, was 5'),
  ].join('');
  $('#e-table').innerHTML = '<tr><th>What</th><th>Before</th><th>After</th><th>How</th></tr>' +
    E.rows.map(r => `<tr><td><b>${esc(r.what)}</b></td><td>${esc(r.before)}</td><td><b>${esc(r.after)}</b></td><td class="why">${esc(r.how)}</td></tr>`).join('');
  $('#e-note').textContent = 'Equivalence: ' + E.equivalence + '.';
}

function renderSoft(S) {
  if (!S) { ['#soft', '#s-kpis', '#s-videos', '#s-table', '#s-stills'].forEach(q => $(q)?.closest('.card')?.remove() ?? $(q)?.remove()); return; }
  const T = S.totals, row = b => S.calibration.find(r => r.bread === b) || {};
  $('#s-kpis').innerHTML = [
    kpi('Soft samples', `${T.samples}`, `${T.stage3_ok}/${T.variants} soft variants verified · ${T.stage4_draws.accepted}/${T.stage4_draws.attempts} layout draws kept`),
    kpi('Brioche bun', `${row('brioche').grip_sink_mm}<small>mm</small>`, 'fingertip sink under the grip'),
    kpi('Croissant', `${row('croissant').grip_sink_mm}<small>mm</small>`, 'fingertip sink under the grip'),
    kpi('Demi-baguette', `${row('baguette').grip_sink_mm}<small>mm</small>`, 'crust: gives the least'),
    kpi('Rigid (before)', `${row('rigid (before)').grip_sink_mm}<small>mm</small>`, 'the bread did not give at all'),
  ].join('');
  $('#s-videos').innerHTML = S.videos.map(v => `<div class="clip"><video data-src="${v.src}" muted loop playsinline preload="none"></video><div class="cap"><b>${esc(v.title)}</b><span>1280×720 · dent = recorded penetration</span></div></div>`).join('');
  $('#s-table').innerHTML = '<tr><th>Bread</th><th>Softness</th><th>Grip force</th><th>Fingertip sink</th><th>Deepest</th><th>Floor press at 2 N</th></tr>' +
    S.calibration.map(r => `<tr><td><b>${esc(r.bread)}</b></td><td>${r.softness ?? '—'}</td><td>${r.grip_force_n} N</td><td><b>${r.grip_sink_mm} mm</b></td><td>${r.max_sink_mm} mm</td><td>${r.squeeze_2n_mm} mm</td></tr>`).join('');
  $('#s-stills').innerHTML = S.stills.map(s => `<figure><img src="${s.src}" alt="${esc(s.title)}" loading="lazy"><figcaption>${esc(s.title)}</figcaption></figure>`).join('');
  lazyVideos($('#s-videos'));
}

function renderWater(W) {
  const box = $('#water');
  if (!W) { box.style.display = 'none'; return; }
  $('#w-human').dataset.src = W.media.human; if (W.media.s3) $('#w-s3').dataset.src = W.media.s3; $('#w-s2').src = W.media.s2;
  const col = { left: '#f472b6', right: '#facc15', both: '#a3e635' };
  $('#w-subgoals').innerHTML = W.subgoals.map(g => `<div class="sg"><div class="m" style="color:${col[g.hand] || 'var(--txt)'}">${esc(g.hand)} · ${esc(g.motion)}<small>${(g.window || []).map(x => (+x).toFixed(1)).join('–')} s</small></div><div class="g">${esc(g.goal)}</div></div>`).join('');
  $('#w-objects').innerHTML = W.objects.map(o => `<b>${esc(o.id.replace(/_/g, ' '))}</b> (${esc(o.type)}, ${esc(o.role)})`).join(' · ');
  $('#w-sessions').innerHTML = '<tr><th>#</th><th>Stage</th><th>Rounds</th><th>Result</th><th>What happened</th></tr>' + W.sessions.map((r, i) => {
    const cls = /passed/.test(r.result) ? 'color:var(--ok)' : /blocked/.test(r.result) ? 'color:#ffb35c' : 'color:#7cc4ff';
    return `<tr><td>${i + 1}</td><td><b>${esc(r.stage)}</b></td><td>${r.rounds}</td><td style="${cls};font-weight:600">${esc(r.result)}</td><td class="why">${esc(r.note)}</td></tr>`; }).join('');
  $('#w-tools').innerHTML = W.tools.map(t => `<div class="tool"><span class="tc">${esc(t.type)}</span><div class="tn">${esc(t.name)}</div><p>${esc(t.summary)}</p></div>`).join('');
  const a = W.stage4 || {}, d = W.dataset || {};
  $('#w-kpis').innerHTML = [
    kpi('Verified layouts', `${a.accepted ?? '—'}`, `of ${a.attempts ?? '—'} drawn (${Math.round(100 * (a.rate || 0))}%), each re-simulated`),
    kpi('CPU per sample', `${a.s_per_sample ?? '—'}<small>s</small>`, 'bimanual, 24 s episodes'),
    kpi('Rendered episodes', `${d.ok ?? '—'}/${d.episodes ?? '—'}`, 'pass the dataset checks'),
    kpi('Frames', d.frames ? `${(d.frames / 1000).toFixed(1)}k` : '—', 'ZED + 2 wrists + third-person, with depth'),
  ].join('');
  $('#w-clips').innerHTML = (W.clips || []).map(c => `<div class="clip"><div class="quad">${['ego', 'tp0', 'wrist_left', 'wrist_right'].filter(v => c[v]).map(v => `<video data-src="${c[v]}" muted loop playsinline preload="none"></video>`).join('')}</div>
    <div class="cap"><b>${esc(c.id.replace('ep_', '').replace(/_r\d$/, ''))}</b><span>ZED · third-person · left wrist · right wrist</span></div></div>`).join('');
  lazyVideos(box);
}

const [Q, D, E, S, W] = await Promise.all([getJSON('data/quality.json'), getJSON('data/diversity.json'), getJSON('data/efficiency.json'), getJSON('data/soft.json'), getJSON('data/water.json')]);
renderWater(W); renderQuality(Q); renderDiversity(D); renderEfficiency(E); renderSoft(S);
