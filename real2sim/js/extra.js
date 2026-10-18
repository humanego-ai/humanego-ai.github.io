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

const [Q, D, E] = await Promise.all([getJSON('data/quality.json'), getJSON('data/diversity.json'), getJSON('data/efficiency.json')]);
renderQuality(Q); renderDiversity(D); renderEfficiency(E);
