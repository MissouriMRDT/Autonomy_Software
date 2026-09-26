// Live per-thread FPS graph for the visualizer.
//
// Polls /api/threads, which returns each AutonomyThread's raw iteration counter and a
// monotonic server timestamp. FPS is the change in the counter divided by the change in
// time between two polls, so it is an exact average over the poll interval.

const POLL_MS = 500;              // How often to sample the counters.
const WINDOW_S = 60;              // How much history the graph shows.
const BELOW_TARGET_RATIO = 0.9;   // A thread under 90% of its IPS limit is flagged.
const COLORS = ['#4dc9f6', '#f67019', '#f53794', '#acc236', '#166a8f', '#00a950', '#8549ba', '#e6c229',
    '#ff6b6b', '#58595b', '#7fdbca', '#c9a0ff', '#ffa94d', '#63e6be', '#91a7ff', '#ff8787'];

const series = new Map();         // id -> { name, max, color, points: [{t, fps}], lastIter, lastT, hidden }
let latestT = 0;
let logScale = null;              // null = auto, true/false = user choice.
let hoverX = null;                // Mouse x in CSS pixels while hovering the graph.
let highlightId = null;           // Thread whose row is hovered.
let nextColor = 0;

const panel = document.getElementById('perf-panel');
const toggle = document.getElementById('perf-toggle');
const summary = document.getElementById('perf-summary');
const content = document.getElementById('perf-content');
const canvas = document.getElementById('perf-canvas');
const table = document.getElementById('perf-table');
const scaleBtn = document.getElementById('perf-scale');
const expandBtn = document.getElementById('perf-expand');
const ctx = canvas.getContext('2d');

// ---------- Data ----------

async function poll() {
    try {
        const response = await fetch('/api/threads');
        if (response.ok) {
            ingest(await response.json());
        } else {
            // The page is served from data/, so it can be newer than the running binary.
            showStatus(response.status === 404 ? 'Not supported by this build. Rebuild Autonomy_Software.' : `Server error ${response.status}`);
        }
    } catch (e) {
        // Server unreachable. Keep the last graph and try again next tick.
        showStatus('Disconnected');
    }
    setTimeout(poll, POLL_MS);
}

function showStatus(text) {
    summary.textContent = text;
    summary.style.color = '#ff6b6b';
}

function ingest(data) {
    const seen = new Set();
    for (const th of data.threads) {
        seen.add(th.id);
        let s = series.get(th.id);
        if (!s) {
            s = { name: th.name, max: th.max, color: COLORS[nextColor++ % COLORS.length], points: [], lastIter: th.iter, lastT: data.t, hidden: false };
            series.set(th.id, s);
            continue;
        }
        const dt = data.t - s.lastT;
        if (dt > 0) {
            s.points.push({ t: data.t, fps: (th.iter - s.lastIter) / dt });
        }
        s.name = th.name;
        s.max = th.max;
        s.lastIter = th.iter;
        s.lastT = data.t;
        while (s.points.length && s.points[0].t < data.t - WINDOW_S) s.points.shift();
    }
    // Threads that were destroyed are no longer in the registry.
    for (const id of series.keys()) if (!seen.has(id)) series.delete(id);
    latestT = data.t;
    render();
}

const currentFps = (s) => (s.points.length ? s.points[s.points.length - 1].fps : null);
const isBelowTarget = (s) => s.max > 0 && currentFps(s) !== null && currentFps(s) < s.max * BELOW_TARGET_RATIO;

// ---------- Rendering ----------

function render() {
    renderSummary();
    if (content.style.display === 'none') return;
    renderTable();
    renderGraph();
}

function renderSummary() {
    if (series.size === 0) {
        showStatus('No threads have been started yet');
        return;
    }
    const below = [...series.values()].filter(isBelowTarget).length;
    summary.textContent = `${series.size} threads` + (below ? ` · ${below} below target` : '');
    summary.style.color = below ? '#ff6b6b' : '#aaa';
}

function renderTable() {
    const rows = [...series.entries()].sort((a, b) => a[1].name.localeCompare(b[1].name));
    table.innerHTML = '';
    for (const [id, s] of rows) {
        const fps = currentFps(s);
        const pct = s.max > 0 && fps !== null ? Math.min(fps / s.max, 1) : null;
        const tr = document.createElement('tr');
        tr.className = s.hidden ? 'perf-hidden' : '';
        tr.title = 'Click to show/hide this thread';
        tr.innerHTML = `
            <td><span class="perf-swatch" style="background:${s.color}"></span></td>
            <td class="perf-name"></td>
            <td class="perf-num" style="color:${isBelowTarget(s) ? '#ff6b6b' : '#fff'}">${fps === null ? '…' : fps.toFixed(fps < 100 ? 1 : 0)}</td>
            <td class="perf-num perf-dim">${s.max > 0 ? s.max : '∞'}</td>
            <td class="perf-bar-cell">${pct === null ? '' : `<div class="perf-bar"><div style="width:${(pct * 100).toFixed(0)}%;background:${isBelowTarget(s) ? '#ff6b6b' : '#00aa00'}"></div></div>`}</td>`;
        tr.querySelector('.perf-name').textContent = s.name;
        tr.addEventListener('click', () => { s.hidden = !s.hidden; render(); });
        tr.addEventListener('mouseenter', () => { highlightId = id; renderGraph(); });
        tr.addEventListener('mouseleave', () => { highlightId = null; renderGraph(); });
        table.appendChild(tr);
    }
}

function niceStep(range, targetTicks) {
    const raw = range / targetTicks;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const norm = raw / mag;
    return (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
}

function renderGraph() {
    // Size the backing store to the displayed size for crisp lines on HiDPI screens.
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
        canvas.width = Math.round(w * dpr);
        canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const visible = [...series.entries()].filter(([, s]) => !s.hidden);
    let yMax = 1, yMin = Infinity;
    for (const [, s] of visible) {
        for (const p of s.points) { yMax = Math.max(yMax, p.fps); if (p.fps > 0) yMin = Math.min(yMin, p.fps); }
        if (s.max > 0) yMax = Math.max(yMax, s.max);
    }
    // Auto: switch to log when one thread runs far faster than the rest.
    const useLog = logScale ?? (yMin !== Infinity && yMax / yMin > 50);
    scaleBtn.textContent = useLog ? 'Log' : 'Linear';

    const pad = { l: 44, r: 10, t: 10, b: 22 };
    const gw = w - pad.l - pad.r, gh = h - pad.t - pad.b;
    const xOf = (t) => pad.l + gw * (1 - (latestT - t) / WINDOW_S);
    let yOf, ticks;
    if (useLog) {
        const lo = 0, hi = Math.ceil(Math.log10(yMax * 1.1));
        yOf = (v) => pad.t + gh * (1 - (Math.log10(Math.max(v, 1)) - lo) / (hi - lo || 1));
        ticks = [];
        for (let e = lo; e <= hi; e++) ticks.push(Math.pow(10, e));
    } else {
        const top = yMax * 1.1, step = niceStep(top, 5);
        yOf = (v) => pad.t + gh * (1 - v / (Math.ceil(top / step) * step));
        ticks = [];
        for (let v = 0; v <= Math.ceil(top / step) * step + 1e-9; v += step) ticks.push(v);
    }

    // Grid and axes.
    ctx.font = '11px sans-serif';
    ctx.strokeStyle = '#333';
    ctx.fillStyle = '#888';
    ctx.lineWidth = 1;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const v of ticks) {
        const y = Math.round(yOf(v)) + 0.5;
        ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
        ctx.fillText(v >= 1000 ? `${v / 1000}k` : `${+v.toFixed(2)}`, pad.l - 6, y);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let s = 0; s <= WINDOW_S; s += 10) {
        ctx.fillText(s === 0 ? 'now' : `-${s}s`, xOf(latestT - s), h - pad.b + 6);
    }

    // One line per visible thread. The hovered row's line is drawn last and thicker.
    const ordered = visible.sort(([a], [b]) => (a === highlightId) - (b === highlightId));
    for (const [id, s] of ordered) {
        if (s.points.length < 2) continue;
        ctx.strokeStyle = s.color;
        ctx.globalAlpha = highlightId === null || highlightId === id ? 1 : 0.25;
        ctx.lineWidth = highlightId === id ? 3 : 1.5;
        ctx.beginPath();
        s.points.forEach((p, i) => (i ? ctx.lineTo(xOf(p.t), yOf(p.fps)) : ctx.moveTo(xOf(p.t), yOf(p.fps))));
        ctx.stroke();
        // Dashed IPS limit for the highlighted thread.
        if (highlightId === id && s.max > 0) {
            ctx.setLineDash([4, 4]);
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(pad.l, yOf(s.max)); ctx.lineTo(w - pad.r, yOf(s.max)); ctx.stroke();
            ctx.setLineDash([]);
        }
    }
    ctx.globalAlpha = 1;

    // Crosshair with the value of every visible thread at the hovered time.
    if (hoverX !== null && hoverX >= pad.l && hoverX <= w - pad.r) {
        const tHover = latestT - WINDOW_S * (1 - (hoverX - pad.l) / gw);
        ctx.strokeStyle = '#aaa';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(hoverX + 0.5, pad.t); ctx.lineTo(hoverX + 0.5, h - pad.b); ctx.stroke();
        const rows = [];
        for (const [, s] of visible) {
            let best = null;
            for (const p of s.points) if (!best || Math.abs(p.t - tHover) < Math.abs(best.t - tHover)) best = p;
            if (best && Math.abs(best.t - tHover) < 1) rows.push({ s, fps: best.fps });
        }
        rows.sort((a, b) => b.fps - a.fps);
        if (rows.length) {
            const lineH = 14, boxW = 190, boxH = rows.length * lineH + 8;
            const bx = hoverX + 10 + boxW > w ? hoverX - 10 - boxW : hoverX + 10;
            const by = Math.max(pad.t, Math.min(pad.t + 4, h - pad.b - boxH));
            ctx.fillStyle = 'rgba(0,0,0,0.85)';
            ctx.fillRect(bx, by, boxW, boxH);
            ctx.textAlign = 'left';
            ctx.textBaseline = 'middle';
            rows.forEach(({ s, fps }, i) => {
                const y = by + 4 + lineH * i + lineH / 2;
                ctx.fillStyle = s.color;
                ctx.fillRect(bx + 6, y - 4, 8, 8);
                ctx.fillStyle = '#fff';
                ctx.fillText(s.name.length > 20 ? s.name.slice(0, 19) + '…' : s.name, bx + 20, y);
                ctx.textAlign = 'right';
                ctx.fillText(fps.toFixed(fps < 100 ? 1 : 0), bx + boxW - 6, y);
                ctx.textAlign = 'left';
            });
        }
    }
}

// ---------- Controls ----------

function setExpanded(open) {
    content.style.display = open ? 'block' : 'none';
    toggle.innerHTML = `Threads ${open ? '&#9660;' : '&#9654;'}`;
    render();
}

function setLarge(large) {
    panel.classList.toggle('perf-large', large);
    expandBtn.innerHTML = large ? '&#10005;' : '&#10530;';
    expandBtn.title = large ? 'Close large view (Esc)' : 'Open large view';
    if (large) setExpanded(true);
    render();
}

toggle.addEventListener('click', () => {
    if (panel.classList.contains('perf-large')) setLarge(false);
    setExpanded(content.style.display === 'none');
});
expandBtn.addEventListener('click', () => setLarge(!panel.classList.contains('perf-large')));
scaleBtn.addEventListener('click', () => {
    logScale = scaleBtn.textContent !== 'Log';
    renderGraph();
});
canvas.addEventListener('mousemove', (e) => { hoverX = e.offsetX; renderGraph(); });
canvas.addEventListener('mouseleave', () => { hoverX = null; renderGraph(); });
window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && panel.classList.contains('perf-large')) setLarge(false); });
window.addEventListener('resize', render);

poll();
