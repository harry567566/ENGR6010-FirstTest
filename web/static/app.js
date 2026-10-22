/* Go2 control panel.
 *
 * Layout: the two views take the room, the buttons sit in a sidebar, and
 * the telemetry is always on screen. The first version put the buttons in
 * the middle and hid the camera and the LiDAR behind a click, which made
 * the whole thing look like a page of documentation rather than a panel.
 *
 * Live telemetry starts by itself as soon as the robot answers a ping, so
 * there is nothing to remember to switch on.
 *
 * Every button comes from control_panel.ACTIONS over /api/actions; this
 * file decides how things look and never what they do.
 */
const { useState, useEffect, useRef, useCallback } = React;
const html = htm.bind(React.createElement);
// every visible string goes through i18n.js; if that file is the one that
// failed to load, fall back to the key so the error box still has words
const t = (...a) => (typeof L !== "undefined" ? L.t(...a) : String(a[0]));

// This window's id. It rides along on the status poll, so the server knows
// a window is still open without any extra traffic, and stops itself once
// the last one is gone.
const CID = Math.random().toString(36).slice(2) + Date.now().toString(36);

const F = (v, d = 2) => (v === undefined || v === null || Number.isNaN(v)
  ? "–" : (+v).toFixed(d));
const DEG = 57.2958;

/* -------------------------------------------------- never fail silently */
/* A blank dark page is the worst failure this panel can have: it looks
 * identical to "the server is down", "the script is stale" and "React
 * threw on the first render". So anything that goes wrong gets written
 * on the page in words, with a button that reloads.
 */
function showFatal(what, detail) {
  const el = document.getElementById("fatal");
  if (!el) return;
  el.style.display = "block";
  el.innerHTML =
    '<b></b><div class="d"></div>' +
    '<button onclick="location.reload()"></button>' +
    '<button onclick="document.getElementById(\'fatal\').style.display=\'none\'">' +
    '</button>';
  const bs = el.querySelectorAll("button");
  el.querySelector("b").textContent = t("fatalHead");
  bs[0].textContent = t("reload");
  bs[1].textContent = t("dismiss");
  el.querySelector(".d").textContent = what + (detail ? "\n\n" + detail : "");
}
// Chrome fires this whenever a ResizeObserver callback resizes its own
// target. It is a warning, not a failure - the browser just reruns the
// callback on the next frame - and showing a red box for it would cry wolf.
const HARMLESS = /ResizeObserver loop/;

window.addEventListener("error", e => {
  if (HARMLESS.test(e.message || "")) return;
  showFatal(e.message || t("scriptErr"), (e.filename || "") + ":" + (e.lineno || ""));
  if (window.__report) window.__report({ jsError: (e.message || "") +
      " @ " + (e.filename || "") + ":" + (e.lineno || "") });
});
window.addEventListener("unhandledrejection", e =>
  showFatal(t("promiseErr"), String(e.reason && (e.reason.stack || e.reason.message)
                                  || e.reason)));

/* Say, once, what this browser actually managed to do. A page that fails
 * to draw cannot be asked what happened afterwards, and the browser the
 * panel runs in is not one I can attach a debugger to. */
function report(extra) {
  let gl = "no", glWhy = "";
  try {
    const c = document.createElement("canvas");
    // the browser explains itself here, and nowhere else
    c.addEventListener("webglcontextcreationerror",
                       ev => { glWhy = String(ev.statusMessage || "").slice(0, 160); });
    gl = (c.getContext("webgl2") || c.getContext("webgl")) ? "yes" : "no";
  } catch (e) { gl = "threw"; glWhy = String(e && e.message || e).slice(0, 160); }
  const body = {
    build: window.BUILD || null, webgl: gl, glWhy,
    size: innerWidth + "x" + innerHeight, dpr: devicePixelRatio,
    rootKids: (document.getElementById("root") || {}).childElementCount,
    ua: navigator.userAgent.slice(0, 110),
    ...extra,
  };
  try {
    fetch("/api/clientlog", { method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body) });
  } catch {}
}
window.__report = report;

class Boundary extends React.Component {
  constructor(p) { super(p); this.state = {}; }
  static getDerivedStateFromError(e) { return { e }; }
  componentDidCatch(e, i) {
    showFatal(String(e && e.message || e), i && i.componentStack);
    report({ crashed: String(e && e.stack || e).slice(0, 400) });
  }
  render() { return this.state.e ? null : this.props.children; }
}

/* Reload when the server is serving newer code than this page is running.
 * Without this, editing app.js leaves an already-open tab on the old
 * version forever, which is indistinguishable from the panel being broken.
 */
function useAutoUpdate() {
  useEffect(() => {
    let alive = true, timer = 0;
    const tick = async () => {
      try {
        const j = await fetch("/api/version").then(r => r.json());
        if (window.BUILD && j.build && j.build !== window.BUILD) {
          location.reload();
          return;
        }
      } catch {}
      if (alive) timer = setTimeout(tick, 10000);
    };
    timer = setTimeout(tick, 10000);
    return () => { alive = false; clearTimeout(timer); };
  }, []);
}

/* ------------------------------------------------------------------ hooks */
function useStatus() {
  const [s, setS] = useState({ iface: "", reachable: false, busy: [] });
  useEffect(() => {
    let alive = true;
    const tick = () => fetch("/api/status?c=" + CID).then(r => r.json())
      .then(j => alive && setS(j)).catch(() => {});
    tick(); const h = setInterval(tick, 2500);
    return () => { alive = false; clearInterval(h); };
  }, []);
  return s;
}

function useLive(reachable) {
  const [s, setS] = useState({});
  const asked = useRef(false);
  useEffect(() => {
    // start on its own once the dog answers: one less thing to forget
    if (reachable && !asked.current) {
      asked.current = true;
      fetch("/api/live/start", { method: "POST" }).catch(() => {});
    }
  }, [reachable]);
  useEffect(() => {
    let alive = true, timer = 0;
    const tick = async () => {
      let on = false;
      try {
        const j = await fetch("/api/live").then(r => r.json());
        if (alive) { setS(j); on = !!j.live; }
      } catch {}
      if (alive) timer = setTimeout(tick, on ? 400 : 1500);
    };
    tick();
    return () => { alive = false; clearTimeout(timer); };
  }, []);
  return s;
}

function useEvents(onLine, onDone) {
  useEffect(() => {
    let es = null, dead = false;

    // Opened only once the page has finished loading. /api/events is a
    // response that never ends, and a request still in flight when the
    // document would otherwise be done keeps the load event from firing -
    // so the tab spins for ever, which looks exactly like a page that
    // cannot load.
    const open = () => {
      if (dead) return;
      es = new EventSource("/api/events");
      es.onmessage = e => {
        let m; try { m = JSON.parse(e.data); } catch { return; }
        if (m.kind === "line") onLine(m);
        else if (m.kind === "done") onDone(m);
      };
    };
    if (document.readyState === "complete") open();
    else window.addEventListener("load", open, { once: true });

    return () => {
      dead = true;
      window.removeEventListener("load", open);
      if (es) es.close();
    };
  }, [onLine, onDone]);
}

/* Tell the server when this page goes away, so closing the tab closes the
 * program. Without it the panel keeps running in the background long after
 * the last window is shut, which is not how an app behaves. */
function useGoodbye() {
  useEffect(() => {
    const bye = () => {
      try {
        navigator.sendBeacon("/api/bye",
          new Blob([JSON.stringify({ c: CID })],
                   { type: "application/json" }));
      } catch {}
    };
    window.addEventListener("pagehide", bye);
    return () => window.removeEventListener("pagehide", bye);
  }, []);
}

/* ------------------------------------------------------------------ views */
function CameraView({ live }) {
  const [src, setSrc] = useState(null);
  const fresh = live.frame_age !== null && live.frame_age !== undefined
                && live.frame_age < 3;
  useEffect(() => {
    let alive = true, url = null, timer = 0;
    // Back off hard when there is nothing to fetch. An unconnected panel
    // used to ask for a frame five times a second forever; with no dog
    // attached every one of those is a 204, and together with the cloud
    // poll and the SSE stream it kept the browser at its connection limit.
    const pull = async () => {
      let got = false;
      try {
        const r = await fetch("/api/frame");
        if (r.status === 200) {
          const b = await r.blob();
          if (url) URL.revokeObjectURL(url);
          url = URL.createObjectURL(b);
          if (alive) setSrc(url);
          got = true;
        }
      } catch {}
      if (alive) timer = setTimeout(pull, got ? 200 : 1500);
    };
    pull();
    return () => { alive = false; clearTimeout(timer);
                   if (url) URL.revokeObjectURL(url); };
  }, []);
  return html`
    <div class="view">
      <div class="vhead"><b>${t("camera")}</b>
        <span>${fresh ? t("nFrames", live.counts ? live.counts.frame : 0)
                      : t("waitFrame")}</span>
      </div>
      <div class="vbody">
        ${src ? html`<img src=${src} />`
              : html`<div class="vnote">
                  ${t("camEmpty")}<br/>${t("camEmpty2")}
                </div>`}
      </div>
    </div>`;
}

function LidarView({ live, lang }) {
  const host = useRef(null);
  const api = useRef(null);
  const [n, setN] = useState(0);
  const [flat, setFlat] = useState(false);   // true when WebGL is unavailable

  useEffect(() => {
    if (!host.current) return;
    api.current = Scene3D.mount(host.current, { pointSize: 0.022 });
    setFlat(!!(api.current && api.current.is2D));
    return () => { if (api.current) api.current.dispose(); api.current = null; };
  }, []);

  useEffect(() => {
    let alive = true, timer = 0;
    const pull = async () => {
      let got = false;
      try {
        const r = await fetch("/api/cloud");
        if (r.status === 200 && api.current) {
          const xyz = new Float32Array(await r.arrayBuffer());
          const c = api.current.setCloud(xyz);
          if (alive) setN(c);
          got = true;
        }
      } catch {}
      if (alive) timer = setTimeout(pull, got ? 300 : 1500);
    };
    pull();
    return () => { alive = false; clearTimeout(timer); };
  }, []);

  const yaw = live.rpy ? live.rpy[2] : 0;
  useEffect(() => {
    if (api.current) api.current.setDogPose(0, 0, yaw);
  }, [yaw]);

  return html`
    <div class="view">
      <div class="vhead"><b>${t("lidar")}</b>
        <span>${n ? t("nPoints", n.toLocaleString()) : t("waitCloud")}</span>
      </div>
      <div class="vbody" ref=${host}></div>
      <div class="vfoot">
        ${flat ? html`<div class="warn">${t("no3D")}</div>` : null}
        ${(() => {
          const how = flat ? t("topDown") : t("orbit");
          if (live.cloud_topic)
            return `${live.cloud_topic}  frame=${live.cloud_frame || "?"}  ·  ${how}`;
          if (n) return how;
          return `${t("cloudHint")}  ·  ${how}`;
        })()}
      </div>
    </div>`;
}

/* -------------------------------------------------------------- telemetry */
function Bars({ vals, max }) {
  const v = (vals && vals.length) ? vals : new Array(12).fill(0);
  return html`<div class="bars">
    ${v.map((x, i) => {
      const u = Math.min(Math.abs(x) / max, 1);
      const c = u > .85 ? "var(--bad)" : u > .6 ? "var(--warn)" : "var(--accent)";
      return html`<i key=${i} title=${F(x)}>
        <span style=${{ height: (u * 100) + "%", background: c }}></span></i>`;
    })}
  </div>`;
}

function Telemetry({ live }) {
  const rpy = live.rpy || [], vel = live.velocity || [];
  const G = (k, v, u) => html`
    <div class="gauge"><div class="k">${k}</div>
      <div class="v">${v}${u ? html`<small> ${u}</small>` : null}</div></div>`;
  return html`
    <div class="tele">
      ${G(t("battery"), live.soc === undefined ? "–" : live.soc, "%")}
      ${G(t("volts"), F(live.power_v, 1), "V")}
      ${G(t("amps"), F(live.power_a, 1), "A")}
      ${G(t("forward"), F(vel[0]), "m/s")}
      ${G(t("height"), F(live.body_height), "m")}
      ${G(t("heading"), F((rpy[2] || 0) * DEG, 0), "°")}
      <div class="gauge" style=${{ gridColumn: "span 3" }}>
        <div class="k">${t("torques")}</div><${Bars} vals=${live.tau} max=${30} />
      </div>
      <div class="gauge" style=${{ gridColumn: "span 3" }}>
        <div class="k">${t("footForce")}</div>
        <${Bars} vals=${live.foot_force} max=${260} />
      </div>
    </div>`;
}

/* ----------------------------------------------------------------- pieces */
function Button({ a, lang, busy, anyBusy, onRun, note }) {
  const w = a[lang] || a.en || {};     // this action's own wording, from ACTIONS
  // Nothing here is ever disabled for being "unsupported": whether the dog
  // answers over this route is the robot's answer to give, not ours.
  return html`
    <button class=${"btn" + (a.motion ? " motion" : "") + (busy ? " busy" : "")}
            disabled=${busy || (anyBusy && !a.motion)}
            onClick=${() => onRun(a.key)}
            title=${(w.what || "") + "\n\n" + (w.why || "")}>
      <span class="lab">${w.label || a.key}</span>
      ${busy ? html`<span class="tag">${t("running")}</span>`
        : note ? html`<span class="tag">${note}</span>`
        : a.motion ? html`<span class="tag motion">${t("moves")}</span>` : null}
    </button>`;
}

function Log({ lines, onClear }) {
  const box = useRef(null); const stick = useRef(true);
  useEffect(() => {
    const el = box.current;
    if (el && stick.current) el.scrollTop = el.scrollHeight;
  }, [lines]);
  return html`
    <div>
      <div class="log" ref=${box} onScroll=${() => {
        const el = box.current; if (!el) return;
        stick.current = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
      }}>
        ${lines.length === 0
          ? html`<span class="l-dim">${t("logEmpty")}</span>`
          : lines.map((l, i) =>
              html`<div key=${i} class=${"l-" + (l.tone || "plain")}>${l.text}</div>`)}
      </div>
      <div class="logbar">
        <button class="mini" onClick=${onClear}>${t("clear")}</button>
        <span>${t("nLines", lines.length)}</span>
      </div>
    </div>`;
}

function RoutePick({ route, setRoute, st }) {
  const on = route === "wifi" ? st.wifi_reachable : st.reachable;
  return html`
    <div>
      <h2>${t("route")}</h2>
      <div class="routes">
        ${["cable", "wifi"].map(r => html`
          <button key=${r} class=${"rbtn" + (route === r ? " sel" : "")}
                  onClick=${() => setRoute(r)}>${t(r)}</button>`)}
      </div>
      <div class="hint">${route === "wifi" ? t("wifiHint") : t("cableHint")}</div>
      <div class="hint">
        <span class=${"dot " + (on ? "on" : "off")}></span>
        ${route === "wifi"
          ? `Mac ${st.mac_wifi_ip || "–"}  →  ${t("dogWifi")} ${st.dog_wifi_ip || "?"}`
          : `Mac ${st.iface || "–"}  →  ${st.robot_ip || "?"}`}
      </div>
    </div>`;
}

function Session() {
  const [s, setS] = useState({});
  const [msg, setMsg] = useState("");
  useEffect(() => {
    const tick = () => fetch("/api/session").then(r => r.json())
      .then(setS).catch(() => {});
    tick(); const h = setInterval(tick, 5000);
    return () => clearInterval(h);
  }, []);
  const go = () => fetch("/api/session/export").then(r => r.json())
    .then(j => setMsg(j.ok ? t("exported", j.path.split("/").pop())
                           : String(j.msg)))
    .catch(e => setMsg(String(e)));
  return html`
    <div>
      <h2>${t("session")}</h2>
      <div class="hint">${t("sessionHint")}</div>
      <div class="hint">
        ${t("nEvents", s.events || 0, s.actions || 0, s.errors || 0)}
      </div>
      <button class="btn" onClick=${go}><span class="lab">${t("exportBtn")}</span></button>
      ${msg ? html`<div class="hint">${msg}</div>` : null}
    </div>`;
}

/* -------------------------------------------------------------------- app */
function App() {
  const [cards, setCards] = useState([]);
  const [lines, setLines] = useState([]);
  // the choice lives in i18n.js so it survives a reload; the state is only
  // here to force a re-render when it changes
  const [lang, setLangState] = useState(L.lang);
  const setLang = next => { L.set(next); setLangState(next); };
  const [running, setRunning] = useState({});
  // the route sticks: a lab session is all cable or all Wi-Fi
  const [route, setRouteState] = useState(() => {
    try { return localStorage.getItem("go2route") || "cable"; }
    catch { return "cable"; }
  });
  const setRoute = r => {
    try { localStorage.setItem("go2route", r); } catch {}
    setRouteState(r);
  };
  const st = useStatus();
  const live = useLive(st.reachable);
  useAutoUpdate();
  useGoodbye();

  useEffect(() => {
    fetch("/api/actions").then(r => r.json()).then(setCards).catch(() => {});
  }, []);

  const onLine = useCallback(m => setLines(ls => {
    const n = ls.concat([{ text: m.text, tone: m.tone }]);
    return n.length > 3000 ? n.slice(-2000) : n;
  }), []);
  const onDone = useCallback(m => setRunning(r => {
    if (!(m.key in r)) return r;
    const n = { ...r }; delete n[m.key]; return n; }), []);
  useEvents(onLine, onDone);

  const run = key => {
    setRunning(r => ({ ...r, [key]: Date.now() }));
    fetch("/api/run", { method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ key, route }) })
      .catch(() => setRunning(r => { const n = { ...r }; delete n[key]; return n; }));
  };

  // The server knows what is actually running; the SSE "done" event is
  // only a hint. Joining the dog's Wi-Fi drops the SSE stream, and any
  // "done" sent while it was down is gone - which used to leave a button
  // reading "running" for ever and, through anyBusy, disable every other
  // button on the panel.
  //
  // The grace period matters: a button is marked running the instant it is
  // pressed, and the server needs a moment before it appears in busy, so a
  // key is only dropped once it has been unaccounted for a few seconds.
  useEffect(() => {
    const live = st.busy || [];
    const now = Date.now();
    setRunning(r => {
      let changed = false;
      const n = {};
      for (const [k, since] of Object.entries(r)) {
        if (live.includes(k) || now - since < 4000) n[k] = since;
        else changed = true;
      }
      return changed ? n : r;
    });
  }, [st]);

  const anyBusy = Object.keys(running).length > 0;
  const ddsOverWifi = st.dds_over_wifi || [];
  // r_net (1-1) has no run script of its own - the server runs
  // mac_net_up.sh or wifi_check.sh for it - so it fails a has_run test and
  // used to be filtered out of the panel entirely. It is the first button
  // anyone needs.
  const acts = cards.flatMap(c => (c.actions || []).filter(
    a => a.where === "robot" && a.key !== "m_stop"
         && (a.has_run || a.key === "r_net")));
  const safe = acts.filter(a => !a.motion);
  const moves = acts.filter(a => a.motion);

  return html`
    <div style=${{ display: "contents" }}>
      <div class="bar">
        <h1>${t("title")}</h1>
        <span class="chip">
          <span class=${"dot " + (st.reachable ? "on" : "off")}></span>
          ${st.reachable
            ? html`${t("connected")} <b>${st.iface}</b>`
            : (st.iface ? t("noAnswer", st.iface) : t("notConnected"))}
        </span>
        ${live.live ? html`<span class="chip">
          ${t("telemetry")} <b>${t("on")}</b></span>` : null}
        ${live.counts ? html`<span class="chip">
          ${t("cState")} <b>${live.counts.low}</b>
          · ${t("cCloud")} <b>${live.counts.cloud}</b>
          · ${t("cFrame")} <b>${live.counts.frame}</b></span>` : null}
        ${live.mode !== undefined
          ? html`<span class="chip">${t("mode")} <b>${live.mode}</b></span>` : null}
        <span class="spacer"></span>
        <button class="mini" onClick=${() => setLang(lang === "zh" ? "en" : "zh")}>
          ${lang === "zh" ? "EN" : "中文"}</button>
      </div>

      <div class="shell">
        <div class="main">
          <div class="views">
            <${CameraView} live=${live} />
            <${LidarView} live=${live} lang=${lang} />
          </div>
          <${Telemetry} live=${live} />
          <${Log} lines=${lines} onClear=${() => setLines([])} />
        </div>

        <div class="side">
          <${RoutePick} route=${route} setRoute=${setRoute} st=${st} />

          <h2>${t("estop")}</h2>
          <button class="btn stop" onClick=${() => run("m_stop")}>
            ${t("stopMove")}</button>
          <div class="hint">${t("estopHint")}</div>

          <h2>${t("safeHead")}</h2>
          ${safe.map(a => html`<${Button} key=${a.key} a=${a} lang=${lang}
            busy=${!!running[a.key]} anyBusy=${anyBusy} onRun=${run}
            note=${route === "wifi" && ddsOverWifi.includes(a.key)
                   ? t("tryDds") : ""} />`)}

          <h2>${t("moveHead")}</h2>
          <div class="hint">${t("moveHint")}</div>
          ${moves.map(a => html`<${Button} key=${a.key} a=${a} lang=${lang}
            busy=${!!running[a.key]} anyBusy=${anyBusy} onRun=${run} />`)}

          <${Session} />

          ${typeof MapView !== "undefined"
            ? html`<h2>${t("recHead")}</h2><${MapView.Panel} lang=${lang} />` : null}
        </div>
      </div>
    </div>`;
}

ReactDOM.createRoot(document.getElementById("root"))
  .render(html`<${Boundary}><${App} /><//>`);

// after the first paint, say whether anything is actually on screen
setTimeout(() => report({
  drew: document.querySelectorAll(".gauge").length > 0,
  canvases: document.querySelectorAll("canvas").length,
  view2D: !!document.querySelector(".vbody canvas:not([data-engine])"),
}), 2500);
