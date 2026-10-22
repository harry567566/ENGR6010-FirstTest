/* Recorded surveys and the maps built from them.
 *
 * Reuses the same raw-float32 path the live cloud uses: /api/map/<name>
 * returns the assembled map as xyz and it goes straight into a GPU buffer.
 * A 300k-point building map draws at the same cost as a 25k-point sweep.
 *
 * The robot's path is drawn as a line through the same scene, so a map that
 * drifted is obvious - the path doubles back through its own walls.
 */
const MapView = (() => {
  const { useState, useEffect, useRef } = React;
  const html = htm.bind(React.createElement);
  const t = (...a) => L.t(...a);

  function Scene({ name }) {
    const host = useRef(null), api = useRef(null);
    const [info, setInfo] = useState("");
    const [flat, setFlat] = useState(false);

    useEffect(() => {
      if (!host.current) return;
      // the same scene the live view uses, without the robot model: a built
      // map is a building, not a snapshot from where the dog is standing
      api.current = Scene3D.mount(host.current,
        { dog: false, dist: 26, pitch: 0.9, pointSize: 0.05, maxPoints: 300000 });
      if (api.current && api.current.is2D) setFlat(true);
      return () => { if (api.current) api.current.dispose(); api.current = null; };
    }, []);

    useEffect(() => {
      if (!name || !api.current) return;
      let dead = false;
      (async () => {
        const r = await fetch("/api/map/" + name);
        if (r.status !== 200 || dead) { setInfo(t("notMapped")); return; }
        const xyz = new Float32Array(await r.arrayBuffer());
        const n = api.current.setCloud(xyz);
        const pr = await fetch("/api/path/" + name).then(x => x.json());
        const P = pr.path || [];
        api.current.setPath(P);
        // frame the whole building
        let sx = 0, sy = 0;
        for (let i = 0; i < n; i++) { sx += xyz[i*3]; sy += xyz[i*3+1]; }
        const cx = sx / n, cy = sy / n;
        let rad = 1;
        for (let i = 0; i < n; i += Math.max(1, (n / 2000) | 0))
          rad = Math.max(rad, Math.hypot(xyz[i*3] - cx, xyz[i*3+1] - cy));
        api.current.frame(cx, cy, rad);
        if (!dead) setInfo(t("mapInfo", n.toLocaleString(), P.length));
      })();
      return () => { dead = true; };
    }, [name]);

    return html`
      <div>
        <div ref=${host} style=${{ width: "100%", height: 230, borderRadius: 9,
          overflow: "hidden", border: "1px solid var(--line)" }}></div>
        ${flat ? html`<div class="hint warn" style=${{ marginTop: 5 }}>
          ${t("no3D")}</div>` : null}
        <div class="hint" style=${{ marginTop: 5 }}>
          ${info || t("pickSurvey")} · ${t("redLine")}
        </div>
      </div>`;
  }

  function Panel({ lang }) {         // lang is only here to force a redraw
    const [list, setList] = useState([]);
    const [sel, setSel] = useState(null);
    useEffect(() => {
      const tick = () => fetch("/api/surveys").then(r => r.json())
        .then(setList).catch(() => {});
      tick();
      const h = setInterval(tick, 8000);
      return () => clearInterval(h);
    }, []);

    return html`
      <div>
        ${list.length === 0
          ? html`<div class="hint">${t("noSurvey")}</div>` : null}
        ${list.map(s => html`
          <button key=${s.name} class=${"btn" + (sel === s.name ? " busy" : "")}
                  onClick=${() => setSel(s.name)}>
            <div class="lab">
              <span>${s.name.replace("survey_", "")}</span>
              ${s.has_map ? html`<span class="tag safe">${t("mapped")}</span>`
                          : html`<span class="tag">${t("rawOnly")}</span>`}
            </div>
            <div class="hint" style=${{margin:0}}>
              ${s.seconds ? t("seconds", (+s.seconds).toFixed(0)) : ""}
              ${s.counts && s.counts.cloud ? " · " + t("cloudFrames", s.counts.cloud) : ""}
              ${s.counts && s.counts.frame ? " · " + t("photos", s.counts.frame) : ""}
              ${s.distance_m != null ? " · " + t("walked", s.distance_m) : ""}
              ${s.map_points
                ? " · " + t("mapPoints", (+s.map_points).toLocaleString()) : ""}
              ${s.had_odometry === false ? " · " + t("icpOnly") : ""}
              ${s.had_odometry === true ? " · " + t("usedOdom") : ""}
            </div>
          </button>`)}
        ${sel ? html`<div style=${{ marginTop: 10 }}>
          <${Scene} name=${sel} key=${lang} /></div>` : null}
      </div>`;
  }

  return { Panel };
})();
