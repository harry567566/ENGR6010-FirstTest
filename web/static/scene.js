/* The 3D view: a Go2 body, a ground grid, the live LiDAR sweep and the path.
 *
 * It draws the robot and the grid immediately, before any data arrives, so
 * the panel looks like an instrument rather than an empty box. The point
 * cloud is added on top when the dog starts sending one.
 *
 * Point clouds arrive from /api/cloud as raw float32 xyz and go straight
 * into a GPU buffer - no parsing in JavaScript.
 */
const Scene3D = (() => {

  const GO2 = { L: 0.70, W: 0.31, H: 0.19, legR: 0.022, stand: 0.33 };

  function buildDog() {
    const g = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(GO2.L, GO2.W, GO2.H),
      new THREE.MeshLambertMaterial({ color: 0x2f3745 }));
    body.position.z = GO2.stand;
    g.add(body);

    // the head, so "forward" is unmistakable
    const head = new THREE.Mesh(
      new THREE.BoxGeometry(0.1, 0.18, 0.12),
      new THREE.MeshLambertMaterial({ color: 0x3d4859 }));
    head.position.set(GO2.L / 2 + 0.04, 0, GO2.stand + 0.03);
    g.add(head);

    // the LiDAR puck on top of the head, where the sweep comes from
    const lid = new THREE.Mesh(
      new THREE.CylinderGeometry(0.035, 0.035, 0.045, 16),
      new THREE.MeshLambertMaterial({ color: 0x4da3ff }));
    lid.rotation.x = Math.PI / 2;
    lid.position.set(GO2.L / 2 + 0.02, 0, GO2.stand + 0.13);
    g.add(lid);

    const legMat = new THREE.MeshLambertMaterial({ color: 0x252c38 });
    for (const sx of [1, -1]) for (const sy of [1, -1]) {
      const leg = new THREE.Mesh(
        new THREE.CylinderGeometry(GO2.legR, GO2.legR, GO2.stand, 8), legMat);
      leg.position.set(sx * GO2.L * 0.36, sy * GO2.W * 0.56, GO2.stand / 2);
      g.add(leg);
    }
    // a short forward arrow, the same idea as the heading arrow on the map
    const dir = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(GO2.L / 2, 0, GO2.stand), 0.45, 0xff5f56, 0.13, 0.08);
    g.add(dir);
    return g;
  }

  /* A 2D top-down fallback, for a browser with no working WebGL.
   *
   * new THREE.WebGLRenderer() THROWS when a context cannot be created -
   * hardware acceleration switched off, the GPU process dead, a remote
   * desktop session, a blocklisted driver. Thrown from inside a React
   * effect that takes down the whole tree, which is a black page with the
   * right title and no error anywhere. A plain canvas is worth far more
   * than that, so the view degrades instead of failing. */
  function mount2D(el, opts = {}) {
    const cv = document.createElement("canvas");
    cv.style.width = "100%"; cv.style.height = "100%";
    el.innerHTML = ""; el.appendChild(cv);
    // No notice is drawn here. An absolutely positioned label inevitably
    // lands on top of whatever else the view shows; the caller puts the
    // "no WebGL" line in its own footer, where it is in normal flow.
    let pts = null, dogYaw = 0, path = null;
    let raf = 0, lastW = 0, lastH = 0;

    /* Draw on the next frame, never straight from the ResizeObserver.
     * Resizing the canvas from inside the observer's own callback makes
     * the observer fire again, which Chrome reports as "ResizeObserver
     * loop completed with undelivered notifications" and which burns a
     * core redrawing the same picture. */
    const draw = () => { if (!raf) raf = requestAnimationFrame(render); };

    const render = () => {
      raf = 0;
      const W = el.clientWidth || 400, H = el.clientHeight || 300;
      const r = Math.min(devicePixelRatio || 1, 2);
      if (W !== lastW || H !== lastH) {        // only when it really changed
        cv.width = W * r; cv.height = H * r;
        lastW = W; lastH = H;
      }
      const g = cv.getContext("2d");
      if (!g) return;
      g.setTransform(r, 0, 0, r, 0, 0);
      g.fillStyle = "#07090d"; g.fillRect(0, 0, W, H);
      let span = 6;
      if (pts && pts.length) {
        let m = 1;
        for (let i = 0; i < pts.length; i += 3)
          m = Math.max(m, Math.abs(pts[i]), Math.abs(pts[i + 1]));
        span = m * 1.1;
      }
      const s = Math.min(W, H) / (2 * span), ox = W / 2, oy = H / 2;
      g.strokeStyle = "#1d232d"; g.lineWidth = 1;          // a 1 m grid
      for (let v = -Math.ceil(span); v <= span; v++) {
        g.beginPath(); g.moveTo(ox + v * s, 0); g.lineTo(ox + v * s, H); g.stroke();
        g.beginPath(); g.moveTo(0, oy + v * s); g.lineTo(W, oy + v * s); g.stroke();
      }
      if (pts) {
        let zmin = 1e9, zmax = -1e9;
        for (let i = 2; i < pts.length; i += 3) {
          if (pts[i] < zmin) zmin = pts[i];
          if (pts[i] > zmax) zmax = pts[i];
        }
        const zs = Math.max(zmax - zmin, 0.4);
        for (let i = 0; i < pts.length; i += 3) {
          const u = (pts[i + 2] - zmin) / zs;
          g.fillStyle = `rgb(${56 + 199 * u | 0},${140 + 89 * Math.sin(u * 3.14) | 0},${255 - 184 * u | 0})`;
          g.fillRect(ox + pts[i] * s, oy - pts[i + 1] * s, 1.6, 1.6);
        }
      }
      if (path && path.length > 1) {                        // where it walked
        g.strokeStyle = "#ff5f56"; g.lineWidth = 1.5; g.beginPath();
        path.forEach((q, i) => i ? g.lineTo(ox + q[0] * s, oy - q[1] * s)
                                 : g.moveTo(ox + q[0] * s, oy - q[1] * s));
        g.stroke();
      }
      if (opts.dog !== false) {                             // the robot
        g.save(); g.translate(ox, oy); g.rotate(-dogYaw);
        g.fillStyle = "#2f3745";
        g.fillRect(-GO2.L / 2 * s, -GO2.W / 2 * s, GO2.L * s, GO2.W * s);
        g.strokeStyle = "#ff5f56"; g.lineWidth = 2; g.beginPath();
        g.moveTo(GO2.L / 2 * s, 0); g.lineTo((GO2.L / 2 + 0.4) * s, 0); g.stroke();
        g.restore();
      }
    };
    const ro = new ResizeObserver(draw); ro.observe(el);
    draw();
    return {
      setCloud(xyz) { pts = xyz; draw(); return xyz.length / 3; },
      setPath(P) { path = P; draw(); },
      setDogPose(x, y, yawRad) { dogYaw = yawRad || 0; draw(); },
      frame() { draw(); },
      dispose() { cancelAnimationFrame(raf); ro.disconnect(); el.innerHTML = ""; },
      is2D: true,
    };
  }

  /** Attach a scene to `el`. Returns handles the caller drives. */
  function mount(el, opts = {}) {
    if (typeof THREE === "undefined") return mount2D(el, opts);
    const w = el.clientWidth || 600, h = el.clientHeight || 400;
    const scene = new THREE.Scene();
    scene.fog = new THREE.Fog(0x07090d, 9, 30);
    const cam = new THREE.PerspectiveCamera(55, w / h, 0.05, 400);
    cam.up.set(0, 0, 1);
    let ren;
    try {
      ren = new THREE.WebGLRenderer({ antialias: true });
      if (!ren.getContext()) throw new Error("no WebGL context");
    } catch (e) {
      return mount2D(el, opts);          // never let this take the page down
    }
    ren.setPixelRatio(Math.min(devicePixelRatio, 2));
    ren.setSize(w, h);
    ren.setClearColor(0x07090d, 1);
    el.innerHTML = "";
    el.appendChild(ren.domElement);

    scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const key = new THREE.DirectionalLight(0xffffff, 0.75);
    key.position.set(3, -4, 7);
    scene.add(key);

    const grid = new THREE.GridHelper(20, 40, 0x39414f, 0x1d232d);
    grid.rotateX(Math.PI / 2);
    scene.add(grid);

    const dog = opts.dog === false ? null : buildDog();
    if (dog) scene.add(dog);

    const MAX = opts.maxPoints || 120000;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position",
      new THREE.BufferAttribute(new Float32Array(MAX * 3), 3));
    geo.setAttribute("color",
      new THREE.BufferAttribute(new Float32Array(MAX * 3), 3));
    geo.setDrawRange(0, 0);
    const cloud = new THREE.Points(geo, new THREE.PointsMaterial(
      { size: opts.pointSize || 0.03, vertexColors: true, sizeAttenuation: true }));
    scene.add(cloud);

    let pathLine = null;

    // orbit, kept small instead of pulling in OrbitControls
    let yaw = opts.yaw ?? -0.72, pitch = opts.pitch ?? 0.30,
        dist = opts.dist ?? 2.6, cx = 0, cy = 0, cz = opts.cz ?? 0.26;
    const place = () => {
      cam.position.set(cx + dist * Math.cos(pitch) * Math.sin(yaw),
                       cy - dist * Math.cos(pitch) * Math.cos(yaw),
                       cz + dist * Math.sin(pitch));
      cam.lookAt(cx, cy, cz);
    };
    place();
    const d = ren.domElement;
    let drag = false, px = 0, py = 0;
    d.style.cursor = "grab";
    d.onpointerdown = e => { drag = true; px = e.clientX; py = e.clientY;
                             d.style.cursor = "grabbing"; };
    d.onpointerup = d.onpointerleave = () => { drag = false; d.style.cursor = "grab"; };
    d.onpointermove = e => {
      if (!drag) return;
      yaw -= (e.clientX - px) * 0.008;
      pitch = Math.max(-0.2, Math.min(1.45, pitch + (e.clientY - py) * 0.008));
      px = e.clientX; py = e.clientY; place();
    };
    d.onwheel = e => {
      e.preventDefault();
      dist = Math.max(0.8, Math.min(140, dist * (1 + e.deltaY * 0.0012)));
      place();
    };

    const ro = new ResizeObserver(() => {
      const W = el.clientWidth, H = el.clientHeight;
      if (!W || !H) return;
      cam.aspect = W / H; cam.updateProjectionMatrix(); ren.setSize(W, H);
    });
    ro.observe(el);

    let raf = 0;
    const loop = () => { ren.render(scene, cam); raf = requestAnimationFrame(loop); };
    loop();

    return {
      /** xyz Float32Array -> the GPU buffer, coloured by height. */
      setCloud(xyz) {
        const n = Math.min(xyz.length / 3, MAX);
        const pos = geo.attributes.position.array;
        const col = geo.attributes.color.array;
        let zmin = 1e9, zmax = -1e9;
        for (let i = 0; i < n; i++) {
          const z = xyz[i * 3 + 2];
          if (z < zmin) zmin = z; if (z > zmax) zmax = z;
        }
        const span = Math.max(zmax - zmin, 0.4);
        for (let i = 0; i < n * 3; i += 3) {
          pos[i] = xyz[i]; pos[i + 1] = xyz[i + 1]; pos[i + 2] = xyz[i + 2];
          const u = (xyz[i + 2] - zmin) / span;
          col[i]     = 0.22 + 0.78 * u;
          col[i + 1] = 0.55 + 0.35 * Math.sin(u * 3.14159);
          col[i + 2] = 1.00 - 0.72 * u;
        }
        geo.attributes.position.needsUpdate = true;
        geo.attributes.color.needsUpdate = true;
        geo.setDrawRange(0, n);
        return n;
      },
      setPath(P) {
        if (pathLine) { scene.remove(pathLine); pathLine.geometry.dispose(); }
        if (!P || P.length < 2) { pathLine = null; return; }
        const g = new THREE.BufferGeometry().setFromPoints(
          P.map(p => new THREE.Vector3(p[0], p[1], (p[2] || 0) + 0.04)));
        pathLine = new THREE.Line(g, new THREE.LineBasicMaterial(
          { color: 0xff5f56 }));
        scene.add(pathLine);
      },
      setDogPose(x, y, yawRad) {
        if (!dog) return;
        dog.position.set(x || 0, y || 0, 0);
        dog.rotation.z = yawRad || 0;
      },
      frame(cxN, cyN, radius) {
        cx = cxN; cy = cyN; cz = 0.3;
        dist = Math.max(2.5, radius * 2.1); place();
      },
      dispose() { cancelAnimationFrame(raf); ro.disconnect(); ren.dispose();
                  el.innerHTML = ""; },
    };
  }

  return { mount, mount2D };
})();
