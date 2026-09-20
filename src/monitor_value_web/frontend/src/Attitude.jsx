import { useEffect, useRef } from "react";
import * as THREE from "three";
import { fmt } from "./format.js";

export function AttitudePanel({ attitude }) {
  const wrapRef = useRef(null);
  const canvasRef = useRef(null);
  const st = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setScissorTest(true);
    renderer.setClearColor(0x0a0e13, 1);

    const scene = new THREE.Scene();
    const hemi = new THREE.HemisphereLight(0x9ec9ff, 0x1a222c, 1.1);
    scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 0.6);
    dir.position.set(2, 3, 1);
    scene.add(dir);

    const rosToThree = new THREE.Group();
    rosToThree.rotation.x = -Math.PI / 2;
    scene.add(rosToThree);

    const vehicle = makeRov();
    vehicle.add(new THREE.AxesHelper(0.28));
    rosToThree.add(vehicle);
    scene.add(new THREE.GridHelper(1.6, 8, 0x2a3542, 0x1a222c));

    const gizmo = makeAxisGizmo();
    const gizmoCam = new THREE.OrthographicCamera(-1.15, 1.15, 1.15, -1.15, 0.1, 10);

    const persp = new THREE.PerspectiveCamera(42, 1, 0.05, 20);
    const start = new THREE.Vector3(0.85, 0.55, 0.85);
    const sph = new THREE.Spherical().setFromVector3(start);
    const orbit = { theta: sph.theta, phi: sph.phi, radius: sph.radius };
    applyOrbit(persp, orbit);

    const orthoSize = 0.55;
    const top = new THREE.OrthographicCamera(-orthoSize, orthoSize, orthoSize, -orthoSize, 0.05, 20);
    top.position.set(0, 1.4, 0);
    top.up.set(0, 0, -1);
    top.lookAt(0, 0, 0);

    const side = new THREE.OrthographicCamera(-orthoSize, orthoSize, orthoSize, -orthoSize, 0.05, 20);
    side.position.set(0, 0, 1.4);
    side.lookAt(0, 0, 0);

    const front = new THREE.OrthographicCamera(-orthoSize, orthoSize, orthoSize, -orthoSize, 0.05, 20);
    front.position.set(-1.4, 0, 0);
    front.lookAt(0, 0, 0);

    const drag = { on: false, x: 0, y: 0 };
    st.current = {
      renderer,
      scene,
      vehicle,
      cameras: { persp, top, side, front },
      gizmo,
      gizmoCam,
      wrap,
      orbit,
      drag,
    };

    const onDown = (ev) => {
      if (!in3d(ev, wrap)) return;
      drag.on = true;
      drag.x = ev.clientX;
      drag.y = ev.clientY;
      wrap.setPointerCapture(ev.pointerId);
      wrap.classList.add("orbiting");
    };
    const onMove = (ev) => {
      wrap.style.cursor = in3d(ev, wrap) || drag.on ? "grab" : "default";
      if (!drag.on) return;
      const dx = ev.clientX - drag.x;
      const dy = ev.clientY - drag.y;
      drag.x = ev.clientX;
      drag.y = ev.clientY;
      orbit.theta -= dx * 0.008;
      orbit.phi = clamp(orbit.phi - dy * 0.008, 0.08, Math.PI - 0.08);
      applyOrbit(persp, orbit);
    };
    const onUp = (ev) => {
      if (!drag.on) return;
      drag.on = false;
      try { wrap.releasePointerCapture(ev.pointerId); } catch (_) { /* ignore */ }
      wrap.classList.remove("orbiting");
    };
    const onWheel = (ev) => {
      if (!in3d(ev, wrap)) return;
      ev.preventDefault();
      orbit.radius = clamp(orbit.radius * (ev.deltaY > 0 ? 1.08 : 0.92), 0.45, 4.0);
      applyOrbit(persp, orbit);
    };

    wrap.addEventListener("pointerdown", onDown);
    wrap.addEventListener("pointermove", onMove);
    wrap.addEventListener("pointerup", onUp);
    wrap.addEventListener("pointercancel", onUp);
    wrap.addEventListener("wheel", onWheel, { passive: false });

    let raf = 0;
    const loop = () => {
      raf = requestAnimationFrame(loop);
      renderViews(st.current);
    };
    const ro = new ResizeObserver(() => resize(st.current));
    ro.observe(wrap);
    resize(st.current);
    loop();
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      wrap.removeEventListener("pointerdown", onDown);
      wrap.removeEventListener("pointermove", onMove);
      wrap.removeEventListener("pointerup", onUp);
      wrap.removeEventListener("pointercancel", onUp);
      wrap.removeEventListener("wheel", onWheel);
      renderer.dispose();
    };
  }, []);

  useEffect(() => {
    const v = st.current?.vehicle;
    if (!v || !attitude) return;
    const roll = deg(attitude.roll_deg);
    const pitch = deg(attitude.pitch_deg);
    const yaw = deg(attitude.yaw_deg);
    v.rotation.order = "ZYX";
    v.rotation.set(roll, pitch, yaw);
  }, [attitude]);

  const rpy = `R ${fmt(attitude?.roll_deg, 1)}  P ${fmt(attitude?.pitch_deg, 1)}  Y ${fmt(attitude?.yaw_deg, 1)}`;

  return (
    <div className="card card-fill">
      <h2>Attitude · IMU {attitude?.source === "snapshot" ? "(10 Hz)" : attitude?.source === "monitor" ? "(1 Hz)" : ""}</h2>
      <div ref={wrapRef} className="attitude-grid">
        <canvas ref={canvasRef} style={{ position: "absolute", inset: 0, width: "100%", height: "100%" }} />
        <ViewLabel title="3D · drag" rpy={rpy} />
        <ViewLabel title="Top (yaw)" rpy={rpy} />
        <ViewLabel title="Side (pitch)" rpy={rpy} />
        <ViewLabel title="Front (roll)" rpy={rpy} />
      </div>
    </div>
  );
}

function ViewLabel({ title, rpy }) {
  return (
    <div className="view-box" style={{ background: "transparent", pointerEvents: "none" }}>
      <div className="view-label">{title}</div>
      <div className="view-rpy">{rpy}</div>
    </div>
  );
}

function in3d(ev, wrap) {
  const r = wrap.getBoundingClientRect();
  const x = ev.clientX - r.left;
  const y = ev.clientY - r.top;
  return x >= 0 && y >= 0 && x < r.width / 2 && y < r.height / 2;
}

function applyOrbit(cam, orbit) {
  cam.position.setFromSphericalCoords(orbit.radius, orbit.phi, orbit.theta);
  cam.lookAt(0, 0, 0);
  cam.updateProjectionMatrix();
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function deg(v) {
  if (v === null || v === undefined || Number.isNaN(v)) return 0;
  return (v * Math.PI) / 180;
}

function makeAxisGizmo() {
  const scene = new THREE.Scene();
  // Match main scene: ROS Z-up → Three Y-up.
  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2;
  scene.add(root);

  const len = 0.85;
  const axes = [
    { dir: [1, 0, 0], color: 0xff4d4f, label: "X" },
    { dir: [0, 1, 0], color: 0x3dd68c, label: "Y" },
    { dir: [0, 0, 1], color: 0x4cc3ff, label: "Z" },
  ];
  for (const ax of axes) {
    const geom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(ax.dir[0] * len, ax.dir[1] * len, ax.dir[2] * len),
    ]);
    const line = new THREE.Line(
      geom,
      new THREE.LineBasicMaterial({ color: ax.color, depthTest: false, transparent: true })
    );
    line.renderOrder = 10;
    root.add(line);

    const tip = new THREE.Mesh(
      new THREE.ConeGeometry(0.07, 0.18, 8),
      new THREE.MeshBasicMaterial({ color: ax.color, depthTest: false })
    );
    tip.position.set(ax.dir[0] * len, ax.dir[1] * len, ax.dir[2] * len);
    if (ax.dir[0]) tip.rotation.z = -Math.PI / 2;
    else if (ax.dir[1]) { /* default cone +Y */ }
    else tip.rotation.x = Math.PI / 2;
    tip.renderOrder = 11;
    root.add(tip);

    const spr = axisSprite(ax.label, ax.color);
    spr.position.set(ax.dir[0] * (len + 0.22), ax.dir[1] * (len + 0.22), ax.dir[2] * (len + 0.22));
    spr.renderOrder = 12;
    root.add(spr);
  }
  return scene;
}

function axisSprite(text, color) {
  const canvas = document.createElement("canvas");
  canvas.width = 64;
  canvas.height = 64;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, 64, 64);
  ctx.font = "bold 44px IBM Plex Sans, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.lineWidth = 6;
  ctx.strokeStyle = "#070b10";
  ctx.strokeText(text, 32, 34);
  ctx.fillStyle = `#${color.toString(16).padStart(6, "0")}`;
  ctx.fillText(text, 32, 34);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  const mat = new THREE.SpriteMaterial({
    map: tex,
    depthTest: false,
    depthWrite: false,
    transparent: true,
  });
  const spr = new THREE.Sprite(mat);
  spr.scale.set(0.4, 0.4, 0.4);
  return spr;
}

function syncGizmoCamera(gizmoCam, viewCam) {
  const dir = viewCam.position.clone();
  if (dir.lengthSq() < 1e-8) dir.set(0, 0, 1);
  else dir.normalize();
  gizmoCam.position.copy(dir.multiplyScalar(2.4));
  gizmoCam.up.copy(viewCam.up);
  gizmoCam.lookAt(0, 0, 0);
  gizmoCam.updateProjectionMatrix();
}

function makeRov() {
  const g = new THREE.Group();
  const hull = new THREE.Mesh(
    new THREE.BoxGeometry(0.42, 0.18, 0.14),
    new THREE.MeshStandardMaterial({ color: 0x3d8fd9, metalness: 0.2, roughness: 0.5 })
  );
  g.add(hull);
  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(0.07, 0.14, 12),
    new THREE.MeshStandardMaterial({ color: 0xf5c542 })
  );
  nose.rotation.z = -Math.PI / 2;
  nose.position.x = 0.26;
  g.add(nose);
  const port = new THREE.Mesh(
    new THREE.BoxGeometry(0.08, 0.04, 0.02),
    new THREE.MeshStandardMaterial({ color: 0xff4d4f })
  );
  port.position.set(0.05, 0.11, 0.04);
  g.add(port);
  const stbd = port.clone();
  stbd.material = new THREE.MeshStandardMaterial({ color: 0x3dd68c });
  stbd.position.set(0.05, -0.11, 0.04);
  g.add(stbd);
  return g;
}

function resize(s) {
  if (!s) return;
  const { wrap, renderer, cameras } = s;
  const w = Math.max(1, wrap.clientWidth);
  const h = Math.max(1, wrap.clientHeight);
  renderer.setSize(w, h, false);
  const aspect = w / 2 / Math.max(1, h / 2);
  cameras.persp.aspect = aspect;
  cameras.persp.updateProjectionMatrix();
}

function renderViews(s) {
  if (!s) return;
  const { renderer, scene, cameras, gizmo, gizmoCam, wrap } = s;
  const w = wrap.clientWidth;
  const h = wrap.clientHeight;
  if (w < 2 || h < 2) return;
  const hw = w / 2;
  const hh = h / 2;
  const views = [
    { cam: cameras.persp, x: 0, y: hh, ww: hw, hh },
    { cam: cameras.top, x: hw, y: hh, ww: hw, hh },
    { cam: cameras.side, x: 0, y: 0, ww: hw, hh },
    { cam: cameras.front, x: hw, y: 0, ww: hw, hh },
  ];
  for (const v of views) {
    renderer.setViewport(v.x, v.y, v.ww, v.hh);
    renderer.setScissor(v.x, v.y, v.ww, v.hh);
    renderer.render(scene, v.cam);

    // Orientation gizmo — bottom-left of each pane (WebGL y = bottom).
    const gz = Math.max(36, Math.min(v.ww, v.hh) * 0.32);
    const margin = Math.max(4, gz * 0.06);
    const gx = v.x + margin;
    const gy = v.y + margin;
    syncGizmoCamera(gizmoCam, v.cam);
    renderer.clearDepth();
    renderer.setViewport(gx, gy, gz, gz);
    renderer.setScissor(gx, gy, gz, gz);
    renderer.render(gizmo, gizmoCam);
  }
}
