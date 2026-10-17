// HumanEgo Real2Sim — three.js replay of physics-verified DexAgent episodes on the Tianji + SharpaWave rig.
// The model JSON is the compiled MuJoCo kinematic tree (bodies, joints, visual geoms, cameras); the browser runs forward
// kinematics from the recorded qpos, so what you see is exactly the simulated state.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const V = new THREE.Vector3(), Q = new THREE.Quaternion(), M = new THREE.Matrix4(), M2 = new THREE.Matrix4();
const ONE = new THREE.Vector3(1, 1, 1);

function mjQuat(q) { return new THREE.Quaternion(q[1], q[2], q[3], q[0]); }   // MuJoCo wxyz -> three xyzw

export class Viewer {
  constructor(container) {
    this.container = container;
    this.loader = new GLTFLoader();
    this.cache = new Map();
    const r = this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    r.outputColorSpace = THREE.SRGBColorSpace;
    r.toneMapping = THREE.NeutralToneMapping;
    r.toneMappingExposure = 1.05;
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.VSMShadowMap;
    r.setClearColor(0x000000, 0);
    container.appendChild(r.domElement);
    r.domElement.style.background = 'radial-gradient(120% 95% at 50% 38%, #d6d4cf 0%, #bdbbb6 46%, #9c9a95 100%)';

    const scene = this.scene = new THREE.Scene();
    const pmrem = new THREE.PMREMGenerator(r);
    scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    scene.environmentIntensity = 0.6;

    const cam = this.camera = new THREE.PerspectiveCamera(38, 16 / 9, 0.02, 30);
    cam.up.set(0, 0, 1);
    this.controls = new OrbitControls(cam, r.domElement);
    Object.assign(this.controls, { enableDamping: true, dampingFactor: 0.07, minDistance: 0.25, maxDistance: 4.5 });
    this.controls.maxPolarAngle = Math.PI * 0.53;
    this.controls.addEventListener('start', () => { if (this.mode !== 'orbit') this.setCamera('orbit', true); this.autoOrbit = false; this.onUserCamera?.(); });

    scene.add(new THREE.HemisphereLight(0xfaf7f0, 0x8a8781, 0.55));
    const key = this.key = new THREE.DirectionalLight(0xfff6ea, 2.2);
    key.position.set(1.2, -0.6, 2.4); key.target.position.set(0, 0.45, 0);
    key.castShadow = true; key.shadow.mapSize.set(4096, 4096);
    const sc = key.shadow.camera; sc.left = -1.2; sc.right = 1.2; sc.top = 1.2; sc.bottom = -1.2; sc.near = 0.4; sc.far = 6;
    key.shadow.bias = -0.0004; key.shadow.radius = 8; key.shadow.blurSamples = 20;
    scene.add(key, key.target);
    const rim = new THREE.DirectionalLight(0xdfe8ff, 0.6); rim.position.set(-1.6, 1.8, 1.6); scene.add(rim);

    // studio floor under the table
    const fc = document.createElement('canvas'); fc.width = fc.height = 512;
    const g = fc.getContext('2d'); const grd = g.createRadialGradient(256, 256, 40, 256, 256, 256);
    grd.addColorStop(0, 'rgba(255,255,255,1)'); grd.addColorStop(0.55, 'rgba(255,255,255,0.85)'); grd.addColorStop(1, 'rgba(255,255,255,0)');
    g.fillStyle = grd; g.fillRect(0, 0, 512, 512);
    const floor = new THREE.Mesh(new THREE.CircleGeometry(4.2, 96),
      new THREE.MeshStandardMaterial({ color: 0xbebdb9, roughness: 0.92, transparent: true, alphaMap: new THREE.CanvasTexture(fc), depthWrite: false }));
    floor.position.set(0, 0.45, -0.56); floor.renderOrder = -2; scene.add(floor);
    const catcher = new THREE.Mesh(new THREE.CircleGeometry(3.0, 96), new THREE.ShadowMaterial({ opacity: 0.22, transparent: true }));
    catcher.position.set(0, 0.45, -0.5595); catcher.receiveShadow = true; scene.add(catcher);

    this.root = new THREE.Group(); scene.add(this.root);
    this.mode = 'orbit'; this.autoOrbit = true; this.time = 0; this.playing = false; this.speed = 1;
    this.setOrbit(true);
    this.resize();
    new ResizeObserver(() => this.resize()).observe(container);
    this.clock = new THREE.Clock();
    r.setAnimationLoop(() => this._tick());
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  async _gltf(url) { if (!this.cache.has(url)) this.cache.set(url, this.loader.loadAsync(url)); return this.cache.get(url); }

  material(role, rgba, node) {
    const c = new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace);
    if (role === 'table') {
      if (!this.woodTex) {
        this.woodTex = new THREE.TextureLoader().load('assets/table_wood.jpg');
        this.woodTex.colorSpace = THREE.SRGBColorSpace; this.woodTex.wrapS = this.woodTex.wrapT = THREE.RepeatWrapping;
        this.woodTex.repeat.set(1.6, 1.0); this.woodTex.anisotropy = 8;
      }
      return new THREE.MeshStandardMaterial({ map: this.woodTex, roughness: 0.62, metalness: 0 });
    }
    if (role === 'arm') {
      if (/quick_changer|flange|wrist/i.test(node || '')) return new THREE.MeshPhysicalMaterial({ color: 0x1c1d20, roughness: 0.4, metalness: 0.5 });
      return new THREE.MeshPhysicalMaterial({ color: 0xeceef0, roughness: 0.32, metalness: 0.02, clearcoat: 0.5, clearcoatRoughness: 0.25 });
    }
    if (role === 'torso') return new THREE.MeshStandardMaterial({ color: 0x3a3d43, roughness: 0.45, metalness: 0.35 });
    if (role === 'hand') {
      const dark = rgba[0] + rgba[1] + rgba[2] < 0.6;
      return new THREE.MeshPhysicalMaterial({ color: dark ? 0x18191c : c, roughness: dark ? 0.55 : 0.36, metalness: dark ? 0.2 : 0.45, clearcoat: 0.25 });
    }
    if (role === 'object') {
      if (/croissant/i.test(node || '')) return new THREE.MeshPhysicalMaterial({ color: new THREE.Color(0xc8781e), roughness: 0.55, sheen: 0.6, sheenColor: new THREE.Color(0xffc26b), clearcoat: 0.15 });
      if (/plate/i.test(node || '')) return new THREE.MeshPhysicalMaterial({ color: new THREE.Color(0xa9bbd6), roughness: 0.22, clearcoat: 0.7, clearcoatRoughness: 0.15 });
    }
    return new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, metalness: 0.05, transparent: rgba[3] < 0.99, opacity: rgba[3] });
  }

  primitive(gm) {
    const s = gm.size;
    switch (gm.type) {
      case 'box': return new THREE.BoxGeometry(2 * s[0], 2 * s[1], 2 * s[2]);
      case 'sphere': return new THREE.SphereGeometry(s[0], 32, 16);
      case 'ellipsoid': { const g = new THREE.SphereGeometry(1, 32, 16); g.scale(s[0], s[1], s[2]); return g; }
      case 'capsule': { const g = new THREE.CapsuleGeometry(s[0], 2 * s[1], 8, 24); g.rotateX(Math.PI / 2); return g; }
      case 'cylinder': { const g = new THREE.CylinderGeometry(s[0], s[0], 2 * s[1], 40); g.rotateX(Math.PI / 2); return g; }
      default: return null;
    }
  }

  async loadModel(url) {
    if (this.modelUrl === url) return;
    this.modelUrl = url;
    const model = this.model = await (await fetch(url)).json();
    const gltf = await this._gltf(model.glb);
    const nodes = new Map(); gltf.scene.traverse(o => { if (o.isMesh) nodes.set(o.name, o); });
    this.root.clear();
    this.bodyObj = model.bodies.map(() => { const o = new THREE.Group(); o.matrixAutoUpdate = false; this.root.add(o); return o; });
    this.world = model.bodies.map(() => new THREE.Matrix4());
    this.local = model.bodies.map(b => new THREE.Matrix4().compose(new THREE.Vector3(...b.pos), mjQuat(b.quat), ONE));
    for (const gm of model.geoms) {
      let geo;
      if (gm.type === 'mesh') {
        const n = nodes.get(gm.node) || gltf.scene.getObjectByName(gm.node);
        if (!n) continue;
        geo = n.geometry;
      } else geo = this.primitive(gm);
      if (!geo) continue;
      const mesh = new THREE.Mesh(geo, this.material(gm.role, gm.rgba, gm.node || gm.name));
      mesh.position.set(...gm.pos); mesh.quaternion.copy(mjQuat(gm.quat));
      mesh.castShadow = gm.role !== 'table'; mesh.receiveShadow = true;
      this.bodyObj[gm.body].add(mesh);
    }
    this.camByName = model.cameras;
  }

  setEpisode(ep) {
    this.ep = ep; this.frames = ep.frames; this.fps = ep.fps;
    this.duration = (this.frames.length - 1) / this.fps;
    this.time = 0; this.applyTime(0);
  }

  // forward kinematics at fractional frame f
  fk(f) {
    const F = this.frames, i0 = Math.max(0, Math.min(F.length - 1, Math.floor(f))), i1 = Math.min(F.length - 1, i0 + 1), u = f - i0;
    const qa = F[i0], qb = F[i1];
    const qv = k => qa[k] + (qb[k] - qa[k]) * u;
    const B = this.model.bodies;
    for (let b = 1; b < B.length; b++) {
      const body = B[b], W = this.world[b];
      let free = false;
      W.copy(this.world[body.parent]).multiply(this.local[b]);
      for (const j of body.joints) {
        if (j.type === 'free') {
          const a = j.qadr;
          Q.set(qv(a + 4), qv(a + 5), qv(a + 6), qv(a + 3)).normalize();
          W.compose(V.set(qv(a), qv(a + 1), qv(a + 2)), Q, ONE); free = true;
        } else if (j.type === 'hinge') {
          const ax = new THREE.Vector3(...j.axis).normalize();
          M.makeTranslation(j.pos[0], j.pos[1], j.pos[2]);
          M.multiply(M2.makeRotationAxis(ax, qv(j.qadr)));
          M.multiply(M2.makeTranslation(-j.pos[0], -j.pos[1], -j.pos[2]));
          W.multiply(M);
        } else if (j.type === 'slide') {
          const x = qv(j.qadr);
          W.multiply(M.makeTranslation(j.axis[0] * x, j.axis[1] * x, j.axis[2] * x));
        }
      }
      this.bodyObj[b].matrix.copy(W); this.bodyObj[b].matrixWorldNeedsUpdate = true;
    }
  }

  applyTime(t) {
    if (!this.frames) return;
    this.time = Math.max(0, Math.min(this.duration, t));
    this.fk(this.time * this.fps);
    this.onTime?.(this.time);
  }

  // ------------------------------------------------------------------ cameras
  setOrbit(instant) {
    const cam = this.camera;
    cam.fov = 38; cam.updateProjectionMatrix();
    if (instant) { cam.position.set(1.05, 1.45, 0.78); this.controls.target.set(0, 0.36, 0.12); }
    this.controls.enabled = true;
  }

  setCamera(mode, keepPose = false) {
    this.mode = mode;
    if (mode === 'orbit') { if (!keepPose) { this.setOrbit(true); this.autoOrbit = true; } this.controls.enabled = true; return; }
    if (mode === 'top') { this.mode = 'orbit'; this.autoOrbit = false; this.camera.fov = 38; this.camera.updateProjectionMatrix();
      this.camera.position.set(0.0, 0.12, 1.45); this.controls.target.set(0, 0.45, 0); this.controls.update(); return; }
    this.controls.enabled = true;
  }

  _attachCam(name) {
    const c = this.camByName?.[name]; if (!c) return false;
    const W = this.world[c.body] || new THREE.Matrix4();
    M.compose(new THREE.Vector3(...c.pos), mjQuat(c.quat), ONE);
    M2.copy(W).multiply(M);
    M2.decompose(this.camera.position, this.camera.quaternion, V);
    if (this.camera.fov !== c.fovy) { this.camera.fov = c.fovy; this.camera.updateProjectionMatrix(); }
    return true;
  }

  _tick() {
    const dt = Math.min(this.clock.getDelta(), 0.1);
    if (this.playing && this.frames) {
      let t = this.time + dt * this.speed;
      if (t > this.duration) t = this.loop ? 0 : this.duration, this.playing = this.loop;
      this.applyTime(t);
      if (!this.playing) this.onEnd?.();
    }
    if (this.mode === 'orbit') {
      if (this.autoOrbit) {
        const tg = this.controls.target, p = this.camera.position;
        const a = Math.atan2(p.y - tg.y, p.x - tg.x) + dt * 0.12, rr = Math.hypot(p.x - tg.x, p.y - tg.y);
        p.x = tg.x + rr * Math.cos(a); p.y = tg.y + rr * Math.sin(a);
      }
      this.controls.update();
    } else {
      const name = { zed: 'ego', human: 'video_cam', wrist: this.wristCam }[this.mode];
      this._attachCam(name);
    }
    this.renderer.render(this.scene, this.camera);
  }
}
