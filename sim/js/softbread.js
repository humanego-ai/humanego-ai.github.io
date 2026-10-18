// Deformable bread for the viewer. The simulator models each bread as a MuJoCo flex whose
// surface follows a trilinear grid of point masses (humanego_sim/softbody.py); here the same
// trilinear interpolation deforms the textured GLB, so what you see is what the fingers squeezed.
//
// Replays send the bread's rigid pose plus sparse grid residuals (in the bread frame);
// Live mode sends the grid corners straight from MuJoCo WASM and the rigid frame is fitted here.
import * as THREE from 'three';

export function breadMaterial(src) {
  // egg-washed crust: base colour + crumb normal map + roughness, a thin glossy coat and a warm sheen
  return new THREE.MeshPhysicalMaterial({
    map: src.map || null, normalMap: src.normalMap || null, normalScale: new THREE.Vector2(0.85, 0.85),
    roughnessMap: src.roughnessMap || null, roughness: 1.0, metalness: 0.0,
    clearcoat: 0.5, clearcoatRoughness: 0.32, sheen: 0.35, sheenRoughness: 0.55,
    sheenColor: new THREE.Color(0xffb35c), envMapIntensity: 1.15,
  });
}

export class SoftBread {
  constructor(name, spec, gltfScene) {
    this.name = name;
    const c = this.cells = spec.cells.map(Number);
    this.lo = spec.lo; this.hi = spec.hi;
    this.h = [0, 1, 2].map((a) => (this.hi[a] - this.lo[a]) / c[a]);
    this.nNode = (c[0] + 1) * (c[1] + 1) * (c[2] + 1);
    this.nodes = spec.nodes;                       // MuJoCo body ids (live mode)
    this.rest = new Float32Array(this.nNode * 3);  // rest corners, bread frame, k fastest (flexcomp order)
    let n = 0;
    for (let i = 0; i <= c[0]; i++) for (let j = 0; j <= c[1]; j++) for (let k = 0; k <= c[2]; k++) {
      this.rest[3 * n] = this.lo[0] + this.h[0] * i;
      this.rest[3 * n + 1] = this.lo[1] + this.h[1] * j;
      this.rest[3 * n + 2] = this.lo[2] + this.h[2] * k;
      n++;
    }
    this.c0 = [0, 1, 2].map((a) => (this.lo[a] + this.hi[a]) / 2);   // rest centroid of the grid

    let src = null;
    gltfScene.updateMatrixWorld(true);
    gltfScene.traverse((o) => { if (o.isMesh && !src) src = o; });
    const geo = src.geometry.clone();
    // deformed on the CPU: work on plain float32 attributes even if the GLB stores them quantized
    for (const k of ['position', 'normal']) {
      const a = geo.attributes[k];
      if (a && (a.isInterleavedBufferAttribute || a.normalized || !(a.array instanceof Float32Array))) {
        const f = new Float32Array(a.count * a.itemSize);
        for (let i = 0; i < a.count; i++) for (let c = 0; c < a.itemSize; c++) f[i * a.itemSize + c] = a.getComponent(i, c);
        geo.setAttribute(k, new THREE.BufferAttribute(f, a.itemSize));
      }
    }
    geo.applyMatrix4(src.matrixWorld);
    geo.rotateX(Math.PI / 2);                      // glTF Y-up -> MuJoCo Z-up (bread frame)
    if (!geo.attributes.normal) geo.computeVertexNormals();
    this.geo = geo;
    this.mesh = new THREE.Mesh(geo, breadMaterial(src.material));
    this.mesh.castShadow = true; this.mesh.receiveShadow = true;
    this.mesh.frustumCulled = false;
    this.group = new THREE.Group();
    this.group.name = name;
    this.group.matrixAutoUpdate = false;
    this.group.add(this.mesh);

    this._bind();
    this.D = new Float32Array(this.nNode * 3);     // corner displacement from rest (bread frame)
    this.deformed = false;
  }

  _idx(i, j, k) { return (i * (this.cells[1] + 1) + j) * (this.cells[2] + 1) + k; }

  _bind() {
    const pos = this.geo.attributes.position.array, nv = pos.length / 3, c = this.cells;
    this.restPos = Float32Array.from(pos);
    this.restNrm = Float32Array.from(this.geo.attributes.normal.array);
    this.bIdx = new Int32Array(nv * 8); this.bW = new Float32Array(nv * 8); this.bCell = new Int32Array(nv);
    for (let v = 0; v < nv; v++) {
      const cell = [0, 0, 0], f = [0, 0, 0];
      for (let a = 0; a < 3; a++) {
        const u = Math.min(Math.max((pos[3 * v + a] - this.lo[a]) / (this.hi[a] - this.lo[a]), 0), 1) * c[a];
        cell[a] = Math.min(Math.floor(u), c[a] - 1); f[a] = u - cell[a];
      }
      this.bCell[v] = (cell[0] * c[1] + cell[1]) * c[2] + cell[2];
      let m = 0;
      for (let di = 0; di < 2; di++) for (let dj = 0; dj < 2; dj++) for (let dk = 0; dk < 2; dk++) {
        this.bIdx[8 * v + m] = this._idx(cell[0] + di, cell[1] + dj, cell[2] + dk);
        this.bW[8 * v + m] = (di ? f[0] : 1 - f[0]) * (dj ? f[1] : 1 - f[1]) * (dk ? f[2] : 1 - f[2]);
        m++;
      }
    }
    // per cell: its 8 corners and the gradient of each corner weight at the cell centre
    const nc = c[0] * c[1] * c[2];
    this.cellCorners = new Int32Array(nc * 8); this.cellGrad = new Float32Array(nc * 24); this.cellM = new Float32Array(nc * 9);
    for (let i = 0; i < c[0]; i++) for (let j = 0; j < c[1]; j++) for (let k = 0; k < c[2]; k++) {
      const ci = (i * c[1] + j) * c[2] + k;
      let m = 0;
      for (let di = 0; di < 2; di++) for (let dj = 0; dj < 2; dj++) for (let dk = 0; dk < 2; dk++) {
        this.cellCorners[8 * ci + m] = this._idx(i + di, j + dj, k + dk);
        this.cellGrad[24 * ci + 3 * m] = (di ? 1 : -1) / (4 * this.h[0]);
        this.cellGrad[24 * ci + 3 * m + 1] = (dj ? 1 : -1) / (4 * this.h[1]);
        this.cellGrad[24 * ci + 3 * m + 2] = (dk ? 1 : -1) / (4 * this.h[2]);
        m++;
      }
    }
  }

  /** Replay: rigid pose (pos, wxyz) of the bread frame. */
  setPose(p, q) {
    this.group.matrix.compose(new THREE.Vector3(p[0], p[1], p[2]), new THREE.Quaternion(q[1], q[2], q[3], q[0]),
      new THREE.Vector3(1, 1, 1));
    this.group.matrixWorldNeedsUpdate = true;
  }

  /** Replay: corner residuals in the bread frame (metres), or null when undeformed. */
  setDisplacement(D) {
    if (!D) { this._restore(); return; }
    this.D.set(D);
    this._apply();
  }

  /** Live: world positions of the grid corners (Float32Array / array, 3 per corner). */
  fromNodes(P) {
    const c = this.cells, ex = [0, 0, 0], ey = [0, 0, 0], cen = [0, 0, 0];
    const ok = (n) => this.nodes[n] >= 0;          // corners MuJoCo pinned to the world are skipped
    let nv = 0; const rc = [0, 0, 0];
    for (let i = 0; i <= c[0]; i++) for (let j = 0; j <= c[1]; j++) for (let k = 0; k <= c[2]; k++) {
      const n = this._idx(i, j, k), a = 3 * n;
      if (!ok(n)) continue;
      for (let d = 0; d < 3; d++) { cen[d] += P[a + d]; rc[d] += this.rest[a + d]; }
      nv++;
      if (i < c[0] && ok(this._idx(i + 1, j, k))) { const b = 3 * this._idx(i + 1, j, k); for (let d = 0; d < 3; d++) ex[d] += P[b + d] - P[a + d]; }
      if (j < c[1] && ok(this._idx(i, j + 1, k))) { const b = 3 * this._idx(i, j + 1, k); for (let d = 0; d < 3; d++) ey[d] += P[b + d] - P[a + d]; }
    }
    const X = new THREE.Vector3(...ex).normalize();
    const Y = new THREE.Vector3(...ey); Y.addScaledVector(X, -Y.dot(X)).normalize();
    const Z = new THREE.Vector3().crossVectors(X, Y);
    const Rm = new THREE.Matrix4().makeBasis(X, Y, Z);
    const t = new THREE.Vector3(cen[0] / nv, cen[1] / nv, cen[2] / nv)
      .sub(new THREE.Vector3(rc[0] / nv, rc[1] / nv, rc[2] / nv).applyMatrix4(Rm));
    this.group.matrix.copy(Rm).setPosition(t);
    this.group.matrixWorldNeedsUpdate = true;
    const e = Rm.elements;   // column-major: columns are X, Y, Z
    let maxd = 0;
    for (let n = 0; n < this.nNode; n++) {
      if (!ok(n)) { this.D[3 * n] = this.D[3 * n + 1] = this.D[3 * n + 2] = 0; continue; }
      const dx = P[3 * n] - t.x, dy = P[3 * n + 1] - t.y, dz = P[3 * n + 2] - t.z;
      for (let a = 0; a < 3; a++) {
        const v = e[4 * a] * dx + e[4 * a + 1] * dy + e[4 * a + 2] * dz - this.rest[3 * n + a];
        this.D[3 * n + a] = v; maxd = Math.max(maxd, Math.abs(v));
      }
    }
    if (maxd < 2.5e-4) this._restore(); else this._apply();
  }

  _restore() {
    if (!this.deformed) return;
    this.geo.attributes.position.array.set(this.restPos);
    this.geo.attributes.normal.array.set(this.restNrm);
    this.geo.attributes.position.needsUpdate = true; this.geo.attributes.normal.needsUpdate = true;
    this.deformed = false;
  }

  _apply() {
    const D = this.D, nc = this.cellM.length / 9;
    // per-cell normal transform: cofactor of J = I + sum_c D_c g_c^T  (proportional to J^-T)
    for (let ci = 0; ci < nc; ci++) {
      const J = [1, 0, 0, 0, 1, 0, 0, 0, 1];
      for (let m = 0; m < 8; m++) {
        const n = this.cellCorners[8 * ci + m], g = 24 * ci + 3 * m;
        for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) J[3 * a + b] += D[3 * n + a] * this.cellGrad[g + b];
      }
      const M = this.cellM, o = 9 * ci;
      M[o] = J[4] * J[8] - J[5] * J[7]; M[o + 1] = J[5] * J[6] - J[3] * J[8]; M[o + 2] = J[3] * J[7] - J[4] * J[6];
      M[o + 3] = J[2] * J[7] - J[1] * J[8]; M[o + 4] = J[0] * J[8] - J[2] * J[6]; M[o + 5] = J[1] * J[6] - J[0] * J[7];
      M[o + 6] = J[1] * J[5] - J[2] * J[4]; M[o + 7] = J[2] * J[3] - J[0] * J[5]; M[o + 8] = J[0] * J[4] - J[1] * J[3];
    }
    const pos = this.geo.attributes.position.array, nrm = this.geo.attributes.normal.array;
    const rp = this.restPos, rn = this.restNrm, nv = rp.length / 3, M = this.cellM;
    for (let v = 0; v < nv; v++) {
      let x = rp[3 * v], y = rp[3 * v + 1], z = rp[3 * v + 2];
      for (let m = 0; m < 8; m++) {
        const w = this.bW[8 * v + m]; if (w === 0) continue;
        const n = 3 * this.bIdx[8 * v + m];
        x += w * D[n]; y += w * D[n + 1]; z += w * D[n + 2];
      }
      pos[3 * v] = x; pos[3 * v + 1] = y; pos[3 * v + 2] = z;
      const o = 9 * this.bCell[v], a = rn[3 * v], b = rn[3 * v + 1], c = rn[3 * v + 2];
      // n' = cof(J) n  (cof(J) rows stored as M[o..o+8] in row-major order of cof(J))
      const nx = M[o] * a + M[o + 1] * b + M[o + 2] * c;
      const ny = M[o + 3] * a + M[o + 4] * b + M[o + 5] * c;
      const nz = M[o + 6] * a + M[o + 7] * b + M[o + 8] * c;
      const l = Math.hypot(nx, ny, nz) || 1;
      nrm[3 * v] = nx / l; nrm[3 * v + 1] = ny / l; nrm[3 * v + 2] = nz / l;
    }
    this.geo.attributes.position.needsUpdate = true; this.geo.attributes.normal.needsUpdate = true;
    this.deformed = true;
  }
}
