/* ============================================================
   VISION CLIENT — riconoscimento braccio robotico (ML)
   Dataset: POST /api/vision/capture  (pulsante Scatta foto)
   Inferenza:
     1) window.RobotVisionLocalAI  → IA locale (ONNX/TFLite) se presente
     2) baseline depth+RGB          → stima 7 giunti senza modello
   ============================================================ */
(function () {
  /* Giunti reali del robot: 6 motori -> 3 giunti visibili.
     BASE = giunto base · G1 = fine braccio 1 · G2 = fine braccio 2.
     La PINZA arriverà quando progettata (quarto punto). */
  const JOINT_LABELS = ['BASE', 'G1', 'G2'];
  const JOINT_COLORS = ['#ffbc5c', '#62e6c7', '#5c9dff'];
  const JOINTS_N = JOINT_LABELS.length;

  let visionEnabled = false;
  let visionLastJoints = null;   // [{x,y,z,label}] x7  (z mm opz)
  let visionBusy = false;
  let visionCount = 0;

  /* ---------- util ---------- */
  function $(id) { return document.getElementById(id); }

  function visionSyncOverlay() {
    const img = $('kinRgb'), c = $('kinVisionOverlay');
    if (!img || !c || !img.parentElement) return;
    const ir = img.getBoundingClientRect();
    const pr = img.parentElement.getBoundingClientRect();
    if (!ir.width) return;
    // object-fit:contain → rettangolo del contenuto, non dell'elemento
    const natW = img.naturalWidth || 640;
    const natH = img.naturalHeight || 480;
    const scale = Math.min(ir.width / natW, ir.height / natH);
    const dw = natW * scale, dh = natH * scale;
    c.style.left = (ir.left - pr.left + (ir.width - dw) / 2) + 'px';
    c.style.top = (ir.top - pr.top + (ir.height - dh) / 2) + 'px';
    c.style.width = dw + 'px';
    c.style.height = dh + 'px';
  }

  async function fetchJson(url, opts) {
    const r = await fetch(url, opts);
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.error || (r.status + ' ' + url));
    return j;
  }

  /* ---------- dataset: Scatta foto ---------- */
  async function visionCapture() {
    if (!kinRgbCanvas || !kinRgbCtx) throw new Error('Kinect RGB non pronta (apri Modalità 2)');
    visionSyncOverlay();

    const dataUrl = kinRgbCanvas.toDataURL('image/jpeg', 0.92);

    // depth downsample 4x → binario raw per training 3D
    let depthB64 = null;
    if (kinDepth && kinDepth.length === 640 * 480) {
      const sw = 160, sh = 120;
      const small = new Uint16Array(sw * sh);
      for (let y = 0; y < sh; y++) {
        for (let x = 0; x < sw; x++) {
          small[y * sw + x] = kinDepth[(y * 4) * 640 + (x * 4)];
        }
      }
      const bytes = new Uint8Array(small.buffer);
      let bin = '';
      const CH = 0x8000;
      for (let i = 0; i < bytes.length; i += CH) {
        bin += String.fromCharCode.apply(null, bytes.subarray(i, i + CH));
      }
      depthB64 = btoa(bin);
    }

    // stato q[] ESP32 (label supervisionata aggiuntiva)
    let q = null;
    try {
      const st = await fetchJson('/api/esp32/status');
      if (st && Array.isArray(st.q)) q = st.q;
    } catch (e) {}

    const body = { image: dataUrl, depthB64, q, note: 'btn-scatta' };
    const out = await fetchJson('/api/vision/capture', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    });
    visionCount = out.count || (visionCount + 1);
    visionUpdateInfo('foto salvata #' + visionCount);
    return out;
  }

  async function visionRefreshCount() {
    try {
      const j = await fetchJson('/api/vision/dataset');
      visionCount = j.count || 0;
      const m = await fetchJson('/api/vision/model/status').catch(() => null);
      visionUpdateInfo('dataset: ' + visionCount + (m ? ' · label: ' + m.labeled : ''));
    } catch (e) {
      visionUpdateInfo('dataset?');
    }
  }

  function visionUpdateInfo(txt) {
    const el = $('kinVisionInfo');
    if (el) el.textContent = txt;
  }

  /* ---------- baseline: segmentazione depth/RGB → 7 giunti ---------- */

  // morfologia binaria (kernel quadrato r)
  function visionErode(src, W, H, r) {
    const out = new Uint8Array(W * H);
    for (let v = r; v < H - r; v++) {
      for (let u = r; u < W - r; u++) {
        let keep = 1;
        for (let dv = -r; dv <= r && keep; dv++) {
          for (let du = -r; du <= r; du++) {
            if (!src[(v + dv) * W + (u + du)]) { keep = 0; break; }
          }
        }
        out[v * W + u] = keep;
      }
    }
    return out;
  }

  function visionDilate(src, W, H, r) {
    const out = new Uint8Array(W * H);
    for (let v = 0; v < H; v++) {
      for (let u = 0; u < W; u++) {
        if (!src[v * W + u]) continue;
        const v0 = Math.max(0, v - r), v1 = Math.min(H - 1, v + r);
        const u0 = Math.max(0, u - r), u1 = Math.min(W - 1, u + r);
        for (let vv = v0; vv <= v1; vv++) {
          for (let uu = u0; uu <= u1; uu++) out[vv * W + uu] = 1;
        }
      }
    }
    return out;
  }

  // largest connected component (4-conn) → mask + indici
  function visionLargestCC(mask, W, H) {
    const lab = new Int32Array(W * H);
    const qx = new Int32Array(W * H);
    const qy = new Int32Array(W * H);
    let best = 0, bestN = 0, bestRoot = -1;
    let next = 1;
    for (let i = 0; i < W * H; i++) {
      if (!mask[i] || lab[i]) continue;
      const id = next++;
      let head = 0, tail = 0, n = 0;
      qx[tail] = i % W; qy[tail] = (i / W) | 0; tail++;
      lab[i] = id;
      while (head < tail) {
        const u = qx[head], v = qy[head]; head++; n++;
        const nb = [[u + 1, v], [u - 1, v], [u, v + 1], [u, v - 1]];
        for (let k = 0; k < 4; k++) {
          const nu = nb[k][0], nv = nb[k][1];
          if (nu < 0 || nv < 0 || nu >= W || nv >= H) continue;
          const ni = nv * W + nu;
          if (mask[ni] && !lab[ni]) { lab[ni] = id; qx[tail] = nu; qy[tail] = nv; tail++; }
        }
      }
      if (n > bestN) { bestN = n; best = id; bestRoot = i; }
    }
    if (bestN < 120) return { mask: new Uint8Array(W * H), n: 0 };
    const out = new Uint8Array(W * H);
    for (let i = 0; i < W * H; i++) if (lab[i] === best) out[i] = 1;
    return { mask: out, n: bestN };
  }

  function visionMedian(arr) {
    if (!arr.length) return 0;
    const a = arr.slice().sort((x, y) => x - y);
    return a[a.length >> 1];
  }

  function visionBaselineDetect() {
    if (!kinDepth || kinDepth.length !== 640 * 480) return null;
    const W = 640, H = 480;
    const dmaxUI = +(($('kinRangeMax') && $('kinRangeMax').value) || 4500);
    const dmax = Math.min(Math.max(dmaxUI, 1000), 8000);
    const dmin = 500;

    // percentili depth (campionamento stride 4)
    const sample = [];
    for (let i = 0; i < kinDepth.length; i += 4) {
      const d = kinDepth[i];
      if (d >= dmin && d <= dmax) sample.push(d);
    }
    if (sample.length < 400) return null;
    sample.sort((a, b) => a - b);
    const p = q => sample[Math.min(sample.length - 1, (q * (sample.length - 1)) | 0)];
    const p50 = p(0.50);

    // istogramma 50mm → separa cluster "vicino" (braccio) da parete
    const binSz = 50;
    const nBins = Math.ceil((dmax - dmin) / binSz) + 2;
    const hist = new Uint32Array(nBins);
    for (let i = 0; i < sample.length; i++) {
      const b = ((sample[i] - dmin) / binSz) | 0;
      if (b >= 0 && b < nBins) hist[b]++;
    }
    let farBin = 0;
    for (let b = 1; b < nBins; b++) if (hist[b] > hist[farBin]) farBin = b;
    const farCount = hist[farBin] || 1;

    // ultimo bin "vicino" con segnale (≥2% del picco lontano o ≥40 campioni)
    let lastNear = -1;
    for (let b = 0; b < farBin - 1; b++) {
      if (hist[b] >= Math.max(40, farCount * 0.02)) lastNear = b;
    }

    let thr;
    if (lastNear >= 0) {
      // inizio del plateau "parete"
      let startFar = farBin;
      while (startFar > lastNear + 1 && hist[startFar - 1] >= farCount * 0.25) startFar--;
      // metà del gap tra cluster vicino e parete
      thr = dmin + ((lastNear + 1 + startFar) / 2) * binSz;
      // se non c'è gap netto, prendi fine del cluster vicino + margine
      if (startFar - lastNear <= 2) thr = dmin + (lastNear + 2) * binSz;
    } else {
      // nessun cluster vicino distinto → soglia fissa rispetto alla parete
      thr = dmin + farBin * binSz - 450;
    }
    thr = Math.min(thr, dmax - 100);
    thr = Math.max(thr, dmin + 200);
    if (thr <= dmin) return null;

    // maschera depth (floor fisso: p05 con sfondo dominante è la parete!)
    let mask = new Uint8Array(W * H);
    const dFloor = dmin;
    for (let i = 0; i < W * H; i++) {
      const d = kinDepth[i];
      if (d >= dFloor && d <= thr) mask[i] = 1;
    }
    // debug console
    if (typeof console !== 'undefined' && console.debug) {
      console.debug('[vision] p50', p50, 'farBin', farBin, 'lastNear', lastNear, 'thr', thr, 'maskN', maskCount(mask));
    }

    // rifinitura RGB: il braccio è scuro rispetto a parete finestra
    if (kinRgbData && kinRgbData.length >= W * H * 4) {
      let darkN = 0;
      const dark = new Uint8Array(W * H);
      for (let i = 0; i < W * H; i++) {
        if (!mask[i]) continue;
        const r = kinRgbData[i * 4], g = kinRgbData[i * 4 + 1], b = kinRgbData[i * 4 + 2];
        const lum = 0.299 * r + 0.587 * g + 0.114 * b;
        if (lum < 110) { dark[i] = 1; darkN++; }
      }
      // se il sottoinsieme scuro è consistente, tieni quello (più pulito)
      if (darkN > 350 && darkN < maskCount(mask) * 0.95) {
        // dilata leggero per non perdere cavi/parti sottili
        mask = visionDilate(dark, W, H, 2);
        mask = visionErode(mask, W, H, 1);
      }
    }

    // apri (enoise) → largest CC → chiudi (collega link sottili)
    mask = visionErode(mask, W, H, 1);
    mask = visionDilate(mask, W, H, 1);
    const cc = visionLargestCC(mask, W, H);
    if (cc.n < 150) return null;
    mask = visionDilate(cc.mask, W, H, 2);
    mask = visionErode(mask, W, H, 1);
    const cc2 = visionLargestCC(mask, W, H);
    if (cc2.n >= 150) mask = cc2.mask; else mask = cc.mask;

    // indici pixel della maschera
    const px = [], py = [], pd = [];
    for (let v = 0; v < H; v++) {
      for (let u = 0; u < W; u++) {
        if (!mask[v * W + u]) continue;
        const d = kinDepth[v * W + u];
        if (d >= dFloor && d <= thr) { px.push(u); py.push(v); pd.push(d); }
      }
    }
    const n = px.length;
    if (n < 150) return null;
    if (typeof console !== 'undefined' && console.debug) {
      console.debug('[vision] CC pixels', n, 'thr', thr);
    }

    // PCA solo sul braccio isolato
    let muU = 0, muV = 0;
    for (let i = 0; i < n; i++) { muU += px[i]; muV += py[i]; }
    muU /= n; muV /= n;
    let suu = 0, svv = 0, suv = 0;
    for (let i = 0; i < n; i++) {
      const du = px[i] - muU, dv = py[i] - muV;
      suu += du * du; svv += dv * dv; suv += du * dv;
    }
    const tr = suu + svv, det = suu * svv - suv * suv;
    const tmp = Math.sqrt(Math.max(0, tr * tr / 4 - det));
    const l1 = tr / 2 + tmp;
    let ax, ay;
    if (Math.abs(suv) > 1e-6) { ax = l1 - svv; ay = suv; }
    else if (suu >= svv) { ax = 1; ay = 0; }
    else { ax = 0; ay = 1; }
    const al = Math.hypot(ax, ay) || 1;
    ax /= al; ay /= al;
    // orienta "verso l'alto" (v decrescente) — base in basso
    if (ay > 0 || (Math.abs(ay) < 1e-6 && ax > 0)) { ax = -ax; ay = -ay; }

    let tMin = Infinity, tMax = -Infinity;
    const ts = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const t = (px[i] - muU) * ax + (py[i] - muV) * ay;
      ts[i] = t;
      if (t < tMin) tMin = t;
      if (t > tMax) tMax = t;
    }
    const span = tMax - tMin;
    if (span < 50) return null;

    // 12 fette → centroidi (u,v) coerenti + mediana depth locale
    const NB = 12;
    const sliceU = new Float64Array(NB).fill(NaN);
    const sliceV = new Float64Array(NB).fill(NaN);
    const sliceD = new Float64Array(NB).fill(NaN);
    const sliceW = new Float64Array(NB).fill(0); // "larghezza" = n pixel (per capire base)
    for (let s = 0; s < NB; s++) {
      const t0 = tMin + (s / NB) * span;
      const t1 = tMin + ((s + 1) / NB) * span;
      const bu = [], bv = [], bd = [];
      for (let i = 0; i < n; i++) {
        if (ts[i] >= t0 && ts[i] < t1) { bu.push(px[i]); bv.push(py[i]); bd.push(pd[i]); }
      }
      if (bu.length < 4) continue;
      sliceU[s] = bu.reduce((a, b) => a + b, 0) / bu.length;
      sliceV[s] = bv.reduce((a, b) => a + b, 0) / bv.length;
      sliceD[s] = visionMedian(bd);
      sliceW[s] = bu.length;
    }

    // 7 giunti da 12 fette (0, 2, 4, ... oppure mappatura uniforme)
    const st = (typeof kinStatus !== 'undefined' && kinStatus && kinStatus.depth)
      ? kinStatus.depth : { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };

    // estremi: base = più largo (corpo base) e/o più in basso
    let endA = -1, endB = -1;
    for (let s = 0; s < NB; s++) if (!isNaN(sliceU[s])) { if (endA < 0) endA = s; endB = s; }
    if (endA < 0 || endB <= endA) return null;
    // punteggio base: larghezza * 2 + quota in basso (v alto)
    const scoreA = sliceW[endA] * 2 + sliceV[endA];
    const scoreB = sliceW[endB] * 2 + sliceV[endB];
    const baseIsA = scoreA >= scoreB;
    // ordine indici fette dalla base all'estremità
    const order = [];
    if (baseIsA) { for (let s = endA; s <= endB; s++) order.push(s); }
    else { for (let s = endB; s >= endA; s--) order.push(s); }
    const valid = order.filter(s => !isNaN(sliceU[s]));
    if (valid.length < 3) return null;

    // campiona JOINTS_N posizioni lungo le fette valide
    const joints = [];
    for (let k = 0; k < JOINTS_N; k++) {
      const fi = (k / (JOINTS_N - 1)) * (valid.length - 1);
      const i0 = Math.floor(fi), i1 = Math.min(valid.length - 1, i0 + 1);
      const fr = fi - i0;
      const s0 = valid[i0], s1 = valid[i1];
      const u = sliceU[s0] * (1 - fr) + sliceU[s1] * fr;
      const v = sliceV[s0] * (1 - fr) + sliceV[s1] * fr;

      // profondità: mediana dei pixel di maschera in finestra 15×15 attorno a (u,v)
      const uc = Math.round(u), vc = Math.round(v);
      const win = [];
      for (let dv = -7; dv <= 7; dv++) {
        for (let du = -7; du <= 7; du++) {
          const uu = uc + du, vv = vc + dv;
          if (uu < 0 || vv < 0 || uu >= W || vv >= H) continue;
          if (!mask[vv * W + uu]) continue;
          const d = kinDepth[vv * W + uu];
          if (d >= dFloor && d <= thr) win.push(d);
        }
      }
      const d = win.length >= 3 ? visionMedian(win)
        : (sliceD[s0] * (1 - fr) + sliceD[s1] * fr);
      if (!d) continue;

      const Xk = (u - st.cx) * d / st.fx;
      const Yk = (v - st.cy) * d / st.fy;
      joints.push({
        label: JOINT_LABELS[joints.length],
        x: u, y: v, z: d,
        wx: Xk, wy: -Yk, wz: d,
        color: JOINT_COLORS[joints.length]
      });
    }
    if (joints.length < JOINTS_N) return null;

    // debug: maschera dietro l'overlay
    visionDrawMask(mask, W, H);
    return joints.slice(0, JOINTS_N);
  }

  function maskCount(mask) {
    let c = 0;
    for (let i = 0; i < mask.length; i++) c += mask[i];
    return c;
  }

let visionLastMask = null;

// Modello locale: 1) ML ONNX (vision/model/model.keypoints.onnx) 2) fallback baseline depth
let _mlSession = null;
let _mlLoadTried = false;
let _mlCanvas = null;
let _mlMtime = 0;
let _mlMtimeTick = 0;

async function visionMlLoad() {
  if (_mlSession) return _mlSession;
  if (_mlLoadTried) return null;
  _mlLoadTried = true;
  try {
    if (typeof ort === 'undefined') return null;
    const st = await fetchJson('/api/vision/model/status');
    if (!st.files || !st.files.length) return null;
    _mlMtime = st.mtime || 0;
    if (typeof ort.env !== 'undefined' && ort.env.wasm) ort.env.wasm.numThreads = 2;
    _mlSession = await ort.InferenceSession.create('/vision/model/' + st.files[0], {
      executionProviders: ['wasm']
    });
    visionUpdateInfo('ML ONNX caricato · ' + st.files[0]);
    return _mlSession;
  } catch (e) {
    console.warn('[vision] caricamento ML ONNX fallito:', e);
    return null;
  }
}

// hot-reload: se l'auto-train sostituisce l'ONNX, ricarica la sessione da solo
async function visionMlCheckUpdate() {
  try {
    const st = await fetchJson('/api/vision/model/status');
    const m = st.mtime || 0;
    if (!m) return;
    if (_mlMtime && m !== _mlMtime) {
      _mlMtime = m;
      _mlSession = null;
      _mlLoadTried = false;
      visionUpdateInfo('nuovo modello ML — ricarico…');
      await visionMlLoad();
    } else if (!_mlMtime) {
      _mlMtime = m;
    }
  } catch (e) {}
}

function visionDepthAt(x, y) {
  if (!kinDepth || kinDepth.length !== 640 * 480) return null;
  const uc = Math.max(0, Math.min(639, Math.round(x)));
  const vc = Math.max(0, Math.min(479, Math.round(y)));
  const d = kinDepth[vc * 640 + uc];
  if (d > 300 && d < 8000) return d;
  const win = [];
  for (let dv = -5; dv <= 5; dv++) {
    for (let du = -5; du <= 5; du++) {
      const uu = uc + du, vv = vc + dv;
      if (uu < 0 || vv < 0 || uu >= 640 || vv >= 480) continue;
      const dd = kinDepth[vv * 640 + uu];
      if (dd > 300 && dd < 8000) win.push(dd);
    }
  }
  if (!win.length) return null;
  win.sort((a, b) => a - b);
  return win[win.length >> 1];
}

async function visionMlDetect() {
  const sess = await visionMlLoad();
  if (!sess || !kinRgbCanvas) return null;
  // letterbox 640x480 → 640x640 (pad 80 sopra/sotto), normalizzato 0..1, RGB
  if (!_mlCanvas) {
    _mlCanvas = document.createElement('canvas');
    _mlCanvas.width = 640; _mlCanvas.height = 640;
  }
  const mx = _mlCanvas.getContext('2d', { willReadFrequently: true });
  mx.fillStyle = '#727272';
  mx.fillRect(0, 0, 640, 640);
  mx.drawImage(kinRgbCanvas, 0, 80, 640, 480);
  const img = mx.getImageData(0, 0, 640, 640).data;
  const nPix = 640 * 640;
  const data = new Float32Array(3 * nPix);
  for (let i = 0; i < nPix; i++) {
    data[i] = img[i * 4] / 255;
    data[i + nPix] = img[i * 4 + 1] / 255;
    data[i + 2 * nPix] = img[i * 4 + 2] / 255;
  }
  const out = await sess.run({ [sess.inputNames[0]]: new ort.Tensor('float32', data, [1, 3, 640, 640]) });
  const pred = out[sess.outputNames[0]].data;          // [1,14,8400]
  const at = (c, i) => pred[c * 8400 + i];
  const sig = v => 1 / (1 + Math.exp(-v));
  const K = JOINTS_N;
  let best = -1, bestS = 0.35;                          // soglia confidenza classe
  for (let i = 0; i < 8400; i++) {
    const s = sig(at(4, i));
    if (s > bestS) { bestS = s; best = i; }
  }
  if (best < 0) return null;
  const joints = [];
  const st = (typeof kinStatus !== 'undefined' && kinStatus && kinStatus.depth)
    ? kinStatus.depth : { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };
  for (let k = 0; k < K; k++) {
    const kc = sig(at(5 + k * 3 + 2, best));
    if (kc < 0.25) return null;                          // keypoint poco sicuro → fallback
    const x = Math.min(639, Math.max(0, at(5 + k * 3, best)));
    const y = Math.min(479, Math.max(0, at(5 + k * 3 + 1, best) - 80));
    const z = visionDepthAt(x, y) || 1500;
    joints.push({
      label: JOINT_LABELS[k], x: Math.round(x), y: Math.round(y), z: z,
      wx: (x - st.cx) * z / st.fx, wy: -(y - st.cy) * z / st.fy, wz: z,
      color: JOINT_COLORS[k]
    });
  }
  return { joints, backend: 'ml-onnx' };
}

window.RobotVisionLocalAI = async function(options) {
  // 1) ML ONNX addestrato sul dataset approvato
  try {
    const out = await visionMlDetect();
    if (out && out.joints && out.joints.length >= JOINTS_N) return out;
  } catch (e) {
    console.warn('[vision] ML ONNX error:', e);
  }
  // 2) fallback: baseline depth+RGB
  try {
    const joints = visionBaselineDetect();
    if (joints && joints.length >= JOINTS_N) {
      return {
        joints: joints.slice(0, JOINTS_N).map((j, i) => ({
          label: j.label || JOINT_LABELS[i],
          x: j.x, y: j.y, z: j.z,
          wx: j.wx, wy: j.wy, wz: j.wz,
          color: j.color || JOINT_COLORS[i]
        })),
        backend: 'baseline-depth'
      };
    }
    return null;
  } catch (e) {
    console.warn('[vision] Local AI error:', e);
    return null;
  }
};

// disegna la maschera segmentata (verde tenue) sotto i giunti
  function visionDrawMask(mask, W, H) {
    visionLastMask = { mask, W, H };
  }

  function visionPaintMask(ctx) {
    if (!visionLastMask || !ctx) return;
    const { mask, W, H } = visionLastMask;
    if (!_maskCanvas) {
      _maskCanvas = document.createElement('canvas');
      _maskCanvas.width = W; _maskCanvas.height = H;
    }
    const mc = _maskCanvas.getContext('2d');
    const img = mc.createImageData(W, H);
    const data = img.data;
    for (let i = 0; i < W * H; i++) {
      if (mask[i]) {
        data[i * 4] = 40; data[i * 4 + 1] = 255; data[i * 4 + 2] = 160;
        data[i * 4 + 3] = 70;
      }
    }
    mc.putImageData(img, 0, 0);
    ctx.drawImage(_maskCanvas, 0, 0);
  }

  let _maskCanvas = null;

  /* ---------- inference unica ---------- */
  async function visionDetect() {
    if (visionBusy) return visionLastJoints;
    visionBusy = true;
    try {
      let joints = null;
      let backend = 'none';

      // 1) IA LOCALE (slot pronto — assegnare window.RobotVisionLocalAI)
      if (typeof window.RobotVisionLocalAI === 'function') {
        try {
          const rgb = kinRgbCtx ? kinRgbCtx.getImageData(0, 0, 640, 480) : null;
          const out = await window.RobotVisionLocalAI({
            imageData: rgb,
            depth: kinDepth,
            width: 640,
            height: 480,
            labels: JOINT_LABELS
          });
          if (out && Array.isArray(out.joints) && out.joints.length >= JOINTS_N) {
            joints = out.joints.slice(0, JOINTS_N).map((j, i) => ({
              label: j.label || JOINT_LABELS[i],
              x: +j.x || 0, y: +j.y || 0,
              z: j.z == null ? null : +j.z,
              wx: j.wx, wy: j.wy, wz: j.wz,
              color: JOINT_COLORS[i]
            }));
            backend = out.backend || 'local-ai';
            visionLastMask = null;
          }
        } catch (e) {
          console.warn('[vision] IA locale fallita, fallback baseline:', e);
        }
      }

      // 2) baseline depth
      if (!joints) {
        visionLastMask = null;
        joints = visionBaselineDetect();
        backend = 'baseline-depth';
      }

      if (joints) visionConstrainJoints(joints);   // vincolo link rigidi 3D
      visionLastJoints = joints;
      visionDrawOverlay(joints);
      visionDrawSkeleton3D(joints);
      visionUpdateInfo(backend + (joints ? ' · ' + joints.length + ' giunti' : ' · nessuno'));
      return joints;
    } finally {
      visionBusy = false;
    }
  }

  /* ---------- overlay 2D su RGB ---------- */
  function visionDrawOverlay(joints) {
    const c = $('kinVisionOverlay');
    if (!c) return;
    visionSyncOverlay();
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, 640, 480);
    visionPaintMask(ctx);
    // abilita il mouse sull'overlay solo quando ci sono giunti da trascinare
    c.style.pointerEvents = (joints && joints.length) ? 'auto' : 'none';
    if (!joints || joints.length < 2) return;

    // ossatura
    ctx.lineWidth = 3;
    ctx.strokeStyle = '#62e6c7';
    ctx.beginPath();
    ctx.moveTo(joints[0].x, joints[0].y);
    for (let i = 1; i < joints.length; i++) ctx.lineTo(joints[i].x, joints[i].y);
    ctx.stroke();

    for (let i = 0; i < joints.length; i++) {
      const j = joints[i];
      ctx.beginPath();
      ctx.arc(j.x, j.y, 9, 0, Math.PI * 2);
      ctx.fillStyle = j.color || JOINT_COLORS[i] || '#fff';
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#04080a';
      ctx.stroke();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 12px monospace';
      ctx.fillText(j.label || JOINT_LABELS[i], j.x + 12, j.y + 4);
      if (j.z) {
        ctx.fillStyle = '#9fb';
        ctx.font = '10px monospace';
        ctx.fillText(Math.round(j.z) + 'mm', j.x + 12, j.y + 16);
      }
    }

    // readout vincolo cinematico: distanze 3D dei link (verde ok / rosso fuori)
    if (visionLastConstraint && joints.length >= JOINTS_N) {
      const c = visionLastConstraint;
      const tol = c.auto ? 100 : 50;
      const segs = [
        { mx: (joints[0].x + joints[1].x) / 2, my: (joints[0].y + joints[1].y) / 2, d: c.d1, L: c.L1 },
        { mx: (joints[1].x + joints[2].x) / 2, my: (joints[1].y + joints[2].y) / 2, d: c.d2, L: c.L2 }
      ];
      ctx.font = 'bold 11px monospace';
      for (const s of segs) {
        const ok = Math.abs(s.d - s.L) <= tol;
        ctx.fillStyle = ok ? '#62e6c7' : '#ff5c5c';
        ctx.fillText((ok ? '⚓ ' : '⚠ ') + s.d + '/' + s.L + 'mm', s.mx + 8, s.my - 8);
      }
    }
  }

  /* ---------- scheletro 3D in kinScene ---------- */
  let kinFakeRoot = null;
  let kinFakeSpheres = [];
  let kinFakeRods = [];

  function visionEnsureSkeleton3D() {
    if (kinFakeRoot || typeof kinScene === 'undefined' || !kinScene) return;
    if (!kinScene) return;
    kinFakeRoot = new THREE.Group();
    kinFakeRoot.name = 'fakeJoints';
    const sm = new THREE.MeshStandardMaterial({
      color: 0xffbc5c, emissive: 0xff6a00, emissiveIntensity: 0.55,
      metalness: 0.2, roughness: 0.4
    });
    const lm = new THREE.MeshStandardMaterial({
      color: 0xff8c42, emissive: 0x331100, emissiveIntensity: 0.4,
      metalness: 0.15, roughness: 0.5
    });
    for (let i = 0; i < JOINTS_N; i++) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(28, 14, 14), sm.clone());
      m.material.color = new THREE.Color(JOINT_COLORS[i]);
      m.visible = false;
      kinFakeRoot.add(m);
      kinFakeSpheres.push(m);
    }
    for (let i = 0; i < JOINTS_N - 1; i++) {
      const m = new THREE.Mesh(new THREE.CylinderGeometry(8, 8, 1, 8), lm);
      m.visible = false;
      kinFakeRoot.add(m);
      kinFakeRods.push(m);
    }
    kinScene.add(kinFakeRoot);
  }

  function visionDrawSkeleton3D(joints) {
    visionEnsureSkeleton3D();
    if (!kinFakeRoot) return;
    if (!joints || joints.length < JOINTS_N) {
      kinFakeSpheres.forEach(s => s.visible = false);
      kinFakeRods.forEach(r => r.visible = false);
      return;
    }
    // coordinate mondo (stesso sistema di kinRebuildCloud, tilt incluso)
    const elevDeg = (kinStatus && typeof kinStatus.elev === 'number') ? kinStatus.elev : 0;
    const thT = elevDeg * Math.PI / 180, ct = Math.cos(thT), sn = Math.sin(thT);
    const pts = joints.map(j => {
      if (j.wx !== undefined && j.wy !== undefined) {
        // già in kin-space (Xk, -Yk, d) — applica tilt come cloud; x ribaltata
        const x = j.wx, y = j.wy, z = j.wz != null ? j.wz : (j.z || 0);
        // se wy è -Yk allora Yk = -wy; formula cloud: y' = -(Yk*ct - d*sn), z' = Yk*sn + d*ct
        const Yk = -y, d = z;
        return new THREE.Vector3(-x, -(Yk * ct - d * sn), Yk * sn + d * ct);
      }
      // fallback proiezione da px + depth (x ribaltata come la nuvola)
      const st = (kinStatus && kinStatus.depth) ? kinStatus.depth : { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };
      const d = j.z || 1500;
      const Xk = (j.x - st.cx) * d / st.fx;
      const Yk = (j.y - st.cy) * d / st.fy;
      return new THREE.Vector3(-Xk, -(Yk * ct - d * sn), Yk * sn + d * ct);
    });

    for (let i = 0; i < JOINTS_N; i++) {
      kinFakeSpheres[i].position.copy(pts[i]);
      kinFakeSpheres[i].visible = true;
    }
    const up = new THREE.Vector3(0, 1, 0);
    for (let i = 0; i < JOINTS_N - 1; i++) {
      const a = pts[i], b = pts[i + 1];
      const mid = a.clone().add(b).multiplyScalar(0.5);
      const dir = b.clone().sub(a);
      const len = dir.length() || 1;
      kinFakeRods[i].position.copy(mid);
      kinFakeRods[i].scale.set(1, len, 1);
      kinFakeRods[i].quaternion.setFromUnitVectors(up, dir.normalize());
      kinFakeRods[i].visible = true;
    }
  }

  /* ---------- wiring UI ---------- */
  function visionWire() {
    const shot = $('kinShotBtn');
    const det = $('kinDetectBtn');
    const tog = $('kinVisionToggle');
    const llm = $('kinLlmBtn');
    const llmYes = $('kinLlmYesBtn');
    const llmNo = $('kinLlmNoBtn');

    if (shot) {
      shot.onclick = async () => {
        shot.disabled = true;
        try {
          await visionCapture();
        } catch (e) {
          visionUpdateInfo('errore: ' + e.message);
          console.error('[vision]', e);
        } finally {
          shot.disabled = false;
        }
      };
    }
    if (det) {
      det.onclick = async () => {
        det.disabled = true;
        try { await visionDetect(); }
        catch (e) { visionUpdateInfo('detect: ' + e.message); }
        finally { det.disabled = false; }
      };
    }
    if (llm) {
      llm.onclick = async () => {
        llm.disabled = true;
        try {
          visionLlmAttempt = 1;
          visionLlmSetMsg('');
          await visionLlmAnalyze(1, '');
        } catch (e) {
          visionUpdateInfo('LLM: ' + e.message);
          visionLlmSetMsg(e.message);
          visionLlmShowConfirm(false);
          console.error('[vision-llm]', e);
        } finally {
          llm.disabled = false;
        }
      };
    }
    if (llmYes) {
      llmYes.onclick = async () => {
        llmYes.disabled = true;
        llmNo.disabled = true;
        try {
          await visionLlmApprove();
        } catch (e) {
          visionLlmSetMsg('salva: ' + e.message);
          console.error('[vision-llm]', e);
        } finally {
          llmYes.disabled = false;
          llmNo.disabled = false;
        }
      };
    }
    if (llmNo) {
      llmNo.onclick = async () => {
        llmYes.disabled = true;
        llmNo.disabled = true;
        try {
          const n = ++visionLlmAttempt;
          visionLlmSetMsg('riprovo…');
          $('kinLlmAttempt').textContent = 'tentativo ' + n;
          await visionLlmAnalyze(n, 'previous joint placement was incorrect');
        } catch (e) {
          visionLlmSetMsg(e.message);
          console.error('[vision-llm]', e);
        } finally {
          llmYes.disabled = false;
          llmNo.disabled = false;
        }
      };
    }
    if (tog) {
      tog.onchange = () => {
        visionEnabled = tog.checked;
        if (!visionEnabled) {
          visionLastMask = null;
          visionDrawOverlay(null);
          visionDrawSkeleton3D(null);
          visionUpdateInfo('rilevamento OFF');
        } else {
          visionUpdateInfo('rilevamento ON · ML auto ogni ~2s');
        }
      };
    }
    visionRefreshCount();
    visionWireDrag();
    visionLoadLinks();
  }

  /* ---------- LLM visione locale: analisi + approvazione umana ---------- */
  let visionLlmAttempt = 1;

  function visionLlmSetMsg(txt) {
    const el = $('kinLlmMsg');
    if (el) el.textContent = txt || '';
  }

  function visionLlmShowConfirm(show) {
    const row = $('kinLlmConfirmRow');
    if (row) row.hidden = !show;
  }

  function visionDepthB64() {
    if (!kinDepth || kinDepth.length !== 640 * 480) return null;
    const sw = 160, sh = 120;
    const small = new Uint16Array(sw * sh);
    for (let y = 0; y < sh; y++) {
      for (let x = 0; x < sw; x++) {
        small[y * sw + x] = kinDepth[(y * 4) * 640 + (x * 4)];
      }
    }
    const bytes = new Uint8Array(small.buffer);
    let bin = '';
    const CH = 0x8000;
    for (let i = 0; i < bytes.length; i += CH) {
      bin += String.fromCharCode.apply(null, bytes.subarray(i, i + CH));
    }
    return btoa(bin);
  }

  async function visionLlmAnalyze(attempt, feedback) {
    if (!kinRgbCanvas || !kinRgbCtx) throw new Error('Kinect RGB non pronta (apri Modalità 2)');
    visionSyncOverlay();
    visionUpdateInfo('LLM analisi… tentativo ' + attempt);
    visionLlmSetMsg('analisi…');

    const dataUrl = kinRgbCanvas.toDataURL('image/jpeg', 0.92);
    const out = await fetchJson('/api/vision/llm/analyze', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ image: dataUrl, attempt, feedback: feedback || '' })
    });
    if (!out || !Array.isArray(out.joints) || out.joints.length < JOINTS_N) {
      throw new Error((out && out.error) || 'risposta LLM senza ' + JOINTS_N + ' giunti');
    }

    const joints = out.joints.slice(0, JOINTS_N).map((j, i) => ({
      label: j.label || JOINT_LABELS[i],
      x: +j.x || 0,
      y: +j.y || 0,
      z: (j.z === undefined || j.z === null) ? null : +j.z,
      color: JOINT_COLORS[i]
    }));

    // z da depth se assente (per scheletro 3D)
    if (kinDepth && kinDepth.length === 640 * 480) {
      const st = (typeof kinStatus !== 'undefined' && kinStatus && kinStatus.depth)
        ? kinStatus.depth : { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };
      for (const j of joints) {
        if (!j.z) {
          const uc = Math.max(0, Math.min(639, Math.round(j.x)));
          const vc = Math.max(0, Math.min(479, Math.round(j.y)));
          let d = kinDepth[vc * 640 + uc];
          if (!d || d <= 0) {
            const win = [];
            for (let dv = -5; dv <= 5; dv++) {
              for (let du = -5; du <= 5; du++) {
                const uu = uc + du, vv = vc + dv;
                if (uu < 0 || vv < 0 || uu >= 640 || vv >= 480) continue;
                const dd = kinDepth[vv * 640 + uu];
                if (dd > 300 && dd < 8000) win.push(dd);
              }
            }
            if (win.length) { win.sort((a, b) => a - b); d = win[win.length >> 1]; }
          }
          if (d) {
            j.z = d;
            const Xk = (j.x - st.cx) * d / st.fx;
            const Yk = (j.y - st.cy) * d / st.fy;
            j.wx = Xk; j.wy = -Yk; j.wz = d;
          }
        } else {
          const st = (typeof kinStatus !== 'undefined' && kinStatus && kinStatus.depth)
            ? kinStatus.depth : { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };
          const Xk = (j.x - st.cx) * j.z / st.fx;
          const Yk = (j.y - st.cy) * j.z / st.fy;
          j.wx = Xk; j.wy = -Yk; j.wz = j.z;
        }
      }
    }

    visionLastMask = null;
    visionConstrainJoints(joints);   // vincolo link rigidi 3D ± tolleranza
    visionLastJoints = joints;
    visionDrawOverlay(joints);
    visionDrawSkeleton3D(joints);
    visionUpdateInfo((out.model || 'llm') + ' · ' + joints.length + ' giunti · t' + attempt);
    visionLlmAttempt = attempt;
    $('kinLlmAttempt').textContent = 'tentativo ' + attempt;
    visionLlmSetMsg(
      (visionLastConstraint
        ? '⚓ ' + visionLastConstraint.d1 + 'mm · ' + visionLastConstraint.d2 + 'mm' +
          (visionLastConstraint.fixed ? ' (corretto)' : '') +
          (visionLastConstraint.auto ? ' [auto]' : '') + ' — '
        : '') + 'corretti?'
    );
    visionLlmShowConfirm(true);
    return joints;
  }

  // Composita RGB pulito + overlay (giunti + linee) → due foto
  async function visionLlmApprove() {
    if (!visionLastJoints || visionLastJoints.length < JOINTS_N) {
      throw new Error('nessuna analisi LLM da approvare');
    }
    visionSyncOverlay();
    visionDrawOverlay(visionLastJoints);

    const imageClean = kinRgbCanvas.toDataURL('image/jpeg', 0.92);

    // annotata: RGB + canvas overlay (giunti + linee di connessione)
    const ann = document.createElement('canvas');
    ann.width = 640; ann.height = 480;
    const actx = ann.getContext('2d');
    actx.drawImage(kinRgbCanvas, 0, 0, 640, 480);
    const ov = $('kinVisionOverlay');
    if (ov) actx.drawImage(ov, 0, 0, 640, 480);
    const imageAnnotated = ann.toDataURL('image/jpeg', 0.95);

    let depthB64 = null;
    try { depthB64 = visionDepthB64(); } catch (e) {}

    let q = null;
    try {
      const st = await fetchJson('/api/esp32/status');
      if (st && Array.isArray(st.q)) q = st.q;
    } catch (e) {}

    const out = await fetchJson('/api/vision/capture-pair', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        imageClean,
        imageAnnotated,
        joints: visionLastJoints.map(j => ({ label: j.label, x: j.x, y: j.y, z: j.z })),
        q,
        depthB64,
        note: 'llm-approve-t' + visionLlmAttempt
      })
    });

    visionCount = out.count || (visionCount + 1);
    visionLoadLinks();   // ricalibra le lunghezze link col nuovo campione approvato
    visionUpdateInfo('salvata doppia foto #' + visionCount + ' · ' + JOINTS_N + ' giunti ok');
    visionLlmSetMsg('✓ salvata');
    visionLlmShowConfirm(false);
    return out;
  }

  /* ---------- trascinamento manuale giunti sull'overlay ---------- */
  let visionDragIdx = -1;

  function visionOverlayEventXY(e) {
    const c = $('kinVisionOverlay');
    const r = c.getBoundingClientRect();
    if (!r.width || !r.height) return null;
    return {
      x: Math.min(639, Math.max(0, (e.clientX - r.left) * 640 / r.width)),
      y: Math.min(479, Math.max(0, (e.clientY - r.top) * 480 / r.height))
    };
  }

  function visionHitJoint(p) {
    if (!visionLastJoints) return -1;
    let best = -1, bestD = 20; // raggio di aggancio in px
    for (let i = 0; i < visionLastJoints.length; i++) {
      const j = visionLastJoints[i];
      const d = Math.hypot(j.x - p.x, j.y - p.y);
      if (d < bestD) { bestD = d; best = i; }
    }
    return best;
  }

  // ricava z dalla depth in (x,y) e ricalcola le coordinate mondo
  /* ---------- vincolo cinematico: link rigidi 3D ± tolleranza ---------- */
  // |BASE-G1| e |G1-G2| sono fisici e costanti: se la distanza 3D (con depth)
  // esce dal range, il giunto esterno viene corretto spostandolo lungo il
  // raggio camera (x,y invariati, solo z aggiornata) fino a rientrare in len.
  let visionLinks = null;      // {L1,L2,tol,count} calibrate sui campioni approvati
  let visionLinksAuto = null;  // riferimento di emergenza: prima rilevazione valida
  let visionLastConstraint = null;

  async function visionLoadLinks() {
    try {
      const j = await fetchJson('/api/vision/links');
      if (j && j.count >= 3 && j.L1 && j.L2) {
        visionLinks = { L1: j.L1, L2: j.L2, tol: j.tol || 50, count: j.count };
      }
    } catch (e) {}
  }

  function visionJoint3D(j, st) {
    const d = j.z || 1500;
    return { x: (j.x - st.cx) * d / st.fx, y: (j.y - st.cy) * d / st.fy, z: d };
  }

  function visionSnapToLink(pa, jb, st, len, tol) {
    const P = visionJoint3D(jb, st);
    const d = Math.hypot(P.x - pa.x, P.y - pa.y, P.z - pa.z);
    if (Math.abs(d - len) <= tol) return { d: Math.round(d), fixed: false };
    // jb = t*ray con ray=((x-cx)/fx,(y-cy)/fy,1) → |t*ray - pa|² = len²
    const rx = (jb.x - st.cx) / st.fx, ry = (jb.y - st.cy) / st.fy;
    const A = rx * rx + ry * ry + 1;
    const B = -2 * (pa.x * rx + pa.y * ry + pa.z);
    const C = pa.x * pa.x + pa.y * pa.y + pa.z * pa.z - len * len;
    const disc = B * B - 4 * A * C;
    if (disc < 0) return { d: Math.round(d), fixed: false };
    const s = Math.sqrt(disc);
    const t1 = (-B + s) / (2 * A), t2 = (-B - s) / (2 * A);
    const cur = jb.z || 1500;                 // radice più vicina allo z attuale
    const t = Math.abs(t1 - cur) <= Math.abs(t2 - cur) ? t1 : t2;
    if (t < 200 || t > 8000) return { d: Math.round(d), fixed: false };
    jb.z = Math.round(t);
    visionReprojectJoint(jb);
    return { d: Math.round(len), fixed: true };
  }

  function visionConstrainJoints(joints) {
    visionLastConstraint = null;
    if (!joints || joints.length < JOINTS_N) return null;
    const st = (typeof kinStatus !== 'undefined' && kinStatus && kinStatus.depth)
      ? kinStatus.depth : { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };
    let L = visionLinks;
    if (!L) {
      // auto-calibrazione di emergenza dalla prima catena valida (tol più larga)
      const P = joints.map(j => visionJoint3D(j, st));
      const d01 = Math.hypot(P[1].x - P[0].x, P[1].y - P[0].y, P[1].z - P[0].z);
      const d12 = Math.hypot(P[2].x - P[1].x, P[2].y - P[1].y, P[2].z - P[1].z);
      if (d01 > 50 && d01 < 2000 && d12 > 50 && d12 < 2000 && !visionLinksAuto) {
        visionLinksAuto = { L1: d01, L2: d12, tol: 100, auto: true };
      }
      L = visionLinksAuto;
    }
    if (!L) return null;
    const P0 = visionJoint3D(joints[0], st);
    const r1 = visionSnapToLink(P0, joints[1], st, L.L1, L.tol);
    const P1 = visionJoint3D(joints[1], st);
    const r2 = visionSnapToLink(P1, joints[2], st, L.L2, L.tol);
    visionLastConstraint = {
      L1: Math.round(L.L1), L2: Math.round(L.L2),
      d1: r1.d, d2: r2.d,
      fixed: (r1.fixed ? 1 : 0) + (r2.fixed ? 1 : 0),
      auto: !!L.auto, count: L.count || 0
    };
    return visionLastConstraint;
  }

  function visionReprojectJoint(j) {
    const st = (typeof kinStatus !== 'undefined' && kinStatus && kinStatus.depth)
      ? kinStatus.depth : { fx: 571.26, fy: 571.26, cx: 320, cy: 240 };
    if (kinDepth && kinDepth.length === 640 * 480) {
      const uc = Math.max(0, Math.min(639, Math.round(j.x)));
      const vc = Math.max(0, Math.min(479, Math.round(j.y)));
      let d = kinDepth[vc * 640 + uc];
      if (!d || d <= 0) {
        const win = [];
        for (let dv = -5; dv <= 5; dv++) {
          for (let du = -5; du <= 5; du++) {
            const uu = uc + du, vv = vc + dv;
            if (uu < 0 || vv < 0 || uu >= 640 || vv >= 480) continue;
            const dd = kinDepth[vv * 640 + uu];
            if (dd > 300 && dd < 8000) win.push(dd);
          }
        }
        if (win.length) { win.sort((a, b) => a - b); d = win[win.length >> 1]; }
      }
      if (d) j.z = d;
    }
    const d = j.z || 1500;
    j.wx = (j.x - st.cx) * d / st.fx;
    j.wy = -(j.y - st.cy) * d / st.fy;
    j.wz = d;
  }

  function visionWireDrag() {
    const c = $('kinVisionOverlay');
    if (!c) return;
    c.addEventListener('pointerdown', e => {
      if (!visionLastJoints || !visionLastJoints.length) return;
      const p = visionOverlayEventXY(e);
      if (!p) return;
      const i = visionHitJoint(p);
      if (i < 0) return;
      visionDragIdx = i;
      c.setPointerCapture(e.pointerId);
      c.style.cursor = 'grabbing';
      e.preventDefault();
    });
    c.addEventListener('pointermove', e => {
      if (visionDragIdx < 0) {
        // cursore a mano quando si passa sopra un giunto
        if (visionLastJoints && visionLastJoints.length) {
          const p = visionOverlayEventXY(e);
          c.style.cursor = (p && visionHitJoint(p) >= 0) ? 'grab' : 'default';
        }
        return;
      }
      const p = visionOverlayEventXY(e);
      if (!p || !visionLastJoints[visionDragIdx]) return;
      const j = visionLastJoints[visionDragIdx];
      j.x = p.x; j.y = p.y;
      visionReprojectJoint(j);
      visionDrawOverlay(visionLastJoints);
      visionDrawSkeleton3D(visionLastJoints);
    });
    const endDrag = e => {
      if (visionDragIdx < 0) return;
      visionDragIdx = -1;
      const c2 = $('kinVisionOverlay');
      if (c2) c2.style.cursor = 'default';
      // rivaluta il vincolo cinematico dopo lo spostamento manuale
      if (visionLastJoints) {
        visionConstrainJoints(visionLastJoints);
        visionDrawOverlay(visionLastJoints);
        visionDrawSkeleton3D(visionLastJoints);
      }
      visionLlmSetMsg('giunto spostato a mano — ora puoi salvare');
    };
    c.addEventListener('pointerup', endDrag);
    c.addEventListener('pointercancel', endDrag);
  }

  // hook nel loop Kinect: dopo ogni frame, se toggle ON, rilevamento ML periodico
  let _visionTick = 0;
  let _visionMlBusy = false;
  function visionMaybeAutoDetect() {
    if (!visionEnabled || !kinDepth) return;
    const now = performance.now();
    // controllo nuovo modello ML ogni ~30 s
    if (now - _mlMtimeTick > 30000) {
      _mlMtimeTick = now;
      visionMlCheckUpdate();
    }
    if (now - _visionTick < 2000) return;  // ML ONNX: ~0.5 Hz (una inferenza ogni 2 s)
    _visionTick = now;
    if (_visionMlBusy) return;
    _visionMlBusy = true;
    visionDetect().catch(e => {
      console.warn('[vision] auto ML:', e.message);
    }).finally(() => { _visionMlBusy = false; });
  }

  // espone hook per kinApplyFrame
  window.__visionOnFrame = visionMaybeAutoDetect;

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', visionWire);
  } else {
    visionWire();
  }

  window.VisionClient = {
    capture: visionCapture,
    detect: visionDetect,
    refresh: visionRefreshCount,
    llmAnalyze: (feedback) => visionLlmAnalyze(visionLlmAttempt, feedback || ''),
    llmApprove: visionLlmApprove,
    get joints() { return visionLastJoints; },
    labels: JOINT_LABELS
  };
})();
