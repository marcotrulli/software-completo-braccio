# VISION — Riconoscimento braccio robotico (ML)

Data creazione: **22.09.2026**

## Cos'è

Sottosistema che usa la **Kinect** per:
1. **Raccogliere foto** nel dataset ML (pulsante 📷 *Scatta foto* nel pannello Kinect)
2. **Riconoscere 3 giunti reali** del braccio (BASE, G1 = fine braccio 1, G2 = fine braccio 2) — 6 motori → 3 giunti visibili; la PINZA arriverà come 4° punto quando progettata
3. **Disegnare** l'ossatura su video RGB e come scheletro 3D nella nuvola
4. **Correzione manuale**: i giunti si possono trascinare col mouse sull'overlay RGB (il 3D si aggiorna in tempo reale)

## Note tecniche (agg. 22.09 sera)

- **Convenzione orientamento ("Specchio")**: il video RGB è mostrato specchiato (`kinMirror`, default ON). Il frame specchiato è ora la convenzione ovunque: canvas analisi (`kinRgbCtx` disegna con `scaleX(-1)`), depth (flippata in `kinFetchDepth`), nuvola 3D e scheletro giunti (X ribaltata in `kinRebuildCloud`/`visionDrawSkeleton3D`), misura click (`/map` mx ribaltato). Overlay giunti e foto salvate sono quindi sempre allineati con ciò che si vede
- **Auto = LLM**: la modalità Auto (checkbox) ora chiama l'LLM ogni ~6 s (prima usava il baseline depth, risultati casuali); resta in pausa mentre attendi la conferma ✅/❌
- **Trascinamento giunti**: con i giunti visualizzati, trascina i pallini sull'overlay RGB; la profondità (z) viene riletta dalla depth in tempo reale e lo scheletro 3D si aggiorna — poi salva con ✅
- **Conteggi generici**: `JOINTS_N`/`VISION_JOINTS_N` in client/server — per aggiungere la PINZA basta aggiungere l'etichetta agli array `JOINT_LABELS` / `VISION_JOINT_LABELS`

## Struttura

```
vision/
├── client.js          ← logica nel browser (capture, detect, overlay)
├── dataset/           ← foto .jpg + meta .json + depth .bin (auto-creata)
├── annotations/       ← (riservato) export label per training
├── model/             ← qui va il modello IA locale
│                         es. model.keypoints.onnx
└── README.md          ← questo file
```

## ML — training keypoint model (cartella `ml/`)

- **Vincolo cinematico attivo**: i link BASE-G1 e G1-G2 sono rigidi; il client corregge
  automaticamente lo z del giunto esterno (lungo il raggio camera) se la distanza 3D
  (con depth) esce da L ± 50 mm. Le lunghezze L1/L2 si **auto-calibrano** sui campioni
  approvati (`GET /api/vision/links`); finché non ce ne sono ≥3, usa la prima catena
  valida come riferimento (tol ±100 mm, indicato con `[auto]`). Il readout ⚓/⚠ è
  disegnato sull'overlay e nel messaggio di conferma
- **Pipeline**: `export_dataset.py` → YOLO-pose → `train.py` (fine-tune yolov8n-pose,
  pretrain COCO, augmentation) → ONNX in `model/model.keypoints.onnx`
- Vedi `ml/README.md` per setup e dettagli

## API server

| Endpoint | Metodo | Uso |
|---|---|---|
| `/api/vision/capture` | POST | Salva foto (+ depth downsample + q[] ESP32) |
| `/api/vision/dataset` | GET | Elenca capture |
| `/api/vision/image/:id` | GET | JPEG |
| `/api/vision/depth/:id` | GET | depth binario |
| `/api/vision/annotate/:id` | POST | Salva 7 giunti annotati |
| `/api/vision/model/status` | GET | Stato modello locale |
| `/api/vision/llm/status` | GET | Stato LLM visione locale (Bionic/lms :5000) |
| `/api/vision/llm/analyze` | POST | Analizza frame → 7 giunti (LLM visione) |
| `/api/vision/capture-pair` | POST | Salva DOPPIA foto: pulita + annotata (dataset ML) |

## LLM visione locale (in uso da 22.09.2026)

- **Modello**: `qwen2.5-vl-7b-instruct` (Qwen2.5-VL-7B Q4_K_M + mmproj Q8_0) in `D:\LMStudioModels\Qwen2.5-VL-7B-Instruct-GGUF`
- **Runtime**: Bionic (motore LM Studio) in servizio su `127.0.0.1:5000/v1` — *Bionic va riavviato per vedere modelli appena scaricati*
- **Fallback automatici**: qwen3-vl-4b → gemma-3-4b (sempre <= 8B)
- **Override**: variabile env `VISION_LLM_MODEL`
- **Prompt**: formato nativo `point:[x,y]` (migliore grounding su Qwen2.5-VL)
- **Flusso**: `🤖 LLM Analizza` → overlay 2D+3D → `✅ Sì, corretti` salva coppia pulita+annotata · `❌ No, riprova` ri-analizza con feedback
- Giunti non perfetti al primo tentativo sono normali: il ciclo conferma/riprova filtra le etichette per il training ML

## Pipeline inferenza (client)

```
frame Kinect (RGB + depth)
        │
        ▼
window.RobotVisionLocalAI ?  ──SÌ──►  IA locale (ONNX/TFLite)
        │ NO
        ▼
baseline depth (segmentazione + PCA + 7 punti)
        │
        ▼
overlay 2D su RGB  +  scheletro 3D in kinScene
```

## Come inserire l'IA LOCALE (prossimo passo)

Nel browser, prima dell'inference:

```js
window.RobotVisionLocalAI = async ({ imageData, depth, width, height, labels }) => {
  // carica ONNX con onnxruntime-web, oppure tflite, oppure worker Python
  // return {
  //   joints: [ {x,y,z,label}, ... x7 ],  // x,y px; z mm se noto
  //   backend: 'my-local-ai'
  // };
};
```

Il file del modello va in `vision/model/` (rilevato da `/api/vision/model/status`).

## Allenamento (ordine consigliato)

1. Modalità 2 Kinect → **Scatta foto** in varie pose (obiettivo: 200–500)
2. Eventuale annotazione manuale dei 7 punti (`/api/vision/annotate/:id`)
3. Train keypoint model (YOLO-pose / Simple-Baseline) sul dataset
4. Esporta ONNX in `vision/model/`
5. Collega `RobotVisionLocalAI` → sostituisce il baseline

## Dipendenze

- Nessuna npm extra per il baseline
- Per IA locale: `onnxruntime-web` (CDN o npm) — da aggiungere quando si carica il modello
