# ML — Training keypoint model (3 giunti)

Pipeline: **dataset approvato → export YOLO-pose → fine-tune YOLOv8n-pose → ONNX → client**

## Setup (una volta)

```bat
cd vision\ml
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

## Flusso

1. **Raccogli** coppie approvate con ✅ nell'app (obiettivo minimo: ~30-50 pose diverse)
2. **Esporta**: `python export_dataset.py` → `yolo_dataset/` (images + labels)
3. **Train**: `python train.py` — fine-tune di yolov8n-pose (pretrain COCO), augmentation inclusa; la RTX 5060 Ti ci mette pochi minuti
4. **Output**: ONNX in `vision/model/model.keypoints.onnx`, rilevato da `/api/vision/model/status`

## Perché è potente già da subito

- **Transfer learning**: parte da un modello già allenato su COCO (sa riconoscere strutture/arti)
- **3 keypoint solo**: problema facile, converge con pochi campioni
- **Augmentation**: flip, rotazione ±10°, scale, variazioni colore — il modello non memorizza
- **Vincolo cinematico** (link rigidi ±5 cm): già attivo in inferenza nel client; si applicherà anche alle previsioni ML

## Pseudo-labeling (opzionale, per scalare veloce)

L'LLM (Qwen2.5-VL-7B) può etichettare automaticamente molti frame; tu approvi solo
quelli buoni con ✅. Così il dataset cresce in fretta senza annotare a mano.

## Auto-retrain (attivo di default)

Il server lancia da solo `auto_train.py` (export → train → ONNX) ogni volta che
i campioni approvati crescono di **20** dall'ultimo training (`VISION_ML_AUTO_MIN`
per cambiarla, `VISION_ML_AUTO=0` per disattivare). Il browser ricarica il nuovo
ONNX da solo (controllo mtime ogni 30 s) — **non serve mai refresh né comandi**.

- Stato: `GET /api/vision/ml/auto` · forza retrain: `POST /api/vision/ml/retrain`
- Log: `vision/ml/auto_train.log`

## Nota

Il vincolo link-rigidi lavora in 3D con la depth, quindi resta valido anche per
le previsioni del modello ML (corregge lo z lungo il raggio camera).
