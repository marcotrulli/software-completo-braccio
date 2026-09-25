#!/usr/bin/env python3
"""Training keypoint model: fine-tune di YOLOv8n-pose (pretrain COCO) sul dataset.

Perché è potente già da subito:
- transfer learning da COCO (il modello sa già estrarre forme/arti)
- augmentation (flip/rotazione/scale/HSV) per non memorizzare
- solo 3 keypoint: compito facile, converge con pochi campioni

Alla fine esporta ONNX in vision/model/model.keypoints.onnx (slot RobotVisionLocalAI).

Uso: python train.py [--epochs 150] [--data yolo_dataset] [--model yolov8n-pose.pt]
"""
import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
MODEL_OUT = ROOT.parent / 'model'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--epochs', type=int, default=150)
    ap.add_argument('--data', default=str(ROOT / 'yolo_dataset'))
    ap.add_argument('--model', default='yolov8n-pose.pt')
    args = ap.parse_args()

    data_dir = Path(args.data)
    if not (data_dir / 'images').exists():
        raise SystemExit(f'dataset non trovato: {data_dir} — esegui prima export_dataset.py')

    yaml_path = data_dir / 'data.yaml'
    yaml_path.write_text(
        'path: .\n'
        'train: images\n'
        'val: images\n'
        'kpt_shape: [3, 3]\n'
        'flip_idx: [0, 1, 2]\n'
        "names: ['robot']\n",
        encoding='utf-8',
    )

    model = YOLO(args.model)
    model.train(
        data=str(yaml_path),
        epochs=args.epochs,
        imgsz=640,
        batch=8,
        patience=40,
        hsv_h=0.015,
        hsv_s=0.5,
        hsv_v=0.4,
        degrees=10,
        translate=0.1,
        scale=0.2,
        fliplr=0.5,
        mosaic=1.0,
    )

    best = model.export(format='onnx', imgsz=640)
    MODEL_OUT.mkdir(parents=True, exist_ok=True)
    target = MODEL_OUT / 'model.keypoints.onnx'
    shutil.copy2(best, target)
    print(f'✅ ONNX esportato → {target}')
    print('   Il client lo rileva automaticamente via /api/vision/model/status')


if __name__ == '__main__':
    main()
