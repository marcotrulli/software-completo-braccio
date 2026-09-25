#!/usr/bin/env python3
"""Test rapido inference ONNX su un'immagine del dataset (render dei keypoint).

Usa il parser ufficiale ultralytics (stesso identico al training/val).
Uso: python infer_test.py [immagine.jpg] [render_out.jpg]
"""
import sys
import json
from pathlib import Path

from PIL import Image, ImageDraw
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
MODEL = ROOT.parent / 'model' / 'model.keypoints.onnx'

img_path = sys.argv[1] if len(sys.argv) > 1 else None
out_path = sys.argv[2] if len(sys.argv) > 2 else 'infer_render.jpg'
gt = None
if not img_path:
    metas = sorted((ROOT.parent / 'dataset').glob('*.json'))
    for mf in reversed(metas):
        m = json.loads(mf.read_text(encoding='utf-8'))
        if m.get('approved') and len(m.get('joints') or []) == 3:
            img_path = str(ROOT.parent / 'dataset' / m['image'])
            gt = m['joints']
            break

model = YOLO(str(MODEL))
r = model.predict(img_path, imgsz=640, verbose=False)[0]

img = Image.open(img_path).convert('RGB')
d = ImageDraw.Draw(img)
if gt:
    for j in gt:
        d.ellipse([j['x'] - 7, j['y'] - 7, j['x'] + 7, j['y'] + 7], outline=(255, 255, 0), width=2)

if r.keypoints is not None and len(r.keypoints.xy):
    kpts = r.keypoints.xy[0].cpu().numpy()
    confs = r.keypoints.conf[0].cpu().numpy() if r.keypoints.conf is not None else [1] * len(kpts)
    cols = [(255, 90, 90), (90, 230, 199), (92, 157, 255)]
    for i, (p, c) in enumerate(zip(kpts, confs)):
        px, py = float(p[0]), float(p[1])
        print(f'G{i}: x={px:.0f} y={py:.0f} conf={float(c):.2f}')
        d.ellipse([px - 8, py - 8, px + 8, py + 8], fill=cols[i % 3], outline=(0, 0, 0))
        d.text((px + 10, py - 6), f'G{i}', fill=(255, 255, 255))
else:
    print('NESSUNA rilevazione')

img.save(out_path, quality=95)
print('render ->', out_path, '(cerchi gialli = ground truth approvata)')
