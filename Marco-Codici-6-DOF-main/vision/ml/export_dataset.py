#!/usr/bin/env python3
"""Esporta il dataset approvato (vision/dataset) in formato YOLO-pose.

Ogni campione approvato con 3 giunti (BASE, G1, G2) e z valido diventa:
  yolo_dataset/images/<id>.jpg
  yolo_dataset/labels/<id>.txt   → class cx cy w h x1 y1 v1 x2 y2 v2 x3 y3 v3 (normalizzati)

Uso: python export_dataset.py [--out yolo_dataset]
"""
import argparse
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DS = ROOT.parent / 'dataset'
LABELS = ['BASE', 'G1', 'G2']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(ROOT / 'yolo_dataset'))
    args = ap.parse_args()
    out = Path(args.out)
    img_dir = out / 'images'
    lbl_dir = out / 'labels'
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    n = 0
    skipped = 0
    for meta_file in sorted(DS.glob('*.json')):
        try:
            meta = json.loads(meta_file.read_text(encoding='utf-8'))
        except Exception:
            skipped += 1
            continue
        joints = meta.get('joints') or []
        if not meta.get('approved') or len(joints) != len(LABELS):
            skipped += 1
            continue
        if any(not j or j.get('z') is None for j in joints):
            skipped += 1
            continue
        img = DS / meta['image']
        if not img.exists():
            skipped += 1
            continue

        xs = [float(j['x']) for j in joints]
        ys = [float(j['y']) for j in joints]
        m = 25  # margine bbox in px
        x1 = max(0.0, min(xs) - m)
        y1 = max(0.0, min(ys) - m)
        x2 = min(639.0, max(xs) + m)
        y2 = min(479.0, max(ys) + m)
        cx, cy = (x1 + x2) / 2 / 640, (y1 + y2) / 2 / 480
        w, h = (x2 - x1) / 640, (y2 - y1) / 480

        parts = [f'0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}']
        for j in joints:
            parts.append(f"{float(j['x']) / 640:.6f} {float(j['y']) / 480:.6f} 2")
        name = meta['id']
        shutil.copy2(img, img_dir / f'{name}.jpg')
        (lbl_dir / f'{name}.txt').write_text(' '.join(parts), encoding='utf-8')
        n += 1

    print(f'esportati: {n} campioni · saltati: {skipped} → {out}')
    if n < 10:
        print('NOTA: meno di 10 campioni — raccogli altre coppie approvate prima del training.')


if __name__ == '__main__':
    main()
