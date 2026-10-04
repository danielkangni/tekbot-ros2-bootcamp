#!/usr/bin/env python3
"""Genere une carte d'occupation (PGM + YAML, format map_server) a partir du monde Gazebo.

Le labyrinthe est un modele SDF compose d'<include> (panneaux, poteaux) poses sur une grille.
Pour chaque <collision><box> on compose les poses (monde -> include -> link -> collision),
on projette la boite sur le plan XY et on rasterise le polygone.

Usage : world_to_map.py [--maze-model DIR] [--resolution 0.05] [--out maps/maze_map]
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
MAZE_PKG = os.path.join(HERE, '..', '..', 'tekbot_sim', 'maze_solving')


def pose_matrix(text):
    """'x y z roll pitch yaw' (SDF) -> matrice homogene 4x4 (R = Rz*Ry*Rx)."""
    x, y, z, r, p, w = [float(v) for v in (text or '0 0 0 0 0 0').split()]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(w), math.sin(w)
    rot = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = [x, y, z]
    return m


def pose_of(elem):
    p = elem.find('pose')
    return pose_matrix(p.text if p is not None else None)


def box_footprints(model_dir, base):
    """Liste de polygones XY (monde) pour toutes les boites de collision d'un modele."""
    root = ET.parse(os.path.join(model_dir, 'model.sdf')).getroot()
    polys = []
    for link in root.iter('link'):
        t_link = base @ pose_of(link)
        for col in link.findall('collision'):
            box = col.find('geometry/box/size')
            if box is None:
                continue
            sx, sy, sz = [float(v) for v in box.text.split()]
            t = t_link @ pose_of(col)
            corners = np.array([[a * sx / 2, b * sy / 2, c * sz / 2, 1.0]
                                for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
            pts = (t @ corners.T).T[:, :2]
            polys.append((pts, (t @ corners.T).T[:, 2].min(), (t @ corners.T).T[:, 2].max()))
    return polys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--maze-model', default=os.path.join(MAZE_PKG, 'models', 'maze_4_metal_6x6'))
    ap.add_argument('--models-dir', default=os.path.join(MAZE_PKG, 'models'))
    ap.add_argument('--world-pose', default='-2.94084 -3.0073 0 0 0 0',
                    help='pose du modele du labyrinthe dans le monde (voir le .world)')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=0.5)
    ap.add_argument('--out', default=os.path.join(HERE, '..', 'maps', 'maze_map'))
    args = ap.parse_args()

    t_world = pose_matrix(args.world_pose)
    maze = ET.parse(os.path.join(args.maze_model, 'model.sdf')).getroot().find('model')
    polys = []
    for inc in maze.findall('include'):
        name = inc.find('uri').text.replace('model://', '')
        polys += box_footprints(os.path.join(args.models_dir, name), t_world @ pose_of(inc))
    # on ne garde que les obstacles qui coupent le plan du lidar (z ~ 0.1-0.3 m)
    polys = [p for p, zmin, zmax in polys if zmin < 0.3 < zmax or zmin < 0.1 < zmax]

    allp = np.vstack(polys)
    xmin, ymin = allp.min(axis=0) - args.margin
    xmax, ymax = allp.max(axis=0) + args.margin
    res = args.resolution
    w, h = int(math.ceil((xmax - xmin) / res)), int(math.ceil((ymax - ymin) / res))

    img = Image.new('L', (w, h), 254)           # 254 = libre
    draw = ImageDraw.Draw(img)
    for p in polys:
        # coordonnees pixel : x vers la droite, y vers le haut (inverse de l'image)
        px = [((x - xmin) / res, h - (y - ymin) / res) for x, y in p]
        hull = convex_hull(px)
        draw.polygon(hull, fill=0, outline=0)    # murs fins (2 cm) : le contour garantit >= 1 pixel
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    img.save(out + '.pgm')
    img.resize((w * 4, h * 4), Image.NEAREST).save(out + '_preview.png')
    with open(out + '.yaml', 'w') as f:
        f.write(f"image: {os.path.basename(out)}.pgm\nmode: trinary\nresolution: {res}\n"
                f"origin: [{xmin:.4f}, {ymin:.4f}, 0.0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f"{len(polys)} obstacles, carte {w}x{h} px, origine ({xmin:.2f}, {ymin:.2f}) m -> {out}.pgm")


def convex_hull(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


if __name__ == '__main__':
    main()
