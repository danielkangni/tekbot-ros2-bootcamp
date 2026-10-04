"""Pathfinding sur grille d'occupation : A* (8-connexe), sans dependance ROS.

Choix de conception
-------------------
* Grille ``blocked[j, i]`` (j = ligne vers le haut comme le repere monde, i = colonne).
* Les obstacles sont gonfles du rayon du robot (``inflate``) : le robot est ensuite traite comme un point.
* Cout d'un pas = longueur * (1 + penalite), la penalite croit quand on s'approche d'un mur :
  le chemin reste au milieu des couloirs au lieu de raser les murs.
* Heuristique octile (exacte sans obstacle, admissible car la penalite ne fait que *augmenter* les couts).
* Pas de coupe de coin en diagonale (les deux cases orthogonales doivent etre libres).
"""
import heapq
import math

import numpy as np

SQRT2 = math.sqrt(2.0)
NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def distance_transform(occupied, resolution):
    """Distance (m) de chaque case a l'obstacle le plus proche (chamfer 3-4, 2 passes)."""
    h, w = occupied.shape
    big = 1e9
    d = np.where(occupied, 0.0, big)
    for j in range(h):                       # passe avant
        for i in range(w):
            v = d[j, i]
            if v == 0.0:
                continue
            if i > 0:
                v = min(v, d[j, i - 1] + 1.0)
            if j > 0:
                v = min(v, d[j - 1, i] + 1.0)
                if i > 0:
                    v = min(v, d[j - 1, i - 1] + SQRT2)
                if i < w - 1:
                    v = min(v, d[j - 1, i + 1] + SQRT2)
            d[j, i] = v
    for j in range(h - 1, -1, -1):           # passe arriere
        for i in range(w - 1, -1, -1):
            v = d[j, i]
            if v == 0.0:
                continue
            if i < w - 1:
                v = min(v, d[j, i + 1] + 1.0)
            if j < h - 1:
                v = min(v, d[j + 1, i] + 1.0)
                if i < w - 1:
                    v = min(v, d[j + 1, i + 1] + SQRT2)
                if i > 0:
                    v = min(v, d[j + 1, i - 1] + SQRT2)
            d[j, i] = v
    return d * resolution


def inflate(occupied, resolution, radius):
    """Gonfle les obstacles de ``radius`` metres. Renvoie (blocked, dist_m)."""
    dist = distance_transform(occupied, resolution)
    return dist <= radius, dist


def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def nearest_free(blocked, cell, max_cells):
    """Case libre la plus proche (BFS) -- utile si le robot est tres pres d'un mur."""
    h, w = blocked.shape
    i0, j0 = cell
    if 0 <= i0 < w and 0 <= j0 < h and not blocked[j0, i0]:
        return cell
    best, best_d = None, max_cells + 1
    for dj in range(-max_cells, max_cells + 1):
        for di in range(-max_cells, max_cells + 1):
            i, j = i0 + di, j0 + dj
            if 0 <= i < w and 0 <= j < h and not blocked[j, i]:
                dd = math.hypot(di, dj)
                if dd < best_d:
                    best, best_d = (i, j), dd
    return best


def astar(blocked, start, goal, resolution=1.0, dist=None, soft_radius=0.0, wall_weight=0.0):
    """A* entre deux cases (i, j).

    Renvoie (chemin [(i, j), ...] ou None, nombre de noeuds developpes).
    ``dist`` (metres) + ``soft_radius`` + ``wall_weight`` activent la penalite de proximite des murs.
    """
    h, w = blocked.shape
    si, sj = start
    gi, gj = goal
    if blocked[sj, si] or blocked[gj, gi]:
        return None, 0
    penalty = None
    if dist is not None and wall_weight > 0.0 and soft_radius > 0.0:
        penalty = wall_weight * np.clip((soft_radius - dist) / soft_radius, 0.0, 1.0)

    g = {start: 0.0}
    parent = {}
    open_heap = [(octile(start, goal), 0.0, start)]
    closed = set()
    expanded = 0
    while open_heap:
        _, gc, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path, expanded
        closed.add(cur)
        expanded += 1
        ci, cj = cur
        for di, dj, step in NEIGHBORS:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < w and 0 <= nj < h) or blocked[nj, ni]:
                continue
            if di != 0 and dj != 0 and (blocked[cj, ni] or blocked[nj, ci]):
                continue                      # pas de coupe de coin
            nxt = (ni, nj)
            if nxt in closed:
                continue
            cost = step * (1.0 + (penalty[nj, ni] if penalty is not None else 0.0))
            ng = gc + cost
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(open_heap, (ng + octile(nxt, goal), ng, nxt))
    return None, expanded


def line_of_sight(blocked, a, b):
    """Vrai si le segment a->b (Bresenham) ne traverse aucune case bloquee."""
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx - dy
    while True:
        if blocked[y0, x0]:
            return False
        if (x0, y0) == (x1, y1):
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def segment_clearance(dist, a, b):
    """Plus petite distance aux obstacles (m) le long du segment a->b (Bresenham)."""
    (x0, y0), (x1, y1) = a, b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx - dy
    low = math.inf
    while True:
        low = min(low, dist[y0, x0])
        if (x0, y0) == (x1, y1):
            return low
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def smooth(blocked, path, dist=None, clearance=0.0):
    """Raccourcit le chemin : saute vers le point le plus lointain visible depuis le point courant.

    Si ``dist`` est fourni, un raccourci n'est accepte que s'il ne passe pas plus pres des murs que
    ``clearance`` metres (ou que ses extremites ne le sont deja) : le chemin lisse reste au large des coins.
    """
    if len(path) < 3:
        return list(path)

    def ok(a, b):
        if not line_of_sight(blocked, a, b):
            return False
        if dist is None:
            return True
        return segment_clearance(dist, a, b) >= min(clearance, dist[a[1], a[0]], dist[b[1], b[0]]) - 1e-9

    out = [path[0]]
    k = 0
    while k < len(path) - 1:
        far = len(path) - 1
        while far > k + 1 and not ok(path[k], path[far]):
            far -= 1
        out.append(path[far])
        k = far
    return out


def path_length(cells, resolution):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(cells, cells[1:])) * resolution
