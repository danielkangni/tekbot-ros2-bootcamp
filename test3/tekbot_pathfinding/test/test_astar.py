"""Tests unitaires de l'algorithme (sans ROS) : python3 -m pytest tekbot_pathfinding/test"""
import math
import os
import time

import numpy as np

from tekbot_pathfinding import astar

HERE = os.path.dirname(os.path.abspath(__file__))


def empty(h=20, w=20):
    return np.zeros((h, w), dtype=bool)


def test_straight_line():
    path, _ = astar.astar(empty(), (0, 0), (10, 0))
    assert path[0] == (0, 0) and path[-1] == (10, 0)
    assert len(path) == 11


def test_diagonal_is_octile_optimal():
    path, _ = astar.astar(empty(), (0, 0), (5, 5))
    assert math.isclose(astar.path_length(path, 1.0), 5 * math.sqrt(2))


def test_goes_around_wall():
    g = empty()
    g[:15, 10] = True                      # mur vertical avec ouverture en haut
    path, _ = astar.astar(g, (2, 5), (18, 5))
    assert path is not None and all(not g[j, i] for i, j in path)
    assert max(j for i, j in path) >= 15


def test_no_path():
    g = empty()
    g[:, 10] = True
    path, _ = astar.astar(g, (2, 5), (18, 5))
    assert path is None


def test_no_corner_cutting():
    g = empty(5, 5)
    g[1, 2] = g[2, 1] = True               # diagonale infranchissable entre (1,1) et (2,2)
    g[1, 1] = False
    path, _ = astar.astar(g, (1, 1), (2, 2))
    assert path is None or len(path) > 2


def test_optimality_vs_dijkstra():
    rng = np.random.default_rng(0)
    for _ in range(20):
        g = rng.random((25, 25)) < 0.25
        g[0, 0] = g[24, 24] = False
        p_a, _ = astar.astar(g, (0, 0), (24, 24))
        p_d, _ = dijkstra(g, (0, 0), (24, 24))
        assert (p_a is None) == (p_d is None)
        if p_a:
            assert math.isclose(astar.path_length(p_a, 1), astar.path_length(p_d, 1), rel_tol=1e-9)


def dijkstra(blocked, start, goal):
    """A* avec heuristique nulle = Dijkstra (reference)."""
    old = astar.octile
    astar.octile = lambda a, b: 0.0
    try:
        return astar.astar(blocked, start, goal)
    finally:
        astar.octile = old


def load_maze(radius):
    raw = open(os.path.join(HERE, '..', 'maps', 'maze_map.pgm'), 'rb').read()
    parts = raw.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3], dtype=np.uint8).reshape(h, w)
    occ = np.flipud(img) < 90
    blocked, dist = astar.inflate(occ, 0.05, radius)
    return occ, blocked, dist


def cell(x, y):
    return int((x + 3.4510) / 0.05), int((y + 3.5173) / 0.05)


def test_real_maze_crossing():
    occ, blocked, dist = load_maze(0.30)
    s, g = cell(-2.44, -2.5), cell(2.56, 2.49)
    t = time.perf_counter()
    path, expanded = astar.astar(blocked, s, g, 0.05, dist, 0.6, 3.0)
    dt = time.perf_counter() - t
    assert path is not None
    assert all(not blocked[j, i] for i, j in path)
    print(f'maze: {len(path)} cellules, {astar.path_length(path, 0.05):.2f} m, {expanded} noeuds, {dt*1000:.0f} ms')
