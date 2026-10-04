"""Noeud de planification : carte + but (RViz '2D Goal Pose') -> chemin A* sur /planned_path.

Entrees : /goal_pose (PoseStamped), /odom (pose du robot, repere monde), /scan (obstacles imprevus), /replan
Sorties : /map, /inflated_map, /maze_markers, /planned_path, /planner_status
"""
import math
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker

from tekbot_pathfinding import astar


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_map(yaml_path):
    cfg = yaml.safe_load(open(yaml_path))
    raw = open(os.path.join(os.path.dirname(yaml_path), cfg['image']), 'rb').read()
    parts = raw.split(b'\n', 3)                       # P5 / largeur hauteur / maxval / donnees
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3], dtype=np.uint8).reshape(h, w)
    p = img / 255.0 if cfg.get('negate', 0) else (255 - img) / 255.0
    occupied = np.flipud(p > cfg['occupied_thresh'])  # ligne 0 = bas de la carte (repere monde)
    return occupied, float(cfg['resolution']), cfg['origin'][:2]


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        default_map = os.path.join(get_package_share_directory('tekbot_pathfinding'), 'maps', 'maze_map.yaml')
        self.declare_parameter('map_yaml', default_map)
        self.declare_parameter('inflation_radius', 0.30)   # > demi-diagonale du robot (0.50 x 0.25 m -> 0.28 m)
        self.declare_parameter('soft_radius', 0.60)        # penalite de proximite des murs jusqu'a cette distance
        self.declare_parameter('wall_weight', 3.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('replan_period', 0.5)
        self.declare_parameter('dynamic_ttl', 120.0)       # duree max de memorisation d'un obstacle detecte au lidar (s)
        self.declare_parameter('lidar_offset', 0.10)       # position du lidar devant base_footprint (m)
        self.declare_parameter('los_clearance', 0.40)      # marge visee par le lissage (m)
        self.declare_parameter('smooth', True)
        g = self.get_parameter
        self.inflation = g('inflation_radius').value
        self.soft = g('soft_radius').value
        self.wall_w = g('wall_weight').value
        self.tol = g('goal_tolerance').value
        self.ttl = g('dynamic_ttl').value
        self.lidar_off = g('lidar_offset').value
        self.do_smooth = g('smooth').value
        self.los_clearance = g('los_clearance').value

        self.occ, self.res, self.origin = load_map(g('map_yaml').value)
        self.h, self.w = self.occ.shape
        self.blocked_static, self.dist_static = astar.inflate(self.occ, self.res, self.inflation)
        self.get_logger().info(f'Carte {self.w}x{self.h} @ {self.res} m, origine {self.origin}, '
                               f'{int(self.occ.sum())} cellules occupees, gonflage {self.inflation} m')

        self.pose = None            # (x, y, yaw) dans le repere monde/map
        self.yaw_rate = 0.0
        self.scan = None
        self.goal = None            # (x, y)
        self.path_cells = None
        self.dynamic = {}           # (i, j) -> instant d'expiration
        self.last_plan_ok = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.inf_pub = self.create_publisher(OccupancyGrid, '/inflated_map', latched)
        self.mk_pub = self.create_publisher(Marker, '/maze_markers', latched)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.status_pub = self.create_publisher(String, '/planner_status', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Empty, '/replan', lambda _: self.plan('replan demande'), 10)
        self.create_timer(g('replan_period').value, self.tick)
        self.publish_maps()

    # ---------- conversions ----------
    def world_to_cell(self, x, y):
        return int(math.floor((x - self.origin[0]) / self.res)), int(math.floor((y - self.origin[1]) / self.res))

    def cell_to_world(self, i, j):
        return self.origin[0] + (i + 0.5) * self.res, self.origin[1] + (j + 0.5) * self.res

    # ---------- callbacks ----------
    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        self.yaw_rate = m.twist.twist.angular.z

    def on_scan(self, m):
        self.scan = m

    def on_goal(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y)
        self.dynamic.clear()
        self._blocked_cache = None
        self.get_logger().info(f'Nouveau but : ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
        self.plan('nouveau but')

    # ---------- obstacles imprevus (lidar) ----------
    def update_dynamic(self):
        """Memorise les impacts lidar qui ne correspondent a rien dans la carte statique.

        Un obstacle n'est oublie que lorsque le lidar *voit au-dela* de sa position (il a disparu) ou apres
        ``dynamic_ttl`` secondes : sinon le planificateur oscillerait entre le detour et le chemin direct
        des que le robot cesse de regarder l'obstacle.
        """
        now = time.monotonic()
        if self.scan is None or self.pose is None:
            return
        x, y, yaw = self.pose
        lx, ly = x + self.lidar_off * math.cos(yaw), y + self.lidar_off * math.sin(yaw)
        s = self.scan
        n = len(s.ranges)
        before = set(self.dynamic)
        # 1) oubli : cellule observee libre (le rayon passe nettement derriere elle)
        for (i, j), t in list(self.dynamic.items()):
            cx, cy = self.cell_to_world(i, j)
            d = math.hypot(cx - lx, cy - ly)
            if t < now:
                del self.dynamic[(i, j)]
            elif 0.3 < d < 2.5:
                a = wrap(math.atan2(cy - ly, cx - lx) - yaw)
                k = int(round((a - s.angle_min) / s.angle_increment)) % n
                rays = [s.ranges[(k + o) % n] for o in (-2, -1, 0, 1, 2)]
                if min(r if math.isfinite(r) else s.range_max for r in rays) > d + 0.35:
                    del self.dynamic[(i, j)]
        # 2) ajout : impacts loin de tout mur connu. Pendant une rotation rapide, le scan (10 Hz) et l'odometrie
        #    ne sont plus synchrones : les murs glissent de quelques cm et ressembleraient a des obstacles.
        if abs(self.yaw_rate) > 0.25:
            if set(self.dynamic) != before:
                self._blocked_cache = None
            return
        for k, r in enumerate(s.ranges):
            if not math.isfinite(r) or r < s.range_min + 0.05 or r > min(s.range_max, 2.5):
                continue
            a = yaw + s.angle_min + k * s.angle_increment
            i, j = self.world_to_cell(lx + r * math.cos(a), ly + r * math.sin(a))
            if 0 <= i < self.w and 0 <= j < self.h and self.dist_static[j, i] > 0.25:
                if (i, j) not in self.dynamic:
                    self.get_logger().info(
                        f'obstacle detecte en ({lx + r * math.cos(a):.2f},{ly + r * math.sin(a):.2f}) '
                        f'[mur connu a {self.dist_static[j, i]:.2f} m, robot ({x:.2f},{y:.2f}) distance {r:.2f} m, '
                        f'angle {math.degrees(a - yaw):.0f} deg]', throttle_duration_sec=0.2)
                self.dynamic[(i, j)] = now + self.ttl
        if set(self.dynamic) != before:
            self._blocked_cache = None

    def current_blocked(self):
        """(blocked, dist) : carte statique + obstacles detectes au lidar, gonflee du rayon du robot."""
        if not self.dynamic:
            return self.blocked_static, self.dist_static
        if getattr(self, '_blocked_cache', None) is None:
            occ = self.occ.copy()
            for i, j in self.dynamic:
                occ[j, i] = True
            self._blocked_cache = astar.inflate(occ, self.res, self.inflation)
        return self._blocked_cache

    # ---------- planification ----------
    def plan(self, reason=''):
        if self.goal is None or self.pose is None:
            return
        blocked, dist = self.current_blocked()
        s = astar.nearest_free(blocked, self.world_to_cell(*self.pose[:2]), int(0.4 / self.res))
        gl = astar.nearest_free(blocked, self.world_to_cell(*self.goal), int(0.3 / self.res))
        if s is None or gl is None:
            self.status('ECHEC : depart ou but dans un obstacle')
            self.last_plan_ok = False
            return
        t0 = time.perf_counter()
        cells, expanded = astar.astar(blocked, s, gl, self.res, dist, self.soft, self.wall_w)
        dt = (time.perf_counter() - t0) * 1000
        if cells is None:
            self.status(f'ECHEC : aucun chemin ({reason}, {expanded} noeuds, {dt:.0f} ms)')
            self.path_cells = None
            self.last_plan_ok = False
            self.publish_path([])
            return
        raw_len = astar.path_length(cells, self.res)
        if self.do_smooth:
            cells = astar.smooth(blocked, cells, dist, self.los_clearance)
        self.path_cells = cells
        self.last_plan_ok = True
        pts = self.resample(cells)
        self.publish_path(pts)
        self.status(f'OK ({reason}) : {raw_len:.2f} m, {expanded} noeuds, {dt:.0f} ms, {len(pts)} points')

    def resample(self, cells, spacing=0.10):
        pts = [self.cell_to_world(*cells[0])]
        for a, b in zip(cells, cells[1:]):
            (ax, ay), (bx, by) = self.cell_to_world(*a), self.cell_to_world(*b)
            n = max(1, int(math.hypot(bx - ax, by - ay) / spacing))
            pts += [(ax + (bx - ax) * t / n, ay + (by - ay) * t / n) for t in range(1, n + 1)]
        if self.goal is not None and math.hypot(pts[-1][0] - self.goal[0], pts[-1][1] - self.goal[1]) < 0.4:
            pts[-1] = self.goal
        return pts

    def tick(self):
        if self.goal is None or self.pose is None:
            return
        if math.hypot(self.pose[0] - self.goal[0], self.pose[1] - self.goal[1]) < self.tol:
            self.status('BUT ATTEINT')
            self.goal, self.path_cells = None, None
            self.publish_path([])
            return
        self.update_dynamic()
        blocked, _ = self.current_blocked()
        need = (not self.last_plan_ok) or self.path_cells is None
        if not need:    # le chemin courant traverse-t-il un obstacle (nouveau) ?
            for a, b in zip(self.path_cells, self.path_cells[1:]):
                if not astar.line_of_sight(blocked, a, b):
                    need = True
                    break
        if need:
            self.plan('chemin bloque' if self.last_plan_ok else 'nouvel essai')
        elif self.path_cells:
            self.publish_path(self.resample(self.path_cells))

    # ---------- publications ----------
    def status(self, text):
        # un echec identique (retente toutes les 0,5 s) n'est journalise qu'une fois toutes les 5 s
        now = time.monotonic()
        key = text.split('(')[0]
        if key == getattr(self, '_last_key', None) and now - getattr(self, '_last_log', 0.0) < 5.0:
            return
        self._last_key, self._last_log = key, now
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def publish_path(self, pts):
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.path_pub.publish(p)

    def grid_msg(self, mask):
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.info.resolution = self.res
        m.info.width, m.info.height = self.w, self.h
        m.info.origin.position.x, m.info.origin.position.y = self.origin
        m.info.origin.orientation.w = 1.0
        m.data = (mask.astype(np.int8) * 100).flatten().tolist()
        return m

    def publish_maps(self):
        self.map_pub.publish(self.grid_msg(self.occ))
        self.inf_pub.publish(self.grid_msg(self.blocked_static))
        mk = Marker()                       # murs en cubes : visible dans RViz meme si l'affichage Map bugue
        mk.header.frame_id = 'map'
        mk.ns, mk.id, mk.type, mk.action = 'maze', 0, Marker.CUBE_LIST, Marker.ADD
        mk.scale.x = mk.scale.y = self.res
        mk.scale.z = 0.3
        mk.color.r = mk.color.g = mk.color.b = 0.85
        mk.color.a = 1.0
        mk.pose.orientation.w = 1.0
        for j, i in zip(*np.nonzero(self.occ)):
            x, y = self.cell_to_world(int(i), int(j))
            mk.points.append(Point(x=x, y=y, z=0.15))
        self.mk_pub.publish(mk)


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
