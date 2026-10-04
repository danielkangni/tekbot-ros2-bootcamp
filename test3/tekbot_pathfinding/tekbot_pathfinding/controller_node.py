"""Suivi de chemin (pure pursuit) + evitement reactif au lidar pour TekBot (4 roues, commande differentielle).

Entrees : /planned_path, /odom, /scan      Sortie : /cmd_vel
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        for name, val in [('v_max', 0.30), ('w_max', 0.9), ('lookahead', 0.45), ('goal_tolerance', 0.12),
                          ('stop_dist', 0.28), ('slow_dist', 0.80), ('front_half_angle', 0.45),
                          ('stuck_time', 3.0)]:
            self.declare_parameter(name, val)
        self.p = {k: self.get_parameter(k).value for k in
                  ('v_max', 'w_max', 'lookahead', 'goal_tolerance', 'stop_dist', 'slow_dist',
                   'front_half_angle', 'stuck_time')}
        self.path = []
        self.idx = 0
        self.pose = None
        self.front = math.inf
        self.left, self.right = math.inf, math.inf
        self.blocked_since = None
        self.active = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.replan_pub = self.create_publisher(Empty, '/replan', 10)
        self.create_subscription(Path, '/planned_path', self.on_path, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_timer(0.05, self.loop)

    def on_path(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        if not pts:
            if self.active:
                self.stop()
            self.path, self.active = [], False
            return
        if not self.path or math.hypot(pts[-1][0] - self.path[-1][0], pts[-1][1] - self.path[-1][1]) > 0.05 \
                or len(pts) != len(self.path):
            self.idx = 0                       # nouveau chemin : on repart du debut
        self.path, self.active = pts, True

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_scan(self, m):
        f, l, r = math.inf, math.inf, math.inf
        for k, rng in enumerate(m.ranges):
            if not math.isfinite(rng) or rng < m.range_min:
                continue
            a = wrap(m.angle_min + k * m.angle_increment)
            if abs(a) < self.p['front_half_angle']:
                f = min(f, rng)
            elif 0.45 <= a < 1.6:
                l = min(l, rng)
            elif -1.6 < a <= -0.45:
                r = min(r, rng)
        self.front, self.left, self.right = f, l, r

    def stop(self):
        self.cmd_pub.publish(Twist())

    def loop(self):
        if not self.active or self.pose is None or not self.path:
            return
        x, y, yaw = self.pose
        p = self.p
        gx, gy = self.path[-1]
        if math.hypot(gx - x, gy - y) < p['goal_tolerance']:
            self.stop()
            self.active = False
            self.get_logger().info('But atteint, arret.')
            return
        # point le plus proche dans une petite fenetre devant l'indice courant (jamais en arriere), puis
        # point de visee a ``lookahead`` metres *le long du chemin* (pas a vol d'oiseau : les virages en
        # epingle du labyrinthe ramenent le chemin a cote de lui-meme, de l'autre cote d'un mur)
        window = range(self.idx, min(len(self.path), self.idx + 8))
        self.idx = min(window, key=lambda k: math.hypot(self.path[k][0] - x, self.path[k][1] - y))
        t, acc = self.idx, 0.0
        while t + 1 < len(self.path) and acc < p['lookahead']:
            acc += math.hypot(self.path[t + 1][0] - self.path[t][0], self.path[t + 1][1] - self.path[t][1])
            t += 1
        tx, ty = self.path[t]
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)

        cmd = Twist()
        if abs(alpha) > 0.5:                                  # mal oriente : on tourne sur place
            cmd.angular.z = max(-p['w_max'], min(p['w_max'], 1.5 * alpha))
        else:
            v = p['v_max'] * max(0.3, math.cos(alpha))
            v = min(v, max(0.08, math.hypot(gx - x, gy - y)))       # ralentit pres du but
            if self.front < p['slow_dist']:                        # evitement : ralentit puis s'arrete
                k = (self.front - p['stop_dist']) / (p['slow_dist'] - p['stop_dist'])
                v *= max(0.0, min(1.0, k))
            cmd.linear.x = v
            cmd.angular.z = max(-p['w_max'], min(p['w_max'], 2.0 * alpha))
        # obstacle tout proche : on ne fonce pas, on s'ecarte du cote le plus degage
        if self.front < p['stop_dist']:
            cmd.linear.x = 0.0
            if abs(alpha) > 0.3:                  # il suffit de se reorienter vers le chemin : on le fait
                cmd.angular.z = max(-p['w_max'], min(p['w_max'], 1.5 * alpha))
            else:                                 # le chemin est devant mais bloque : on s'ecarte du cote libre
                cmd.angular.z = 0.6 if self.left > self.right else -0.6
            now = self.get_clock().now().nanoseconds * 1e-9
            self.blocked_since = self.blocked_since or now
            if now - self.blocked_since > p['stuck_time']:
                self.replan_pub.publish(Empty())              # le planificateur ajoute l'obstacle et replanifie
                self.blocked_since = now
        else:
            self.blocked_since = None
        self.get_logger().info(
            f'pos=({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f} cible=({tx:.2f},{ty:.2f}) alpha={math.degrees(alpha):.0f} '
            f'avant={self.front:.2f} g={self.left:.2f} d={self.right:.2f} -> v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}',
            throttle_duration_sec=1.0)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()
