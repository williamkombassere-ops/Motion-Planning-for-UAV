"""
Node ROS2 qui suit le chemin publié sur /plan et envoie des commandes de
vitesse (geometry_msgs/Twist) au Crazyflie pour le faire avancer le long
du chemin, point par point.

⚠️ Ce node NE FAIT PAS décoller ni atterrir le drone. Il faut d'abord le
faire décoller (ex. touche 't' avec teleop_twist_keyboard, comme dans le
tutoriel Bitcraze), PUIS envoyer un objectif via RViz2 (2D Goal Pose).
Ce node prendra alors le relais sur le topic cmd_vel pour suivre le chemin
planifié, à la place du clavier.

Les commandes de vitesse sont exprimées dans le repère du corps du drone
(x = avant, y = gauche), ce qui correspond à ce qu'attend le driver
Crazyswarm2 sur le topic cmd_vel (comme avec teleop_twist_keyboard).
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'crazyflie/base_link')
        self.declare_parameter('waypoint_tolerance', 0.15)   # m
        self.declare_parameter('max_linear_speed', 0.3)      # m/s
        self.declare_parameter('control_rate_hz', 10.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.waypoints = []   # liste de (x, y) restant à atteindre
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

        self.get_logger().info(
            f"Path follower démarré. Publie sur '{self.cmd_vel_topic}', "
            f"lit la pose via TF '{self.map_frame}' -> '{self.robot_frame}'."
        )

    def path_callback(self, path_msg: Path):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        self.get_logger().info(f"Nouveau chemin reçu : {len(self.waypoints)} points.")

    def get_current_pose(self):
        """Retourne (x, y, yaw) en repère map, ou None si TF indisponible."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"TF indisponible: {ex}", throttle_duration_sec=2.0)
            return None

        q = t.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return t.transform.translation.x, t.transform.translation.y, yaw

    def control_loop(self):
        if not self.waypoints:
            return

        pose = self.get_current_pose()
        if pose is None:
            return
        x, y, yaw = pose

        # Retire les waypoints déjà atteints
        while self.waypoints:
            tx, ty = self.waypoints[0]
            if math.hypot(tx - x, ty - y) < self.waypoint_tolerance:
                self.waypoints.pop(0)
            else:
                break

        if not self.waypoints:
            self.cmd_pub.publish(Twist())  # stop
            self.get_logger().info("Chemin terminé, arrêt.")
            return

        target_x, target_y = self.waypoints[0]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        # Direction vers le waypoint, exprimée dans le repère map
        angle_to_target = math.atan2(dy, dx)
        # Passage repère map -> repère corps du drone (rotation de -yaw)
        relative_angle = angle_to_target - yaw
        speed = min(self.max_linear_speed, distance)

        cmd = Twist()
        cmd.linear.x = speed * math.cos(relative_angle)
        cmd.linear.y = speed * math.sin(relative_angle)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # stop propre en sortant
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
