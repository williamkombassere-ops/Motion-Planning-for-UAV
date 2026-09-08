"""
Node ROS2 de planification de trajectoire 2D pour le Crazyflie.

S'intègre à côté du `simple_mapper` du package crazyflie_ros2_multiranger :
il ne remplace rien, il consomme la carte que celui-ci publie.

Souscrit :
    <map_topic>   (nav_msgs/OccupancyGrid), défaut '/crazyflie/map'
    /goal_pose    (geometry_msgs/PoseStamped) — objectif envoyé depuis RViz2
                  avec le bouton "2D Goal Pose"
    Position courante du drone : lue via TF (map_frame -> robot_frame), car
    c'est ce que le simple_mapper publie déjà (transforms préconfigurées).

Publie :
    /plan         (nav_msgs/Path) — chemin planifié, visible dans RViz2

⚠️ IMPORTANT : les noms de topics/frames par défaut ci-dessous sont ceux
documentés par le tutoriel Bitcraze, mais peuvent varier selon la version
exacte de crazyflie_ros2_multiranger que tu as clonée. Avant de lancer,
vérifie avec `ros2 topic list` et `ros2 run tf2_tools view_frames` pendant
que la simulation tourne, et ajuste config/planner_params.yaml si besoin.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros import TransformException

from motion_planning_project.astar import AStarPlanner
from motion_planning_project.utils import world_to_grid, grid_to_world


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('map_topic', '/crazyflie/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'crazyflie/base_link')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('treat_unknown_as_occupied', True)

        self.map_topic = self.get_parameter('map_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.treat_unknown_as_occupied = self.get_parameter('treat_unknown_as_occupied').value

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_msg: OccupancyGrid = None

        self.get_logger().info(
            f"Planner node démarré. Carte attendue sur '{self.map_topic}', "
            f"pose via TF '{self.map_frame}' -> '{self.robot_frame}'. "
            f"En attente d'un objectif sur /goal_pose ..."
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def get_current_pose(self):
        """Lit la position courante du drone via TF. Retourne (x, y) en mètres ou None."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as ex:
            self.get_logger().warn(f"TF indisponible ({self.map_frame} -> {self.robot_frame}): {ex}")
            return None

    def goal_callback(self, goal_msg: PoseStamped):
        if self.map_msg is None:
            self.get_logger().warn(f"Pas de carte reçue sur '{self.map_topic}', impossible de planifier.")
            return

        current = self.get_current_pose()
        if current is None:
            self.get_logger().warn("Impossible de récupérer la position courante du drone (TF).")
            return

        path = self.plan_path(current, (goal_msg.pose.position.x, goal_msg.pose.position.y))
        if path is None:
            self.get_logger().warn("Aucun chemin trouvé vers l'objectif (obstacle ou zone inconnue).")
            return

        self.path_pub.publish(path)
        self.get_logger().info(f"Chemin publié avec {len(path.poses)} points.")

    def plan_path(self, start_xy, goal_xy):
        info = self.map_msg.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        data = self.map_msg.data
        grid = [data[row * width:(row + 1) * width] for row in range(height)]

        start_cell = world_to_grid(start_xy[0], start_xy[1], origin_x, origin_y, resolution)
        goal_cell = world_to_grid(goal_xy[0], goal_xy[1], origin_x, origin_y, resolution)

        planner = AStarPlanner(
            grid,
            occupied_threshold=self.occupied_threshold,
            treat_unknown_as_occupied=self.treat_unknown_as_occupied,
        )
        cell_path = planner.plan(start_cell, goal_cell)

        if cell_path is None:
            return None

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for gx, gy in cell_path:
            wx, wy = grid_to_world(gx, gy, origin_x, origin_y, resolution)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
