"""
Node ROS2 de planification de trajectoire.

Souscrit :
    /map        (nav_msgs/OccupancyGrid) - carte d'occupation
    /goal_pose  (geometry_msgs/PoseStamped) - objectif envoyé par RViz (bouton "2D Goal Pose")
    /odom       (nav_msgs/Odometry) - position courante du robot

Publie :
    /plan       (nav_msgs/Path) - chemin planifié
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped

from motion_planning_project.astar import AStarPlanner
from motion_planning_project.utils import world_to_grid, grid_to_world


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        # Paramètres
        self.declare_parameter('occupied_threshold', 50)
        self.occupied_threshold = self.get_parameter('occupied_threshold').value

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.map_msg: OccupancyGrid | None = None
        self.current_pose: PoseStamped | None = None

        self.get_logger().info('Planner node démarré. En attente de /map et /goal_pose ...')

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose

    def goal_callback(self, goal_msg: PoseStamped):
        if self.map_msg is None:
            self.get_logger().warn("Pas de carte reçue, impossible de planifier.")
            return
        if self.current_pose is None:
            self.get_logger().warn("Pas de position courante (odom), impossible de planifier.")
            return

        path = self.plan_path(self.current_pose, goal_msg)
        if path is None:
            self.get_logger().warn("Aucun chemin trouvé vers l'objectif.")
            return

        self.path_pub.publish(path)
        self.get_logger().info(f"Chemin publié avec {len(path.poses)} points.")

    def plan_path(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> Path | None:
        info = self.map_msg.info
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height

        # Reconstruction de la grille 2D (row-major, comme OccupancyGrid.data)
        data = self.map_msg.data
        grid = [data[row * width:(row + 1) * width] for row in range(height)]

        start_cell = world_to_grid(start_pose.pose.position.x, start_pose.pose.position.y,
                                    origin_x, origin_y, resolution)
        goal_cell = world_to_grid(goal_pose.pose.position.x, goal_pose.pose.position.y,
                                   origin_x, origin_y, resolution)

        planner = AStarPlanner(grid, occupied_threshold=self.occupied_threshold)
        cell_path = planner.plan(start_cell, goal_cell)

        if cell_path is None:
            return None

        path_msg = Path()
        path_msg.header.frame_id = self.map_msg.header.frame_id
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
