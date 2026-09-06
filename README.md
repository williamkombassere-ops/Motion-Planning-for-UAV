# motion_planning_project

Projet de motion planning sous **ROS2 + Gazebo**, en Python.

Le package fournit un planificateur A* qui :
- souscrit à `/map` (`nav_msgs/OccupancyGrid`) et `/odom` (`nav_msgs/Odometry`) ;
- reçoit un objectif via `/goal_pose` (bouton **2D Goal Pose** dans RViz2) ;
- publie le chemin planifié sur `/plan` (`nav_msgs/Path`).

Un monde Gazebo minimal avec deux obstacles est fourni pour tester rapidement.

## Structure

```
motion_planning_project/
├── motion_planning_project/   # code Python du package
│   ├── astar.py                # algorithme A* (pur Python, testable sans ROS)
│   ├── planner_node.py         # node ROS2
│   └── utils.py                # conversions repère monde <-> grille
├── launch/
│   └── motion_planning.launch.py
├── config/
│   └── planner_params.yaml
├── worlds/
│   └── simple_world.world
└── test/
    └── test_planner.py
```

## Installation

Dans un workspace ROS2 (ex. `~/ros2_ws`) :

```bash
cd ~/ros2_ws/src
git clone <url-de-ton-repo> motion_planning_project
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select motion_planning_project
source install/setup.bash
```

## Lancement

```bash
ros2 launch motion_planning_project motion_planning.launch.py
```

Puis dans RViz2, utilise le bouton **2D Goal Pose** pour envoyer un objectif : le
chemin planifié apparaît sur le topic `/plan`.

## Tests

Les tests de l'algorithme A* sont indépendants de ROS2 et peuvent être lancés avec :

```bash
python3 -m pytest test/
```

## Pistes d'évolution

- Remplacer A* par RRT* ou Hybrid A* pour une meilleure prise en compte de la cinématique du robot.
- Ajouter un contrôleur (pure pursuit / MPC) qui suit le `/plan` publié.
- Intégrer avec Nav2 comme plugin de planificateur global.

## Licence

MIT — voir [LICENSE](LICENSE).
