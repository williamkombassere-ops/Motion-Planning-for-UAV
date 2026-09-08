# motion_planning_project

Motion planning **2D** pour le drone **Crazyflie**, en ROS2 Humble + Gazebo (Fortress/Harmonic),
qui vient se greffer sur la démo officielle Bitcraze
[*Crazyflie's Adventures with ROS 2 and Gazebo*](https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/).

Ce package **ne remplace pas** `crazyflie_ros2_multiranger` : il consomme la carte
(`OccupancyGrid`) publiée par son node `simple_mapper`, planifie un chemin dessus avec un
A*, et fait suivre ce chemin au drone en publiant des commandes de vitesse — à la place du
téléopérateur clavier.

> **Pourquoi 2D et pas 3D ?** Le deck multi-ranger ne mesure des distances qu'horizontalement,
> à altitude constante. Un évitement d'obstacles vraiment 3D nécessiterait un capteur
> supplémentaire (vertical) ou une carte 3D, absents de ce setup.

## Où ça se branche

```
~/crazyflie_mapping_demo/
├── simulation_ws/
│   └── crazyflie-simulation/          (existant, non modifié)
└── ros2_ws/src/
    ├── crazyflie_ros2_multiranger/    (existant, non modifié — fournit /crazyflie/map)
    ├── ros_gz_crazyflie/              (existant, non modifié)
    ├── crazyswarm2/                   (existant, non modifié)
    └── motion_planning_project/       (CE package)
```

## Contenu

- `motion_planning_project/astar.py` — A* 2D pur Python (testable sans ROS)
- `motion_planning_project/planner_node.py` — souscrit à `/crazyflie/map`, reçoit un
  objectif via `/goal_pose` (bouton **2D Goal Pose** de RViz2), publie `/plan`
- `motion_planning_project/path_follower_node.py` — suit `/plan` et publie des `Twist`
  sur `/cmd_vel` pour faire avancer le drone point par point
- `launch/motion_planning.launch.py` — lance les deux nodes ci-dessus
- `config/planner_params.yaml` — noms de topics/frames, à **vérifier et ajuster** (voir plus bas)

## ⚠️ À vérifier avant de lancer

Les noms de topics/frames par défaut (`/crazyflie/map`, `map`, `crazyflie/base_link`,
`/cmd_vel`) sont ceux documentés par le tutoriel Bitcraze, mais peuvent varier légèrement
selon la version exacte des repos que tu as clonés. Une fois la démo existante lancée
(étape 1 ci-dessous), vérifie dans un terminal :

```bash
ros2 topic list          # confirme le nom exact du topic de la carte et de cmd_vel
ros2 run tf2_tools view_frames   # génère frames.pdf : confirme les noms de frames TF
```

Ajuste `config/planner_params.yaml` en conséquence si les noms diffèrent.

## Installation

```bash
cd ~/crazyflie_mapping_demo/ros2_ws/src
git clone <url-de-ton-repo> motion_planning_project
cd ~/crazyflie_mapping_demo/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select motion_planning_project
source install/setup.bash
```

## Utilisation

**Terminal 1** — lance la démo existante (Gazebo + simple_mapper + RViz2), comme d'habitude :

```bash
source ~/crazyflie_mapping_demo/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/root/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py
```

**Terminal 2** — fait décoller le drone au clavier (comme dans le tutoriel), pour avoir une
première carte partielle :

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Appuie sur `t` pour décoller, déplace-le un peu pour que le multiranger voie quelques
obstacles, puis **ferme ce terminal** (ou laisse-le, mais n'envoie plus de commandes).

**Terminal 3** — lance le planificateur + le suiveur de chemin :

```bash
source ~/crazyflie_mapping_demo/ros2_ws/install/setup.bash
ros2 launch motion_planning_project motion_planning.launch.py
```

**Dans RViz2** : clique sur **2D Goal Pose**, puis clique/glisse sur la carte à l'endroit où
tu veux envoyer le drone. Le chemin planifié s'affiche sur `/plan`, et le drone doit s'y
diriger automatiquement.

## Tests

```bash
cd ~/crazyflie_mapping_demo/ros2_ws/src/motion_planning_project
python3 -m pytest test/
```

## Limitations connues / pistes d'évolution

- Suivi de chemin très simple (P-control point à point) — pas d'évitement dynamique
  d'obstacle en cours de trajet, pas de lissage de trajectoire.
- Pas de gestion du décollage/atterrissage — à faire au clavier avant/après.
- Replanification uniquement sur nouvel objectif, pas en continu.
- Passer à RRT* ou à un post-traitement de lissage (Bezier/B-spline) pour des trajectoires
  plus fluides.
- Brancher sur NAV2 (mentionné dans le tutoriel Bitcraze) pour bénéficier de la
  replanification dynamique et de l'évitement d'obstacles en temps réel.

## Licence

MIT — voir [LICENSE](LICENSE).
