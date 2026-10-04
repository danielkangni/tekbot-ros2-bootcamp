# Test 3 — Pathfinding A* pour TekBot (ROS 2 Humble + Gazebo Classic)

Le robot **TekBot** rejoint un point choisi dans le labyrinthe de `maze_solving` : un **A\*** calcule le chemin sur une
carte d'occupation, un contrôleur le suit, et le lidar permet de détecter un obstacle imprévu et de **replanifier**.
Résultats visibles dans Gazebo et RViz2.

```
test3/
├── tekbot_pathfinding/     package ROS 2 (A*, planificateur, contrôleur, launch, RViz, carte)
│   ├── tekbot_pathfinding/   astar.py · planner_node.py · controller_node.py
│   ├── launch/pathfinding.launch.py
│   ├── rviz/pathfinding.rviz
│   ├── maps/maze_map.{pgm,yaml}      carte du labyrinthe (5 cm/cellule)
│   ├── scripts/world_to_map.py       génère la carte depuis le monde Gazebo
│   └── test/test_astar.py            tests unitaires de l'algorithme
├── docker/                 environnement ROS 2 Humble prêt à l'emploi (Podman ou Docker)
└── tekbot_sim/             à cloner (voir ci-dessous) : robot + labyrinthe fournis
```

## 1. Récupérer la simulation fournie

```bash
cd test3
git clone https://github.com/charif-tekbot/tekbot_sim.git
```

## 2. Lancer — option A : conteneur (n'importe quel Linux, par ex. Ubuntu 24.04)

Prérequis : [Podman](https://podman.io) (ou Docker : `ENGINE=docker`), un serveur X11/XWayland. Dossier `test3/` :

```bash
./docker/run.sh build     # une seule fois : image ROS 2 Humble + Gazebo Classic + RViz2 + Nav2 (~3 Go)
./docker/run.sh colcon    # compile tekbot_description, tekbot_control, maze_solving, tekbot_pathfinding
```

**Terminal 1 — simulation** (attendre ~1 min, le robot rouge apparaît dans le labyrinthe) :
```bash
./docker/run.sh ros2 launch maze_solving tekbot_maze.launch.py
```
**Terminal 2 — pathfinding + RViz2** :
```bash
./docker/run.sh exec
ros2 launch tekbot_pathfinding pathfinding.launch.py
```
Variables utiles : `SOFTWARE_GL=1` force le rendu CPU (lent, déconseillé), `ENGINE=docker` utilise Docker.

## 3. Lancer — option B : ROS 2 Humble natif (Ubuntu 22.04)

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-joint-state-publisher ros-humble-rviz2 \
  ros-humble-rqt-gui ros-humble-rqt-gui-py ros-humble-joy ros-humble-teleop-twist-joy \
  ros-humble-robot-localization ros-humble-tf2-ros python3-colcon-common-extensions
mkdir -p ~/ws/src && ln -s "$PWD/tekbot_sim" "$PWD/tekbot_pathfinding" ~/ws/src/
cd ~/ws && source /opt/ros/humble/setup.bash
colcon build --packages-select tekbot_description tekbot_control maze_solving tekbot_pathfinding
source install/setup.bash
ros2 launch tekbot_pathfinding pathfinding.launch.py sim:=true     # simulation + pathfinding + RViz2
```
(`sim:=true` lance aussi Gazebo, le robot et le labyrinthe ; sans cet argument, la simulation doit déjà tourner.)

## 4. Donner un but

- Dans RViz2 : outil **2D Goal Pose**, clic dans le labyrinthe.
- Ou en ligne de commande (envoyer **une fois**) :
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.56, y: 0.7}, orientation: {w: 1.0}}}"
```
Le chemin s'affiche en rouge (`/planned_path`), le robot le suit, `/planner_status` indique `OK`, `ECHEC : aucun chemin`
ou `BUT ATTEINT`. Les centres de cases sont à `x = -2.94 + i + 0.5`, `y = -3.01 + j + 0.5` (i, j de 0 à 5).

## 5. Tester un obstacle imprévu

Poser une boîte sur le chemin pendant que la simulation tourne, puis envoyer un but : le lidar la détecte et le chemin
est recalculé.
```bash
cat > /tmp/box.sdf <<'XML'
<?xml version="1.0"?><sdf version="1.6"><model name="obstacle"><static>true</static><link name="l">
<collision name="c"><geometry><box><size>0.3 0.3 0.4</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.3 0.3 0.4</size></box></geometry></visual></link></model></sdf>
XML
ros2 run gazebo_ros spawn_entity.py -entity obstacle -file /tmp/box.sdf -x -0.33 -y -0.54 -z 0.2
```

## 6. Tests unitaires de l'algorithme (sans ROS)

```bash
cd tekbot_pathfinding && python3 -m pytest -q test
```
Ils vérifient notamment que A* trouve des chemins de même longueur que Dijkstra sur 20 grilles aléatoires.

## 7. Topics

| Topic | Type | Rôle |
|---|---|---|
| `/goal_pose` | `PoseStamped` | but (RViz 2D Goal Pose) |
| `/odom`, `/scan` | `Odometry`, `LaserScan` | pose du robot (repère monde), lidar |
| `/planned_path` | `nav_msgs/Path` | chemin calculé par A* |
| `/cmd_vel` | `Twist` | commande du robot |
| `/map`, `/maze_markers` | `OccupancyGrid`, `Marker` | carte et murs (affichage RViz) |
| `/planner_status` | `String` | état du planificateur |

## 8. Régénérer la carte

```bash
python3 tekbot_pathfinding/scripts/world_to_map.py      # écrit tekbot_pathfinding/maps/maze_map.{pgm,yaml}
```

## 9. Limites connues

- L'affichage « Map » de RViz2 (`/map`) peut rester vide selon le pilote graphique (erreur GLSL) : les murs sont donc
  aussi publiés en cubes sur `/maze_markers`, toujours visibles.
- Un seul scénario d'obstacle a été validé en simulation ; le comportement face à plusieurs obstacles n'est pas garanti.
- L'odométrie du simulateur est exprimée dans le repère du monde : sur un robot réel, il faudrait de la localisation
  (AMCL/SLAM) entre la carte et l'odométrie.
