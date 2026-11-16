# Bootcamp robotique TekBot · ROS 2

Exercices réalisés pendant le bootcamp robotique de TekBot EPITECH (Bénin, 2026), du Python orienté objet jusqu'à la navigation autonome d'un robot simulé.

| Dossier | Sujet | Technologies |
|---|---|---|
| [`test1/`](test1/) | Modélisation objet d'un robot : classe `Robot`, sous-classes `WheeledRobot` et `LeggedRobot` | Python, UML |
| [`test2/`](test2/) | Package ROS 2 `sensor_data_evaluation` : un nœud publie des mesures de capteurs, un autre les reçoit et les évalue | ROS 2 Humble, rclpy |
| [`test3/`](test3/) | **Navigation autonome A\*** : le robot TekBot rejoint un point du labyrinthe, suit le chemin et replanifie quand le lidar détecte un obstacle | ROS 2 Humble, Gazebo Classic, RViz2, Docker |

## Point fort : navigation A* (test 3)

- Planification A\* sur une carte d'occupation du labyrinthe (5 cm par cellule).
- Contrôleur de suivi de trajectoire.
- Détection d'obstacles imprévus au lidar et replanification.
- Environnement ROS 2 Humble conteneurisé (Podman ou Docker) et tests unitaires de l'algorithme.

Les instructions de lancement complètes sont dans [`test3/README.md`](test3/README.md).

---

Auteur : KANGNI Jean-Daniel · [Portfolio](https://danielkangni.github.io) · [GitHub](https://github.com/danielkangni)
