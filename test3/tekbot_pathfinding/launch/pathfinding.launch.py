"""Pathfinding TekBot : planificateur A* + controleur + RViz2.

  ros2 launch tekbot_pathfinding pathfinding.launch.py            # simulation deja lancee
  ros2 launch tekbot_pathfinding pathfinding.launch.py sim:=true  # lance aussi Gazebo + robot + labyrinthe
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_time = {'use_sim_time': True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('maze_solving'), 'launch', 'tekbot_maze.launch.py'])),
        condition=IfCondition(LaunchConfiguration('sim')))

    # L'odometrie du robot est exprimee dans le repere du monde (odometry_source = 1) et la carte aussi :
    # map -> odom est donc l'identite.
    map_to_odom = Node(package='tf2_ros', executable='static_transform_publisher',
                       arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
                       parameters=[sim_time])

    planner = Node(package='tekbot_pathfinding', executable='planner_node', name='planner',
                   output='screen', parameters=[sim_time])
    controller = Node(package='tekbot_pathfinding', executable='controller_node', name='controller',
                      output='screen', parameters=[sim_time])
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                arguments=['-d', PathJoinSubstitution([FindPackageShare('tekbot_pathfinding'), 'rviz',
                                                       'pathfinding.rviz'])],
                parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false', description='lancer aussi la simulation'),
        DeclareLaunchArgument('rviz', default_value='true', description='lancer RViz2'),
        sim, map_to_odom, planner, controller, rviz])
