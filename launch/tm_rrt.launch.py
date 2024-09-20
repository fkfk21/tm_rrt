from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tm_rrt',
            executable='tm_rrt_node',
            name='tm_rrt_node',
            output='screen',
            parameters=[
                {'domain_file': 'domain_2c4p (anomaly2.1).prolog'},  # JRAS case3.2
                # {'domain_file': 'domain_3c6p (anomaly2.2).prolog'},  # JRAS case3.3
                # {'domain_file': 'domain_3c6p (anomaly2.3).prolog'},  # JRAS case3.4

                {'planner_type': 'tm_rrt'},  # TM-RRT
                # {'planner_type': 'bfs_rrt'},  # BFS+RRT
                {'debug': False},
                # get task selector (best = 0, uniform = 1, montecarlo = 2)
                {"task_selector": 0},
                {'number_of_runs': 100},
                {'w_b': 1.0},
                {'w_t': 5.0},
                {'path_len': 0.9},
                {'p_s': 0.3},
                {'p_c': 0.3}
            ]
        )
    ])
