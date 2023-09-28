import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='controller',
            namespace='tb3_0',
            name='controller'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='controller',
            namespace='tb3_1',
            name='controller'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='leader',
            name='leader')
    ])
