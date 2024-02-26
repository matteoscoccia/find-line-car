import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='firstPackage',
            executable='controller',
            namespace='car_1',
            name='controller'),     
        launch_ros.actions.Node(
            package='firstPackage',
            executable='motors',
            namespace='car_1',
            name='motors'),
        launch_ros.actions.Node(
            package='firstPackage',
            executable='camera',
            namespace='car_1',
            name='camera'),
    ])
"""launch_ros.actions.Node(
            package='firstPackage',
            executable='findline',
            namespace='car_1',
            name='findline'),"""
"""launch_ros.actions.Node(
            package='firstPackage',
            executable='camera',
            namespace='car_1',
            name='camera'),"""
"""launch_ros.actions.Node(
            package='firstPackage',
            executable='movement',
            namespace='car_1',
            name='movement'),"""
