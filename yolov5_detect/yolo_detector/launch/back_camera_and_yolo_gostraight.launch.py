from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_yolo(context, *args, **kwargs):
    target_class_value = LaunchConfiguration('target_class').perform(context)

    yolo_node = Node(
        package='yolo_detector',
        executable='back_yolo_gostraight_node',
        name='back_yolo_gostraight_node',
        output='screen',
        parameters=[{"target_class": target_class_value}]
    )
    return [TimerAction(period=2.0, actions=[yolo_node])]

def generate_launch_description():
    usbcam_dir = get_package_share_directory('camera_bringup')
    usbcam_launch_dir = os.path.join(usbcam_dir, 'launch')

    declare_target_class_arg = DeclareLaunchArgument(
        'target_class', default_value='4',
        description='检测类别'
    )

    usbcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usbcam_launch_dir, 'demo.launch.py')
        ),
        launch_arguments={
            'namespace': 'back_usbcam'
        }.items()
    )

    return LaunchDescription([
        declare_target_class_arg,
        usbcam_launch,
        OpaqueFunction(function=launch_yolo)
    ])
