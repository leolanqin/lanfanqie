from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    usbcam_dir = get_package_share_directory('camera_bringup')
    usbcam_launch_dir = os.path.join(usbcam_dir, 'launch')

    # YOLO 节点（等待相机就绪后再启动）
    yolo_node = Node(
        package='yolo_detector',
        executable='former_yolo_gostraight_node',
        name='former_yolo_gostraight_node',
        output='screen',
        parameters=[]
    )

    usbcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usbcam_launch_dir, 'demo1.launch.py')
        ),
        launch_arguments={
            'namespace': 'former_usbcam'
        }.items()
    )

    return LaunchDescription([
        usbcam_launch,
        TimerAction(  # 延迟 N 秒后启动 yolo 节点
            period=2.0,  # 延迟时间，单位：秒（可根据相机启动时间调节）
            actions=[yolo_node]
        )
    ])

