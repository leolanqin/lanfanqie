import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 命名空间（便于区分多个摄像头节点）
    cam_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='该摄像头节点的命名空间'
    )

    cam_namespace = LaunchConfiguration('namespace')

    # 获取参数文件路径
    usb_cam_dir = get_package_share_directory('camera_bringup')
    params_path = os.path.join(usb_cam_dir, 'config', 'params1.yaml')

    return LaunchDescription([
        cam_namespace_arg,

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            namespace=cam_namespace,
            parameters=[
                params_path,
            ],
            respawn=True,              
            respawn_delay=1.0,         # 崩了等 1 秒自动重启
        )
    ])

