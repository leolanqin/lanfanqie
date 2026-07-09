import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    cam_namespace=DeclareLaunchArgument(
        'namespace', 
        default_value='')

    # 获取 usb_cam 包的 share 路径
    usb_cam_dir = get_package_share_directory('camera_bringup')

    # 拼接参数文件路径
    params_path = os.path.join(
        usb_cam_dir,
        'config',
        'params.yaml'  # 确保此文件存在
    )

    # 创建启动描述对象
    ld = LaunchDescription()

    # 加入参数声明
    ld.add_action(cam_namespace)

    # 创建并添加摄像头节点
    ld.add_action(
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[params_path],
            respawn=True,              
            respawn_delay=1.0,         # 崩了等 1 秒自动重启
        )
    )

    return ld


