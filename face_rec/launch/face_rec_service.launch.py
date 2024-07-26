from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    decl_face_data = DeclareLaunchArgument(
        name='face_data',
        default_value='/home/xiao/uucar_ws/src/face_rec/face_data',
        description='Path to face data yaml file'
    )
    face_rec_service = Node(
        package='face_rec',
        executable='face_rec_service',
        name='face_rec_service',
        output='screen',
        # respawn=True,
        parameters=[{
            "face_data": LaunchConfiguration(variable_name='face_data')
        }
        ]
    )
    ld.add_action(decl_face_data)
    ld.add_action(face_rec_service)
    return ld