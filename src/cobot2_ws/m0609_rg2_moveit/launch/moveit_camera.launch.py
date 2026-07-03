import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():

    bringup_pkg = get_package_share_directory('m0609_rg2_bringup')
    moveit_pkg  = get_package_share_directory('m0609_rg2_moveit')

    # 카메라 포함 URDF
    xacro_file = os.path.join(bringup_pkg, 'urdf', 'm0609_with_rg2_camera.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    srdf_file = os.path.join(moveit_pkg, 'config', 'm0609_rg2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {
            'robot_description_semantic': f.read()
        }

    kinematics_yaml = load_yaml('m0609_rg2_moveit', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_yaml = load_yaml('m0609_rg2_moveit', 'config/joint_limits.yaml')
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    ompl_planning_yaml = load_yaml('m0609_rg2_moveit', 'config/ompl_planning.yaml')
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
    }

    moveit_controllers_yaml = load_yaml('m0609_rg2_moveit', 'config/moveit_controllers.yaml')

    # camera:=true 일 때만 RealSense 드라이버 기동
    # 가상 환경에서는 false(기본)로 두면 URDF 모델만 RViz에 표시됨
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='false',
        description='true = 실제 RealSense 연결 시 드라이버 기동'
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'enable_sync': True,
        }],
        condition=IfCondition(LaunchConfiguration('camera')),
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description]
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipelines,
            moveit_controllers_yaml,
            {'use_sim_time': False},
        ]
    )

    rviz_config_file = os.path.join(moveit_pkg, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines,
            joint_limits,
        ]
    )

    return LaunchDescription([
        camera_arg,
        realsense_node,
        static_tf,
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ])
