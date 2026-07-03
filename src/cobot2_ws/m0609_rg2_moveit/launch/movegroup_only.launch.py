import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# bringup.launch.py(mode:=real|virtual)가 이미 robot_state_publisher /
# joint_state_publisher / static_tf / rviz(URDF 확인용)를 띄워 놓은 상태에서,
# 그 위에 move_group만 추가로 띄우기 위한 launch.
# (moveit.launch.py는 자체 robot_state_publisher 등을 같이 띄워서
#  bringup과 노드 이름이 충돌하므로 실기/가상 로봇 연동 시에는 이 파일을 사용)
#
# 사용법:
#   ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<IP> port:=<PORT>
#   ros2 launch m0609_rg2_moveit movegroup_only.launch.py


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    args = [
        DeclareLaunchArgument('rviz', default_value='true', description='MoveIt RViz 플러그인 실행 여부'),
    ]

    bringup_pkg = get_package_share_directory('m0609_rg2_bringup')
    moveit_pkg = get_package_share_directory('m0609_rg2_moveit')

    xacro_file = os.path.join(bringup_pkg, 'urdf', 'm0609_with_rg2.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    srdf_file = os.path.join(moveit_pkg, 'config', 'm0609_rg2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {
            'robot_description_semantic': f.read()
        }

    kinematics_yaml = load_yaml('m0609_rg2_moveit', 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

    joint_limits_yaml = load_yaml('m0609_rg2_moveit', 'config/joint_limits.yaml')
    joint_limits = {
        'robot_description_planning': joint_limits_yaml
    }

    ompl_planning_yaml = load_yaml('m0609_rg2_moveit', 'config/ompl_planning.yaml')
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
    }

    moveit_controllers_yaml = load_yaml('m0609_rg2_moveit', 'config/moveit_controllers.yaml')

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
        ],
        remappings=[
            ('/dsr_moveit_controller/follow_joint_trajectory', '/dsr01/dsr_moveit_controller/follow_joint_trajectory'),
            ('/rg2_gripper_controller/follow_joint_trajectory', '/dsr01/rg2_gripper_controller/follow_joint_trajectory')
        ]
    )

    # bringup.launch.py가 이미 name='rviz2'로 RViz를 띄우므로 이름 충돌 방지
    rviz_config_file = os.path.join(moveit_pkg, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines,
            joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [
        move_group_node,
        rviz_node,
    ])
