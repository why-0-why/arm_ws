import launch
import launch_ros

## 电机端节点启动 ##
def generate_launch_description():
    # 1.声明launch参数
    position_mode = launch.actions.DeclareLaunchArgument('position_mode', default_value='2')# 电机模式 1:速度 2：位置 3：测试模式

    # 关节电机参数
    joint1_node_id = launch.actions.DeclareLaunchArgument('joint1_node_id', default_value='3')
    joint1_name = launch.actions.DeclareLaunchArgument('joint1_name', default_value='joint1')


    joint2_node_id = launch.actions.DeclareLaunchArgument('joint2_node_id', default_value='2')
    joint2_name = launch.actions.DeclareLaunchArgument('joint2_name', default_value='joint2')

    # 2.创建节点，把参数传入给节点
    # 关节电机节点
    action_node_joint1 = launch_ros.actions.Node(
        package='servo_motor_control',
        executable='servo_motor_node',
        name=[launch.substitutions.LaunchConfiguration('joint1_name')],
        output='log',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('joint1_node_id'),'mode': launch.substitutions.LaunchConfiguration('position_mode')}]
    )
    action_node_joint2 = launch_ros.actions.Node(
        package='servo_motor_control',
        executable='servo_motor_node',
        name=[launch.substitutions.LaunchConfiguration('joint2_name')],
        output='log',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('joint2_node_id'),'mode': launch.substitutions.LaunchConfiguration('position_mode')}]
    )
    # 3.合成启动描述并返回
    launch_description = launch.LaunchDescription([
        # 电机参数
        position_mode,
        # joint1的节点
        joint1_node_id,
        joint1_name,
        action_node_joint1,
        # joint2的节点
        joint2_node_id,
        joint2_name,
        action_node_joint2,
    ])
    return launch_description