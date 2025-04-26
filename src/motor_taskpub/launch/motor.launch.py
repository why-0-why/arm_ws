import launch
import launch_ros

def generate_launch_description():
    # 1.声明launch参数    
    # 声明关节数量
    action_declare_arg_node_num = launch.actions.DeclareLaunchArgument('node_num', default_value='2')
    # motor1 1. 声明参数
    action_declare_arg_motor1_node_id = launch.actions.DeclareLaunchArgument('motor1_node_id', default_value='3')
    action_declare_arg_motor1_name = launch.actions.DeclareLaunchArgument('motor1_name', default_value='motor3')
    action_declare_arg_motor1_mode = launch.actions.DeclareLaunchArgument('motor1_mode', default_value='2')
    # 电机模式 1:速度 2：位置 3：测试模式
    # motor2 1. 声明参数
    action_declare_arg_motor2_name = launch.actions.DeclareLaunchArgument('motor2_name', default_value='motor2')
    action_declare_arg_motor2_node_id = launch.actions.DeclareLaunchArgument('motor2_node_id', default_value='2')
    action_declare_arg_motor2_mode = launch.actions.DeclareLaunchArgument('motor2_mode', default_value='2')
    # 2.创建节点，把参数传入给节点
    # motor1 2. 创建控制节点
    action_node_motor1 = launch_ros.actions.Node(
        package='servo_motor_control',
        executable='servo_motor_node',
        name=[launch.substitutions.LaunchConfiguration('motor1_name')],
        output='log',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('motor1_node_id'),'mode': launch.substitutions.LaunchConfiguration('motor1_mode')}]
    )
    # motor2 2. 创建控制节点
    action_node_motor2 = launch_ros.actions.Node(
        package='servo_motor_control',
        executable='servo_motor_node',
        name=[launch.substitutions.LaunchConfiguration('motor2_name')],
        output='log',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('motor2_node_id'),'mode': launch.substitutions.LaunchConfiguration('motor2_mode')}]
    )
    # motor1 3. 创建状态显示节点
    action_node_motor_status_display1 = launch_ros.actions.Node(
        package='motor_status_display',
        executable='motor_status_display_node',
        output='log',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('motor1_node_id')}]
    )
    # motor2 3. 创建状态显示节点
    action_node_motor_status_display2 = launch_ros.actions.Node(
        package='motor_status_display',
        executable='motor_status_display_node',
        output='log',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('motor2_node_id')}]
    )
    action_node_motor_taskpub = launch_ros.actions.Node(
        package='motor_taskpub',
        executable='motor_taskpub_node',
        output='screen',
        parameters=[{'node_num': launch.substitutions.LaunchConfiguration('node_num'),'node_id1': launch.substitutions.LaunchConfiguration('motor1_node_id')},{'node_id2': launch.substitutions.LaunchConfiguration('motor2_node_id')}]
    )
    # 3.合成启动描述并返回
    launch_description = launch.LaunchDescription([
        action_declare_arg_motor1_name,
        action_declare_arg_motor1_node_id,
        action_declare_arg_motor1_mode,
        action_node_motor1,

        action_declare_arg_motor2_name,
        action_declare_arg_motor2_node_id,
        action_declare_arg_motor2_mode,
        action_node_motor2,

        action_declare_arg_node_num,
        action_node_motor_taskpub,

        action_node_motor_status_display1,
        action_node_motor_status_display2,
    ])
    return launch_description