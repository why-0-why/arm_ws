import launch
import launch_ros

def generate_launch_description():
    # 1.声明launch参数
    position_mode = launch.actions.DeclareLaunchArgument('position_mode', default_value='2')# 电机模式 1:速度 2：位置 3：测试模式

    # 关节电机参数
    joint1_node_id = launch.actions.DeclareLaunchArgument('joint1_node_id', default_value='3')
    joint1_name = launch.actions.DeclareLaunchArgument('joint1_name', default_value='joint1')


    joint2_node_id = launch.actions.DeclareLaunchArgument('joint2_node_id', default_value='2')
    joint2_name = launch.actions.DeclareLaunchArgument('joint2_name', default_value='joint2')

    # 控制器参数
    M3508_reduction_ratio = launch.actions.DeclareLaunchArgument('M3508_reduction_ratio', default_value='19')# M3508减速比1:19
    M3508_encode_num= launch.actions.DeclareLaunchArgument('M3508_encode_num', default_value='8192')# M3508转子编码线数8192
    M2006_reduction_ratio = launch.actions.DeclareLaunchArgument('M2006_reduction_ratio', default_value='36')# M2006减速比1:36
    M2006_encode_num = launch.actions.DeclareLaunchArgument('M2006_encode_num', default_value='8192')# M2006转子编码线数8192    
    controller1_name = launch.actions.DeclareLaunchArgument('controller1_name', default_value='C1')
    controller2_name = launch.actions.DeclareLaunchArgument('controller2_name', default_value='C2')
    controller1_node_id = launch.actions.DeclareLaunchArgument('controller1_node_id', default_value='1')
    controller2_node_id = launch.actions.DeclareLaunchArgument('controller2_node_id', default_value='2')


    # 2.创建节点，把参数传入给节点
    # 控制器节点
    action_node_controller1 = launch_ros.actions.Node(
        package='angle_read',
        executable='angle_read_node',
        name=[launch.substitutions.LaunchConfiguration('controller1_name')],
        output='screen',
        parameters=[{'controller_node_id': launch.substitutions.LaunchConfiguration('controller1_node_id'),
                     'reduction_ratio': launch.substitutions.LaunchConfiguration('M3508_reduction_ratio'), 
                     'encode_num': launch.substitutions.LaunchConfiguration('M3508_encode_num'), 
                     'controled_node_id': launch.substitutions.LaunchConfiguration('joint1_node_id')}]
    )
    action_node_controller2 = launch_ros.actions.Node(
        package='angle_read',
        executable='angle_read_node',
        name=[launch.substitutions.LaunchConfiguration('controller2_name')],
        output='screen',
        parameters=[{'controller_node_id': launch.substitutions.LaunchConfiguration('controller2_node_id'),'reduction_ratio': launch.substitutions.LaunchConfiguration('M3508_reduction_ratio'), 'encode_num': launch.substitutions.LaunchConfiguration('M3508_encode_num'), 'controled_node_id': launch.substitutions.LaunchConfiguration('joint2_node_id')}]
    )

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
    # 状态显示节点
    action_node_motor_status_display1 = launch_ros.actions.Node(
        package='motor_status_display',
        executable='motor_status_display_node',
        output='screen',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('joint1_node_id'),'controller_node_id': launch.substitutions.LaunchConfiguration('controller1_node_id')}]
    )
    action_node_motor_status_display2 = launch_ros.actions.Node(
        package='motor_status_display',
        executable='motor_status_display_node',
        output='screen',
        parameters=[{'node_id': launch.substitutions.LaunchConfiguration('joint2_node_id'),'controller_node_id': launch.substitutions.LaunchConfiguration('controller2_node_id')}]
    )
    # 3.合成启动描述并返回
    launch_description = launch.LaunchDescription([
        # 电机参数
        position_mode,
        # 控制器参数
        M3508_reduction_ratio,
        M3508_encode_num,
        M2006_reduction_ratio,
        M2006_encode_num, 
        # joint1的节点
        joint1_node_id,
        joint1_name,
        action_node_joint1,
        # joint2的节点
        joint2_node_id,
        joint2_name,
        action_node_joint2,

        # 控制器节点
        controller1_node_id,
        controller1_name,
        action_node_controller1,
        controller2_node_id,
        controller2_name,
        action_node_controller2,

        # 状态显示节点
        action_node_motor_status_display1,
        action_node_motor_status_display2,
        

    ])
    return launch_description