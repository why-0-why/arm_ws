import launch
import launch_ros

## 控制端节点启动 ##
def generate_launch_description():
    # 1.声明launch参数
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
        # 控制器参数
        M3508_reduction_ratio,
        M3508_encode_num,
        M2006_reduction_ratio,
        M2006_encode_num, 

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