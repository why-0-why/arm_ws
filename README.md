# 机械臂控制项目

## 当前目标

1. 用rviz实现机械臂的数字孪生
2. 用moveit实现机械臂的规划算法
3. 实现多机共同开发

## 硬件

- 机械臂电机：通讯方式为canopen,通讯协议为cia402协议的伺服电机

- 控制器电机: RobotMaster M3508和M006

## 软件开发进度

### 环境

- ubuntu 24.04 LTS 
- ROS2 Jazzy

### 功能包概览

#### 控制包

- can_tool:can通信工具类,实现write和read功能

- servo_motor_interface:为控制canopen协议的伺服电机设计的消息接口,在本项目中多次用到(有待完善)

- servo_motor_control:canopen协议的伺服电机控制逻辑的实现,提供一个服务(用于接受控制指令),发布电机状态话题(用于显示电机状态)，发布标准joint_status（rviz显示）

- angle_read:判断控制器是否在线；通过can读取C610和C620的数据,映射控制器电机(RM电机,M3508和M2006)的轴角度,并请求伺服电机控制服务(以实现主从控制)

- motor_status_display:Qt5实现的人机交互界面,显示伺服电机与控制器的状态,提供控制按钮(位置指令 ，控制器置零，__小范围点动__).


#### 仿真模拟包

- arm_description：提供gazebo世界sdf文件，机器人xacro及urdf文件。
- arm_bringup：提供仿真、模型展示的launch文件。

#### 规划算法包

考虑用moveit
