# 这是一个机械臂控制项目

## 硬件

- 机械臂电机：通讯方式为canopen,通讯协议为cia402协议的伺服电机

- 控制器电机: RobotMaster M3508和M006

## 软件开发进度

### 功能包概览

#### 控制包

- can_tool:can通信工具类,实现write和read功能

- servo_motor_interface:为控制canopen协议的伺服电机设计的消息接口,在本项目中多次用到(有待完善)

- servo_motor_control:canopen协议的伺服电机控制逻辑的实现,提供一个服务(用于接受控制指令),发布一个话题(用于显示电机状态)

- angle_read:通过can读取C610和C620的数据,映射控制器电机(RM电机,M3508和M2006)的轴角度,并请求伺服电机控制服务(以实现主从控制)

- motor_status_display:Qt5实现的人机交互界面,显示伺服电机与控制器的状态,提供控制按钮.

#### 规划算法包

考虑用moveit

#### 仿真模拟包

gazebo和rviz,目前只考虑用rviz做显示