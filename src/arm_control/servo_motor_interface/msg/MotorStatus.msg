uint8 node_id # canid of the motor
uint8 motor_mode # 电机模式 1:速度 2：位置 3：测试模式
uint32 motor_speed_set # 设定目标速度
uint32 motor_position_set # 设定目标位置
uint32 motor_speed # 当前实际速度
uint32 motor_position_real # 当前实际位置
uint32 motor_position_relative # 当前相对位置
uint32 motor_zero_point # 零点位置
bool motor_position_running_flag # 位置模式运行标志位