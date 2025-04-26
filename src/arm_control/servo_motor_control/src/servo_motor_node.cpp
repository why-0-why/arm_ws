#include "servo_motor_control/servo_motor_node.hpp"

class ServoMotorNode : public rclcpp::Node
{
private:
    rclcpp::Service<CommandFeedback>::SharedPtr servo_command_srv_;     // 指令服务，用于发送/接收指令并处理
    rclcpp::Publisher<MotorStatus>::SharedPtr servo_satatus_publisher_; // 发布者智能指针
    rclcpp::TimerBase::SharedPtr motor_status_pub_timer_;               // 发布定时器智能指针
    rclcpp::TimerBase::SharedPtr timer_send_;                           // 定时器智能指针
    // rclcpp::TimerBase::SharedPtr timer_heart; // 定时器智能指针

    /* 发送缓存 */
    uint32_t send_id_ = 0;
    uint8_t send_len_ = 0;
    uint8_t send_data_[8] = {0};
    uint32_t send_count_bfore_ = 0;

    /* 接收缓存 */
    uint32_t recv_id_ = 0;
    uint8_t recv_len_ = 0;
    uint8_t recv_data_[8] = {0};
    // SDO缓冲区
    uint32_t SDO_buffer_id_ = 0;
    uint8_t SDO_buffer_len_ = 0;
    uint8_t SDO_buffer_data_[8] = {0};

    /* canopen配置参数 */
    /* Motor参数 */
    uint8_t node_id_{0x00}; // 节点ID 0x01~0x7F
    CanTool can_;
    uint8_t motor_mode_{0x00};                 // 电机模式
    uint32_t motor_speed_set_{0};              // 电机设定速度
    uint32_t motor_position_set_{0};           // 电机设定位置
    uint32_t motor_speed_ = 0;                 // 电机实际速度
    uint32_t motor_position_relative_ = 0;     // 电机相对0点位置
    uint32_t motor_position_real_ = 0;         // 电机实际位置
    uint32_t motor_zero_point_ = 0;            // 电机0点位置
    bool zero_point_flag_ = false;             // 0点标志位
    bool motor_position_running_flag_ = false; // 电机就位标志位
    /* 参数设置句柄 */
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    /* 数据统计 */
    uint64_t data_count_[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 接收数据统计第11、12个是SDO发送、接收
public:
    /* @brief   获取控制电机参数,开启控制服务节点,开启电机状态发布节点,开启定时器持续发送指令,开启canopen接收线程
     * @param
     * @return
     */
    ServoMotorNode() : Node("servo_motor_node"), can_("can0", 1000000)
    {
        /* srv1.声明和获取参数 */
        this->declare_parameter("node_id", 0x00);
        this->declare_parameter("mode", 0);
        this->declare_parameter("speed_set", 0);
        this->declare_parameter("position_set", 0);

        this->get_parameter("node_id", node_id_);
        this->get_parameter("mode", motor_mode_);
        this->get_parameter("speed_set", motor_speed_set_);
        this->get_parameter("position_set", motor_position_set_);

        std::cout << "节点id:" << static_cast<int>(node_id_) << std::endl;
        /* srv2.创建生产端服务*/
        servo_command_srv_ = this->create_service<CommandFeedback>("servo_motor" + std::to_string(node_id_) + "_command", std::bind(&ServoMotorNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2)); // 指定服务名称，回调函数
        RCLCPP_INFO(this->get_logger(), "服务创建成功");
        /* srv3.服务回调函数 */
        // 见feedback_callback函数

        /* Motor1.canopen接收线程(数据处理RCL info展示) */
        std::thread recv_always_Thread([&]() -> void
                                       {
                                           while(1)
                                            {
                                                /* Recv0.清空缓冲区 */
                                                recv_id_ = 0;
                                                recv_len_ = 0;
                                                for (int i = 0; i < 8; i++)
                                                {
                                                    recv_data_[i] = 0;
                                                }
                                                /* Recv1.检查是否有数据 */
                                                if(can_.recv(&recv_id_, &recv_len_, recv_data_));
                                                else return;
                                                /* Recv2.处理数据 */
                                                if (recv_id_ == (uint32_t)(Heartbeat_ID + node_id_)) // 心跳包
                                                {   
                                                    data_count_[0]++; // 接收数据统计
                                                }
                                                else if (recv_id_ == (uint32_t)(SDO_RECV_ID + node_id_)) // 接收数据
                                                {
                                                    data_count_[12]++; // 接收数据统计
                                                    // 将recv_data_拷贝到SDO_buffer_data_
                                                    SDO_buffer_id_ = recv_id_;
                                                    SDO_buffer_len_ = recv_len_;
                                                    for (int i = 0; i < 8; i++)
                                                    {
                                                        SDO_buffer_data_[i] = recv_data_[i];
                                                    }
                                                }
                                            } });
        recv_always_Thread.detach(); // 分离线程，防止阻塞
        /*Motor2.设置电机模式*/
        if (!MotorSet(motor_mode_))
        {
            RCLCPP_INFO(this->get_logger(), "电机模式设置可能有问题");
            // return; // 初始化失败退出
        }
        /*Motor3.电机初始化*/
        if (!servo_motor_init())
        {
            RCLCPP_INFO(this->get_logger(), "电机初始化可能有问题");
            // return; // 初始化失败退出
        }

        /*Motor4.定时器发送指令*/
        RCLCPP_INFO(this->get_logger(), "定时器创建成功");
        timer_send_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ServoMotorNode::timer_send_callback, this)); // 定时器，周期50ms

        /* Pub1.创建发布者 */
        servo_satatus_publisher_ = this->create_publisher<MotorStatus>("servo_motor_status", 10); // 指定发布者名称和队列长度
        RCLCPP_INFO(this->get_logger(), "发布者创建成功");
        /* Pub2.创建定时器 */
        motor_status_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ServoMotorNode::motor_status_pub_timer_callback, this)); // 定时器，周期100ms
    }

    //////////////////////////////////////////////////////////////////////////
    //                             应用层
    //////////////////////////////////////////////////////////////////////////

    /* @brief 伺服电机初始化(cia402标准,控制字操作)
     * @param
     * @param
     * @return true:发送成功,false:发送失败
     */
    bool servo_motor_init()
    {
        /* Motor1.1.设置启动1（清除报错） */
        RCLCPP_INFO(this->get_logger(), "电机初始化中1");
        bool try_flag = false; // 尝试标志
        for (int try_count = 0; try_count < 3; try_count++)
        {
            if (SDO_send(8, startcommand[0]))
            {
                try_flag = true; // 尝试成功标志
                break;           // 尝试发送初始化指令1
            }
            else
            { // 等待100ms
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!try_flag)
            RCLCPP_INFO(this->get_logger(), "清除报错失败，可能无错误"); // 清除报错失败不退出

        // 睡眠0.05s
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        /* Motor1.2.设置启动2 */
        RCLCPP_INFO(this->get_logger(), "电机初始化中2");
        try_flag = false; // 尝试标志重置
        for (int try_count = 0; try_count < 3; try_count++)
        {
            if (SDO_send(8, startcommand[1]))
            {
                try_flag = true; // 尝试成功标志
                break;           // 尝试发送初始化指令2
            }
            else
            { // 等待100ms
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!try_flag)
            return try_flag; // 尝试失败退出

        // 睡眠0.05s
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        /* Motor1.3.设置启动3 */
        RCLCPP_INFO(this->get_logger(), "电机初始化中3");
        try_flag = false; // 尝试标志
        for (int try_count = 0; try_count < 3; try_count++)
        {
            if (SDO_send(8, startcommand[2]))
            {
                try_flag = true; // 尝试成功标志
                break;           // 尝试发送初始化指令3
            }
            else
            { // 等待100ms
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!try_flag)
            return try_flag; // 尝试失败退出

        // 睡眠0.05s
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        /* Motor1.4.设置启动4 */
        RCLCPP_INFO(this->get_logger(), "电机初始化中4");
        try_flag = false; // 尝试标志
        for (int try_count = 0; try_count < 3; try_count++)
        {
            if (SDO_send(8, startcommand[3]))
            {
                try_flag = true; // 尝试成功标志
                break;           // 尝试发送初始化指令4
            }
            else
            { // 等待100ms
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!try_flag)
            return try_flag; // 尝试失败退出

        RCLCPP_INFO(this->get_logger(), "电机初始化完成");
        return try_flag;
    }

    /* @brief canopen设置电机模式
     * @param mode 电机模式(0:速度模式,1:位置模式,2:测试模式)
     * @return true:设置成功,false:设置失败
     */
    bool MotorSet(uint8_t mode)
    {
        bool set_flag = false; // 设置标志
        switch (mode)
        {
        case Motor_Speed_mode:
            if (SDO_send(8, set_speed_command) && SDO_send(8, motor_speed_command))
            {
                RCLCPP_INFO(this->get_logger(), "设置电机模式：速度模式成功");
                motor_mode_ = Motor_Speed_mode;
                set_flag = true;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "设置电机模式：速度模式失败");
            }
            break;
        case Motor_Position_mode:
            sleep(0.1); // 等待100ms
            /* Pos1.原点定位模式下设置初始位置 */
            if (!zero_point_flag_) // 最高位为1，说明原点设置成功
            {
                SDO_send(8, motor_position_read_real);                                                                                                                // 发送原点设置指令
                motor_position_real_ = SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256;     // 读取绝对位置信息
                motor_zero_point_ = SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256;        // 读取绝对位置信息
                SDO_send(8, motor_position_read_relative);                                                                                                            // 发送原点设置指令
                motor_position_relative_ = SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256; // 读取相对位置信息
                motor_zero_point_ -= SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256;       // 减去相对位置信息
                zero_point_flag_ = true;                                                                                                                              // 原点设置成功标志置位
                RCLCPP_INFO(this->get_logger(), "原点绝对位置：%d", motor_zero_point_);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "原点绝对位置：%d", motor_zero_point_);
            }
            /* Pos2.设置位置模式 */
            if (SDO_send(8, set_position_command))
            {
                RCLCPP_INFO(this->get_logger(), "设置电机模式：位置模式成功");
                set_flag = true;
                motor_mode_ = Motor_Position_mode;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "设置电机模式：位置模式失败");
            }
            break;
        case Motor_Test_mode:
            if (SDO_send(8, set_position_command) && SDO_send(8, motor_position_start))
            {
                RCLCPP_INFO(this->get_logger(), "设置电机模式：测试模式成功");
                motor_mode_ = Motor_Test_mode;
                set_flag = true;
            }
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "参数错误");
            break;
        }
        return set_flag;
    }

    /* @brief 伺服电机轮询定时器-回调函数
     *        定时查看电机状态
     * @param 无
     * @return 无
     */
    void timer_send_callback()
    {
        switch (motor_mode_)
        {
            /* Motor4.1.速度模式下发送读取速度指令 */
        case Motor_Speed_mode:
            if (SDO_send(8, motor_speed_read))
            {
                motor_speed_ = SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256; // 读取速度信息
                // RCLCPP_INFO(this->get_logger(), "实时速度：%d", motor_speed_);                                                                            // 发送成功速度信息
            }
            break;
        case Motor_Position_mode:
            if (SDO_send(8, motor_position_read_relative))
            {
                // RCLCPP_INFO(this->get_logger(), "SDO数据：%x, %x, %x, %x", SDO_buffer_data_[4], SDO_buffer_data_[5], SDO_buffer_data_[6], SDO_buffer_data_[7]);
                motor_position_relative_ = SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256; // 读取位置信息
                // RCLCPP_INFO(this->get_logger(), "相对位置：%d", motor_position_relative_);                                                                            // 发送成功位置信息
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "发送指令失败"); // 发送失败打印信息
            }
            if (SDO_send(8, motor_position_read_real))
            {
                // RCLCPP_INFO(this->get_logger(), "SDO数据：%x, %x, %x, %x", SDO_buffer_data_[4], SDO_buffer_data_[5], SDO_buffer_data_[6], SDO_buffer_data_[7]);
                motor_position_real_ = SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256; // 读取位置信息
                // RCLCPP_INFO(this->get_logger(), "绝对位置：%d", motor_position_real_);                                                                            // 发送成功位置信息
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "发送指令失败"); // 发送失败打印信息
            }
            if (SDO_send(8, motor_speed_read))
            {
                // RCLCPP_INFO(this->get_logger(), "SDO数据：%x, %x, %x, %x", SDO_buffer_data_[4], SDO_buffer_data_[5], SDO_buffer_data_[6], SDO_buffer_data_[7]);
                motor_speed_ = SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256; // 读取速度信息
                // RCLCPP_INFO(this->get_logger(), "实时速度：%d", motor_speed_);                                                                            // 发送成功速度信息
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "发送指令失败"); // 发送失败打印信息
            }
            if (SDO_send(8, motor_find0_ifOK))
            {
                // RCLCPP_INFO(this->get_logger(), "SDO数据：%x, %x, %x, %x", SDO_buffer_data_[4], SDO_buffer_data_[5], SDO_buffer_data_[6], SDO_buffer_data_[7]);
                if (((SDO_buffer_data_[5] & 0x10) >> 2) == 0)
                    motor_position_running_flag_ = false; // 已到位
                else
                    motor_position_running_flag_ = true; // 未到位或者未执行
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "发送指令失败"); // 发送失败打印信息
            }
            break;
        case Motor_Test_mode:
            if (SDO_send(8, motor_position_read_relative))
            {
                // RCLCPP_INFO(this->get_logger(), "实时位置：%d", SDO_buffer_data_[4] + SDO_buffer_data_[5] * 256 + SDO_buffer_data_[6] * 256 * 256 + SDO_buffer_data_[7] * 256 * 256 * 256); // 发送成功打印信息
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "发送指令失败"); // 发送失败打印信息
            }
            break;
        default:
            // RCLCPP_INFO(this->get_logger(), "电机模式错误");
            break;
        }
    }

    /* @brief 伺服电机状态发布定时器-回调函数(组织、发布电机状态信息)
     * @param 无
     * @return 无
     */
    void motor_status_pub_timer_callback()
    {
        MotorStatus msg;
        msg.node_id = node_id_;
        msg.motor_mode = motor_mode_;
        msg.motor_speed_set = motor_speed_set_;
        msg.motor_position_set = motor_position_set_;
        msg.motor_speed = motor_speed_;
        msg.motor_position_real = motor_position_real_;
        msg.motor_position_relative = motor_position_relative_;
        msg.motor_zero_point = motor_zero_point_;
        msg.motor_position_running_flag = motor_position_running_flag_;
        servo_satatus_publisher_->publish(msg); // 发布电机状态信息
    }

    /* @breif 控制电机服务
     * @param 无
     * @return 无
     */
    void feedback_callback(const std::shared_ptr<CommandFeedback::Request> request, const std::shared_ptr<CommandFeedback::Response> response)
    {
        response->status = response->FAILURE; // 默认反馈失败
        switch (request->command_num)
        {
        case SET_MODE:
            if (MotorSet(request->data[0]))
                response->status = response->SUCCESS; // 反馈成功
            break;
        case SAVE:
            if (SDO_send(8, save_command))
                response->status = response->SUCCESS; // 反馈成功
            break;
        case SET_SPEED:
            if (motor_mode_ == Motor_Speed_mode) // 模式正确
            {
                motor_speed_command[4] = request->data[0];
                motor_speed_command[5] = request->data[1];
                motor_speed_command[6] = request->data[2];
                motor_speed_command[7] = request->data[3];
                if (SDO_send(8, motor_speed_command))
                {
                    response->status = response->SUCCESS; // 反馈成功
                    motor_speed_set_ = request->data[0] + request->data[1] * 256 + request->data[2] * 256 * 256 + request->data[3] * 256 * 256 * 256;
                }
            }
            break;
        case SET_POS:
            if (motor_mode_ == Motor_Position_mode) // 模式正确
            {
                motor_position_set_target[4] = request->data[0];
                motor_position_set_target[5] = request->data[1];
                motor_position_set_target[6] = request->data[2];
                motor_position_set_target[7] = request->data[3];
                // RCLCPP_WARN(this->get_logger(), "设置目标位置：%d", motor_position_set_target[4] + motor_position_set_target[5] * 256 + motor_position_set_target[6] * 256 * 256 + motor_position_set_target[7] * 256 * 256 * 256);
                if (SDO_send(8, motor_position_set_target) && SDO_send(8, motor_position_start)) // 设置目标位置并启动
                {
                    response->status = response->SUCCESS; // 反馈成功
                    motor_position_set_ = request->data[0] + request->data[1] * 256 + request->data[2] * 256 * 256 + request->data[3] * 256 * 256 * 256;
                }
            }
            break;
        case SET_TEST:
            if (motor_mode_ != Motor_Test_mode)
            {
                /* 组织常规指令 */
                command[0] = request->command;
                command[1] = request->index[0];
                command[2] = request->index[1];
                command[3] = request->subindex;
                command[4] = request->data[0];
                command[5] = request->data[1];
                command[6] = request->data[2];
                command[7] = request->data[3];
                if (SDO_send(8, command))
                    response->status = response->SUCCESS; // 反馈成功
            }
            break;
        default:
            if (servo_motor_init())
                response->status = response->SUCCESS;
            break;
        }
    }

    //////////////////////////////////////////////////////////////////////////
    //                             协议层
    //////////////////////////////////////////////////////////////////////////

    /* @brief SDO发送函数(组织SDO指令,多次尝试发送,并等待SDO数据返回)
     * @param len: SDO指令长度
     *        data: SDO指令数据
     * @return bool: 成功返回true,失败返回false
     */
    bool SDO_send(uint8_t len, uint8_t *data)
    {
        bool recvflag = false; // 接收标志
        /* SDO0.清空缓冲区 */
        SDO_buffer_id_ = 0;
        SDO_buffer_len_ = 0;
        for (int i = 0; i < len; i++)
        {
            SDO_buffer_data_[i] = 0;
        }
        /* SDO1.尝试发送 */
        for (int try_count = 0; try_count < 5; try_count++)
        {
            send_count_bfore_ = data_count_[12];          // 记录发送数据前的接收数据统计
            can_.send(SDO_SEND_ID + node_id_, len, data); // 发送初始化指令

            /* SDO2.等待接收数据或超时重试 */
            bool timeoutflag = false;                                // 超时标志
            auto send_start_time = std::chrono::system_clock::now(); // 获取并记录当前时间
            while (data_count_[12] - send_count_bfore_ < 1)
            {
                if (std::chrono::system_clock::now() - send_start_time > std::chrono::milliseconds(15)) // 超时时间为15ms
                {
                    timeoutflag = true; // 超时标志置位
                    break;
                }
            }

            /* SDO3.接收并处理数据 */
            if (timeoutflag) // 判断超时
            {
                continue; // 超时重试
                // RCLCPP_INFO(this->get_logger(), "SDO接收超时");
            }
            else if (data_count_[12] - send_count_bfore_ > 0)
            {
                if (SDO_buffer_id_ == (uint32_t)(SDO_RECV_ID + node_id_) && SDO_buffer_data_[0] != 0x80)
                {
                    recvflag = true;
                    break; // 接收成功退出
                } // 设置成功退出
                else
                    RCLCPP_INFO(this->get_logger(), "SDO设置失败"); // 设置失败重试
            }
            /* SDO4.错误处理 */
            else
            {
                RCLCPP_INFO(this->get_logger(), "SDO未知错误");
            }
        }
        return recvflag; // 接收成功返回true
    }
};

int main(int argc, char **argv)
{
    // 主线程
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}