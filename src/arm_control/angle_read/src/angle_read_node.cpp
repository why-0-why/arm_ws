#include "rclcpp/rclcpp.hpp"
#include "servo_motor_interface/srv/command_feedback.hpp"
#include "can_tool/can_tool.hpp"
#include "servo_motor_control/servo_motor_node.hpp"

using CommandFeedback = servo_motor_interface::srv::CommandFeedback;

#define READ_ID 0x200

class AngelReadNode : public rclcpp::Node
{
private:
    rclcpp::Service<CommandFeedback>::SharedPtr controller_command_srv_; // 指令服务，用于接收指令
    rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli_;       // 指令客户端，用于发送指令

    rclcpp::TimerBase::SharedPtr timer_send_; // 定时器智能指针
    CanTool can_;
    /* 接收缓存 */
    uint32_t recv_id_ = 0;
    uint8_t recv_len_ = 0;
    uint8_t recv_data_[8] = {0};
    /* 控制器电机机电参数 */
    uint8_t controller_node_id_{0};
    uint8_t reduction_ratio_{0};
    uint16_t encode_num_{0};
    /*被控制电机机电参数*/
    uint8_t controled_node_id_{0};
    uint8_t controled_reduction_ratio_{50};
    uint16_t controled_encode_num_{16384};
    /*控制器电机数据*/
    bool zero_request_flag_ = 0;
    double rotor_angle_;
    double axis_angle_;
    /*被控制电机数据*/
    int32_t controled_position_;
    /* 窗口滤波器 */
#define WINDOW_SIZE 20 // 窗口大小
    long window_[WINDOW_SIZE];
    long window_index_ = 0;

public:
    AngelReadNode() : Node("angel_read_node"), can_("can0", 1000000)
    {
        param_ini(); // 参数初始化

        /* Cli1.创建客户端,用以发布伺服电机控制服务 */
        servo_command_cli_ = this->create_client<CommandFeedback>("servo_motor" + std::to_string(controled_node_id_) + "_command");

        /* Serv1.创建服务,用于接收控制器服务请求 */
        controller_command_srv_ = this->create_service<CommandFeedback>("controller" + std::to_string(controller_node_id_) + "_command", std::bind(&AngelReadNode::controller_command_callback, this, std::placeholders::_1, std::placeholders::_2));

        /* AngleRead1.创建线程,用于接收can总线数据 */
        std::thread recv_always_Thread(std::bind(&AngelReadNode::angle_read_pro, this));

        recv_always_Thread.detach(); // 分离线程，防止阻塞

        /* Cli2.创建定时器,用于定时发送位置指令 */
        timer_send_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AngelReadNode::timer_send_callback, this)); // 定时器，周期100ms
    }

    /* @brief ros节点参数声明与获取
     * @param 无
     * @return 无
     */
    void param_ini()
    {

        /* Cli1.声明和获取参数 */
        // 控制器电机
        this->declare_parameter("controller_node_id", 0);
        this->declare_parameter("reduction_ratio", 36);
        this->declare_parameter("encode_num", 8192);
        // 被控电机
        this->declare_parameter("controled_node_id", 0x00);

        this->get_parameter("controller_node_id", controller_node_id_);
        this->get_parameter("reduction_ratio", reduction_ratio_);
        this->get_parameter("encode_num", encode_num_);
        this->get_parameter("controled_node_id", controled_node_id_);
    }

    /* @brief 接受can总线电调数据,解析转子角度,记录多圈编码值
     * @param 无
     * @return 无
     */
    void angle_read_pro()
    {
        while (1)
        {
            /* Recv0.清空缓冲区 */
            recv_id_ = 0;
            recv_len_ = 0;
            for (int i = 0; i < 8; i++)
            {
                recv_data_[i] = 0;
            }
            /* Recv1.检查是否有数据 */
            if (can_.recv(&recv_id_, &recv_len_, recv_data_))
                ;
            else
                return;
            /* Recv2.处理数据 */
            if (recv_id_ == (uint32_t)(READ_ID + controller_node_id_)) // RM电调数据
            {
                rotor_angle_ = (recv_data_[0] * 255 + recv_data_[1]) * 360.0f / encode_num_;
                axis_angle_ = this->GetAxisAngle(recv_data_[0] * 256 + recv_data_[1]) * 360.0f / (encode_num_ * reduction_ratio_);
                controled_position_ = this->simpleMovingAverage(controled_position_); // 滤波
                RCLCPP_INFO(this->get_logger(), "%d电机转子角度%f", controller_node_id_, rotor_angle_);
                RCLCPP_INFO(this->get_logger(), "电机轴角度%f", axis_angle_);
            }
        }
    }

    /* @brief 计算机械轴多圈编码值计算
     * @param encoder_value 转子单圈编码器数值(范围:0~encode_num_)
     * @return 机械轴多圈编码值
     */
    long GetAxisAngle(long encoder_value)
    {
        long axis_encoder;               // 机械轴多圈编码值
        static int rotor_count;          // 转子圈数
        static bool init = 0;            // 计一次运行改为1，用于数据缓存初始化
        static long zero;                // 零点位置
        static int encoder_buf[5] = {0}; // 绝对值编码器数据缓存

        if (zero_request_flag_)
        {
            init = 0;
            zero_request_flag_ = 0;
        }

        encoder_buf[0] = encoder_value; // 读取数据缓存

        if (!init)
        { // 开机初始化
            init = 1;
            encoder_buf[1] = encoder_buf[2] = encoder_buf[3] = encoder_buf[0];
            rotor_count = 0;
            zero = encoder_buf[0];
        }

        if (abs(encoder_buf[0] - encoder_buf[1]) > (encode_num_ + 500))
        { // 去除异常数据
            encoder_buf[0] = encoder_buf[1] + (encoder_buf[1] - encoder_buf[2]);
        }

        if ((encoder_buf[0] - encoder_buf[1]) > encode_num_ * 0.9)
        { // 提升或圆盘顺时针过程,数值减小
            rotor_count--;
        }
        else if ((encoder_buf[1] - encoder_buf[0]) > encode_num_ * 0.9)
        { // 下降或圆盘逆时针过程,数值增加
            rotor_count++;
        }

        axis_encoder = rotor_count * encode_num_ + encoder_buf[0] - zero;

        encoder_buf[3] = encoder_buf[2];
        encoder_buf[2] = encoder_buf[1];
        encoder_buf[1] = encoder_buf[0];

        return (axis_encoder);
    }

    /*
     * @brief 请求canopen电机控制器发送位置指令
     * @param position 目标位置(一圈编码值是819200)
     * @return 无
     */
    void send_position_command(int32_t position)
    {
        /* Cli3.构造请求 */
        auto request = std::make_shared<CommandFeedback::Request>();
        request->command_num = SET_POS; // 假设位置指令的命令编号为0xf3
        request->data[0] = (uint8_t)(-position >> 0);
        request->data[1] = (uint8_t)(-position >> 8);
        request->data[2] = (uint8_t)(-position >> 16);
        request->data[3] = (uint8_t)(-position >> 24);
        /* Cli4.发送请求 */
        servo_command_cli_->async_send_request(request);
    }

    /* @brief 控制器服务请求回调函数
     * @param request 请求数据
     * @param response 响应数据
     * @return 无
     */
    void controller_command_callback(const std::shared_ptr<CommandFeedback::Request> request, const std::shared_ptr<CommandFeedback::Response> response)
    {
        response->status = CommandFeedback::Response::FAILURE; // 默认失败
        switch (request->command_num)
        {
        case SET_ZERO:
            zero_request_flag_ = 1;
            while (zero_request_flag_)
                ;
            response->status = CommandFeedback::Response::SUCCESS;
            break;
        default:
            break;
        }
    }

    /*
     * @brief 定时回调-定时请求伺服电机控制服务,发送位置指令
     * @param 无
     * @return 无
     */
    void timer_send_callback()
    {
        controled_position_ = axis_angle_ / 360.0f * controled_encode_num_ * controled_reduction_ratio_;

        /* 发送位置指令 */
        send_position_command(controled_position_);
    }

    /**
     * @brief 简单的滑动平均滤波器函数,每次输入一个新值，返回滤波后的值。
     *
     * @param newData 输入的新值
     * @param size 窗口大小（仅在第一次调用时设置）
     * @return 滤波后的值
     *
     * @note
     * - 第一次调用时需要设置窗口大小。
     * - 窗口大小必须大于0。
     */
    long simpleMovingAverage(long newData)
    {
        // 将新数据放入窗口中
        window_[window_index_] = newData;
        window_index_ = (window_index_ + 1) % WINDOW_SIZE; // 更新索引，循环使用窗口

        // 计算窗口内数据的总和
        long sum = 0;
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            sum += window_[i];
        }
        return sum / WINDOW_SIZE; // 返回滤波后的值
    }
};

int main(int argc, char **argv)
{
    // 主线程
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AngelReadNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}