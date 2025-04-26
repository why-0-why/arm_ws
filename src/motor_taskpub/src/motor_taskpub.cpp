#include "rclcpp/rclcpp.hpp"
#include "servo_motor_interface/srv/command_feedback.hpp"
#include "servo_motor_interface/msg/motor_status.hpp"
#include "../../servo_motor_control/include/servo_motor_control/servo_motor_node.hpp"

using CommandFeedback = servo_motor_interface::srv::CommandFeedback;
using MotorStatus = servo_motor_interface::msg::MotorStatus;
/*测试用变量*/
uint16_t eachstep = 1000;
uint16_t totalstep = 20;

class MotorTaskpub : public rclcpp::Node
{
private:
    rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli1_;          // 指令客户端，用于发送指令
    rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli2_;          // 指令客户端，用于发送指令
    rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli3_;          // 指令客户端，用于发送指令
    rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli4_;          // 指令客户端，用于发送指令
    rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli5_;          // 指令客户端，用于发送指令
    rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli6_;          // 指令客户端，用于发送指令
    rclcpp::Subscription<MotorStatus>::SharedPtr servo_status_subscription_; // 订阅者智能指针

    /* 对应电机参数 */
    uint8_t node_num_{2};
    uint8_t node_id_[6] = {};
    uint8_t motor_index_ = 0;
    // bool motor_status_flag_[6] = {0, 0, 0, 0, 0, 0};                // 电机运行状态缓存
    uint16_t position_index_[6] = {0, 0, 0, 0, 0, 0};               // 位置缓存索引（0~999），刷新缓存/执行缓存用
    uint16_t position_top_[6] = {totalstep, totalstep, 0, 0, 0, 0}; // 位置缓存上限（position_index_==position_top_时，不再发送位置指令）
    int32_t position_buf_[6][1000] = {};

public:
    MotorTaskpub() : Node("motor_taskpub_node")
    {
        /* Cli1.声明和获取参数 */
        this->declare_parameter("node_num", 0); // 节点数量1~6
        this->get_parameter("node_num", node_num_);
        for (int i = 0; i < node_num_; i++)
        {
            this->declare_parameter("node_id" + std::to_string(i + 1), 0x00);
            this->get_parameter("node_id" + std::to_string(i + 1), node_id_[i]);
        }
        /* Cli2.创建客户端 */
        for (int i = 0; i < node_num_; i++)
        {
            std::string node_name = "servo_motor" + std::to_string(node_id_[i]) + "_command";
            switch (i)
            {
            case 0:
                servo_command_cli1_ = this->create_client<CommandFeedback>(node_name);
                break;
            case 1:
                servo_command_cli2_ = this->create_client<CommandFeedback>(node_name);
                break;
            case 2:
                servo_command_cli3_ = this->create_client<CommandFeedback>(node_name);
                break;
            case 3:
                servo_command_cli4_ = this->create_client<CommandFeedback>(node_name);
                break;
            case 4:
                servo_command_cli5_ = this->create_client<CommandFeedback>(node_name);
                break;
            case 5:
                servo_command_cli6_ = this->create_client<CommandFeedback>(node_name);
                break;
            default:
                break;
            }
        }

        /* Sub1.创建订阅者 */
        servo_status_subscription_ = this->create_subscription<MotorStatus>(
            "/servo_motor_status", 10,
            [this](const MotorStatus::SharedPtr msg)
            {
                uint8_t which_motor = 255;
                /*Motorrun0.确定节点编号*/
                for (int i = 0; i < node_num_; i++)
                {
                    if (msg->node_id == node_id_[i])
                    {
                        which_motor = i;
                        break;
                    }
                }
                // /* Motorrun1.确定电机未运行 */
                // if (msg->motor_position_running_flag == 1)
                //     return;
                // else if (msg->motor_position_running_flag == 0)
                // {
                //     // motor_status_flag_[which_motor] = 0;
                //     RCLCPP_INFO(this->get_logger(), "Motor %d is not running", which_motor);
                // }

                if (position_index_[which_motor] <= position_top_[which_motor])
                //&& msg->motor_position_running_flag == 0&& position[index_falg]!=0&& motor_status_flag_[which_motor] == 0
                {
                    /* Motorrun2.发送位置指令 */
                    /* Cli2.构造请求 */
                    auto request = std::make_shared<CommandFeedback::Request>();
                    request->command_num = SET_POS; // 假设位置指令的命令编号为0xf3
                    // request->data[0] = (uint8_t)(position_buf_[position_index_[which_motor]] >> 0);
                    // request->data[1] = (uint8_t)(position_buf_[position_index_[which_motor]] >> 8);
                    // request->data[2] = (uint8_t)(position_buf_[position_index_[which_motor]] >> 16);
                    // request->data[3] = (uint8_t)(position_buf_[position_index_[which_motor]]>> 24);
                    request->data[0] = (uint8_t)((-40000 - eachstep * position_index_[which_motor]) >> 0);
                    request->data[1] = (uint8_t)((-40000 - eachstep * position_index_[which_motor]) >> 8);
                    request->data[2] = (uint8_t)((-40000 - eachstep * position_index_[which_motor]) >> 16);
                    request->data[3] = (uint8_t)((-40000 - eachstep * position_index_[which_motor]) >> 24);
                    /* Cli3.发送请求 */
                    position_index_[which_motor] = (position_index_[which_motor] + 1) % totalstep;
                    // motor_status_flag_[which_motor] = 1;
                    switch (which_motor)
                    {
                    case 0:
                        servo_command_cli1_->async_send_request(request);
                        break;
                    case 1:
                        servo_command_cli2_->async_send_request(request);
                        break;
                    case 2:
                        servo_command_cli3_->async_send_request(request);
                        break;
                    case 3:
                        servo_command_cli4_->async_send_request(request);
                        break;
                    case 4:
                        servo_command_cli5_->async_send_request(request);
                        break;
                    case 5:
                        servo_command_cli6_->async_send_request(request);
                        break;
                    default:
                        break;
                    }
                }
            });
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTaskpub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}