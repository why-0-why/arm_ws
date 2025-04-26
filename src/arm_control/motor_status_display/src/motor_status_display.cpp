#include "rclcpp/rclcpp.hpp"
#include "servo_motor_interface/srv/command_feedback.hpp"
#include "servo_motor_interface/msg/motor_status.hpp"
#include "servo_motor_control/servo_motor_node.hpp"

using CommandFeedback = servo_motor_interface::srv::CommandFeedback;
using MotorStatus = servo_motor_interface::msg::MotorStatus;

/*Qt头文件*/
#include <QApplication> //Qt应用
#include <QLabel>       //Qt标签
#include <QPushButton>  //Qt按钮
#include <QLineEdit>    //Qt文本输入框
#include <QString>      //Qt字符串
#include <QVBoxLayout>  //Qt垂直布局
#include <QHBoxLayout>  //Qt水平布局
#include <QFormLayout>  //Qt表单布局
#include <QMessageBox>  //Qt消息框
#include <QObject>      //包含QObject头文件

class MotorStatusDisplay : public rclcpp::Node, public QObject
{
private:
  rclcpp::Client<CommandFeedback>::SharedPtr servo_command_cli_;           // 指令客户端，用于发送指令
  rclcpp::Subscription<MotorStatus>::SharedPtr servo_status_subscription_; // 订阅者智能指针
  rclcpp::Client<CommandFeedback>::SharedPtr controller_command_srv_;      // 控制器指针

  QLabel *label_;
  QPushButton *send_button_speed_mode_;    // 发送速度指令的按钮
  QLineEdit *speed_line_edit_;             // 速度指令的文本输入框
  QPushButton *send_button_position_mode_; // 发送位置指令的按钮
  QLineEdit *position_line_edit_;          // 位置指令的文本输入框
  QPushButton *send_button_change_mode_;   // 发送修改模式指令的按钮
  QLineEdit *change_mode_line_edit_;       // 修改模式的文本输入框
  QPushButton *send_button_reset;          // 发送控制器清零/电机回零点的按钮
  QVBoxLayout *v_layout_;
  QHBoxLayout *h_layout_;
  QFormLayout *form_layout_;
  QWidget *widget_; // 主窗口

  /* 对应电机参数 */
  uint8_t node_id_{0x00};
  /* 对应控制器参数 */
  uint8_t controller_node_id_{0x00};

public:
  MotorStatusDisplay() : Node("motor_status_display"), QObject()
  {
    /* Cli1.声明和获取参数 */
    this->declare_parameter("node_id", 0x00);
    this->declare_parameter("controller_node_id", 0);

    this->get_parameter("node_id", node_id_);
    this->get_parameter("controller_node_id", controller_node_id_);
    /* Cli2.创建客户端 */
    servo_command_cli_ = this->create_client<CommandFeedback>("servo_motor" + std::to_string(node_id_) + "_command");
    controller_command_srv_ = this->create_client<CommandFeedback>("controller" + std::to_string(controller_node_id_) + "_command");
    /* Sub1.创建订阅者 */
    servo_status_subscription_ = this->create_subscription<MotorStatus>(
        "/servo_motor_status", 10,
        [this](const MotorStatus::SharedPtr msg)
        {
          if (msg->node_id == node_id_) // 仅显示对应节点的状态信息
            label_->setText(get_qstr_from_msg(msg));
        });
    /* Qt1.创建一个空的 SystemStatus 对象，转化成 QString 进行显示 */
    label_ = new QLabel(get_qstr_from_msg(std::make_shared<MotorStatus>()));
    /* Qt2.创建按钮和文本标签 */
    send_button_speed_mode_ = new QPushButton("发送速度指令");
    speed_line_edit_ = new QLineEdit();
    send_button_position_mode_ = new QPushButton("发送位置指令");
    position_line_edit_ = new QLineEdit();
    send_button_change_mode_ = new QPushButton("发送模式指令");
    change_mode_line_edit_ = new QLineEdit();
    send_button_reset = new QPushButton("发送清零/回零点指令");
    /* Qt3.创建布局 */
    v_layout_ = new QVBoxLayout();

    // 水平布局第一个竖直布局
    h_layout_ = new QHBoxLayout();
    form_layout_ = new QFormLayout();
    form_layout_->addRow("速度指令:", speed_line_edit_);
    form_layout_->addRow("位置指令:", position_line_edit_);
    form_layout_->addRow("修改模式指令:(0x01:速度模式 0x02:位置模式 0x03:测试模式)", change_mode_line_edit_);
    h_layout_->addLayout(form_layout_);
    // 水平布局第二个竖直布局
    form_layout_ = new QFormLayout();
    form_layout_->addWidget(send_button_speed_mode_);
    form_layout_->addWidget(send_button_position_mode_);
    form_layout_->addWidget(send_button_change_mode_);
    form_layout_->addWidget(send_button_reset);
    h_layout_->addLayout(form_layout_);

    v_layout_->addWidget(label_);
    v_layout_->addLayout(h_layout_);
    /* Qt4.设置布局 */
    widget_ = new QWidget();
    widget_->setLayout(v_layout_);
    widget_->show();
    /* Qt5.连接按钮的点击信号和槽函数 */
    connect(send_button_speed_mode_, &QPushButton::clicked, this, &MotorStatusDisplay::send_speed_command);
    connect(send_button_position_mode_, &QPushButton::clicked, this, &MotorStatusDisplay::send_position_command);
    connect(send_button_change_mode_, &QPushButton::clicked, this, &MotorStatusDisplay::send_change_command);
    connect(send_button_reset, &QPushButton::clicked, this, &MotorStatusDisplay::send_reset_command);
  }

  /* @brief 转换 MotorStatus 消息到 QString 类型
   * @param msg 要转换的 MotorStatus 消息
   * @return 转换后的 QString 类型数据
   */
  QString get_qstr_from_msg(const MotorStatus::SharedPtr msg)
  {
    std::stringstream show_str;
    show_str
        << "===========motor" << static_cast<int>(msg->node_id) << "状态可视化显示工具============\n"
        << "node_id:\t" << static_cast<int>(msg->node_id) << "\t\n"
        << "电机模式(1:速度 2:位置 3:测试模式):\t" << static_cast<int>(msg->motor_mode) << "\t\n"
        << "目标速度:\t" << msg->motor_speed_set << "\t\n"
        << "目标相对位置:\t" << static_cast<int>(msg->motor_position_set) << "\t\n"
        << "实际速度:\t" << static_cast<int16_t>(msg->motor_speed) << "\trpm\n"
        << "实际位置:\t" << static_cast<int>(msg->motor_position_real) << "\t\n"
        << "相对位置:\t" << static_cast<int>(msg->motor_position_relative) << "\t\n"
        << "零点位置:\t" << msg->motor_zero_point << "\t\n"
        << "位置模式运行标志位\t" << msg->motor_position_running_flag << "\t\n"
        << "============================================================";

    return QString::fromStdString(show_str.str());
  }

  /* @brief 发送速度指令的槽函数
   * @note 发送速度指令的按钮点击信号连接到此槽函数
   */
  void send_speed_command()
  {
    bool ok;
    uint32_t speed = speed_line_edit_->text().toUInt(&ok); // 获取uint32类型的数据
    if (ok)                                                // 如果转换成功
    {
      /* Cli2.构造请求*/
      auto request = std::make_shared<CommandFeedback::Request>();
      request->command_num = SET_SPEED;
      request->data[0] = (uint8_t)(speed >> 0);
      request->data[1] = (uint8_t)(speed >> 8);
      request->data[2] = (uint8_t)(speed >> 16);
      request->data[3] = (uint8_t)(speed >> 24);
      /* Cli3.发送请求 */
      servo_command_cli_->async_send_request(request);
    }
    else // 如果转换失败
    {
      QMessageBox::warning(nullptr, "警告", "请输入有效的速度值！");
    }
  }

  /* @brief 发送位置指令的槽函数
   * @note 发送位置指令的按钮点击信号连接到此槽函数
   */
  void send_position_command()
  {
    bool ok;
    int32_t position = position_line_edit_->text().toInt(&ok); // 获取int32类型的数据
    if (ok)                                                    // 如果转换成功
    {
      /* Cli2.构造请求 */
      auto request = std::make_shared<CommandFeedback::Request>();
      request->command_num = SET_POS; // 假设位置指令的命令编号为0xf3
      request->data[0] = (uint8_t)(position >> 0);
      request->data[1] = (uint8_t)(position >> 8);
      request->data[2] = (uint8_t)(position >> 16);
      request->data[3] = (uint8_t)(position >> 24);
      /* Cli3.发送请求 */
      servo_command_cli_->async_send_request(request);
    }
    else // 如果转换失败
    {
      QMessageBox::warning(nullptr, "警告", "请输入有效的速度值！");
    }
  }

  /* @brief 发送修改模式指令的槽函数
   * @note 发送修改模式指令的按钮点击信号连接到此槽函数
   */
  void send_change_command()
  {
    bool ok;
    uint8_t mode = change_mode_line_edit_->text().toUInt(&ok); // 获取uint8类型的数据
    if (ok && (mode == 1 || mode == 2 || mode == 3))           // 如果转换成功
    {
      /* Cli2.构造请求 */
      auto request = std::make_shared<CommandFeedback::Request>();
      request->command_num = SET_MODE; // 假设修改模式指令的命令编号为0xf4
      request->data[0] = mode;
      /* Cli3.发送请求 */
      servo_command_cli_->async_send_request(request);
    }
    else // 如果转换失败
    {
      QMessageBox::warning(nullptr, "警告", "请输入有效的模式值！");
    }
  }
  /**
   * @brief 发送控制器清零/电机回零点的槽函数
   * @note 发送控制器清零/电机回零点的按钮点击信号连接到此槽函数
   */
  void send_reset_command()
  {
    /* Cli2.构造请求 */
    auto request = std::make_shared<CommandFeedback::Request>();
    request->command_num = SET_ZERO; // 假设修改模式指令的命令编号为0xf5
    /* Cli3.发送请求 */
    controller_command_srv_->async_send_request(request);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<MotorStatusDisplay>();
  std::thread spin_thread([&]()
                          { rclcpp::spin(node); });
  spin_thread.detach(); // node线程分离
  app.exec();
  rclcpp::shutdown();
  return 0;
}