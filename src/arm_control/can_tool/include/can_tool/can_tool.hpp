#ifndef CAN_TOOL_HPP // 防止重复包含
#define CAN_TOOL_HPP

#include <iostream> // 包含输入输出流库，用于输出信息；用于在控制台输出调试信息
/*can操作库*/
#include <net/if.h>     // 提供网络接口的定义
#include <sys/ioctl.h>  // 提供 ioctl 函数的定义
#include <sys/socket.h> // 提供 socket 函数的定义
#include <linux/can.h>  // 定义 CAN 协议相关的结构和常量
#include <unistd.h>     // 提供对 POSIX 操作系统 API 的访问，套接字文件读写
/* 数据操作库 */
#include <cstdint> // 包含整型定义，如 uint32_t
#include <cstring> // 提供字符串操作的函数
#include <cstdlib> // 提供 malloc、realloc、free 等内存管理函数
#include <regex>   // 提供正则表达式库
/* 线程库 */
#include <thread> // 提供线程相关的函数

class CanTool
{
private:
    /* socketcan参数及变量*/
    int socketcan_;            // CAN 套接字
    struct ifreq ifr_;         // 网络接口请求结构
    struct sockaddr_can addr_; // 套接字地址结构
    int error_;                // 错误

public:
    /* @brief: can0.linux系统初始化can口
     * @param std::string can_name: can口名称
     *        uint32_t bitrate: can口波特率
     * @return: void
     */
    void can_init(std::string can_name, uint32_t bitrate)
    {
        // 参数校验
        if (!std::regex_match(can_name, std::regex("^can\\d+")))
        {
            throw std::invalid_argument("Invalid CAN interface name");
        }

        if (bitrate < 5000 || bitrate > 1000000)
        {
            throw std::range_error("Invalid bitrate value");
        }
        try
        {
            std::system("sudo modprobe peak_usb");
            // 加载can驱动(依据设备情况)
            std::system(("sudo ip link set " + can_name + " up type can bitrate " + std::to_string(bitrate)).c_str());
            std::system(("sudo ip link set " + can_name + " txqueuelen 100000").c_str());
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return;
        }
    }

    /* @brief: can1.socketcan初始化
     * @param std::string can_name: can口名称
     * @return: void
     */
    void socketcan_init(std::string can_name)
    {
        /* can1.1创建 CAN 套接字 */
        socketcan_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socketcan_ < 0)
        {
            perror("无法创建 CAN 套接字"); // 输出错误信息，使用 perror 获取系统错误信息
            return;                        // 返回
        }

        /* can1.2配置要使用的 CAN 接口名称 */
        strcpy(ifr_.ifr_name, can_name.c_str());        // 将接口名复制到接口请求结构中
        if (ioctl(socketcan_, SIOCGIFINDEX, &ifr_) < 0) // 获取接口的索引
        {
            perror("无法获取接口索引"); // 输出错误信息
            return;                     // 返回
        }

        /* can1.3绑定套接字到指定的 CAN 接口 */
        addr_.can_family = AF_CAN;                                          // 设置协议族为 CAN
        addr_.can_ifindex = ifr_.ifr_ifindex;                               // 绑定的接口索引
        if (bind(socketcan_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) // 执行绑定操作
        {
            perror("无法绑定到接口"); // 输出错误信息
            return;                   // 返回
        }
    }

    /* @brief: 构造函数,创建与can_name对应的对象
     * @param std::string can_name: can口名称
     * @return: void
     */
    CanTool(std::string can_name, uint32_t bitrate)
    {
        can_init(can_name, bitrate);
        socketcan_init(can_name);
    }

    /* @brief: 析构函数,释放资源
     * @return: void
     */
    ~CanTool()
    {
        // 关闭套接字
        if (close(socketcan_) < 0)
        {
            perror("关闭套接字失败"); // 添加关闭套接字的错误处理
        }
    }

    /* @brief: can2.发送can数据帧
     * @param uint32_t id: can标识符
     *        uint8_t len: 数据长度
     *        uint8_t *data: 数据内容
     * @return: bool
     */
    bool send(uint32_t id, uint8_t len, uint8_t *data)
    {
        /* can2.1组织can数据帧*/
        struct can_frame frame;        // 定义 CAN 帧结构
        frame.can_id = id;             // 设置 CAN 标识符
        frame.can_dlc = len;           // 设置数据长度
        memcpy(frame.data, data, len); // 设置数据
#ifdef __TEST__                        // 输出发送的数据
        std::cout << "发送数据内容: ";
        for (size_t i = 0; i < len; ++i)
        {
            std::cout << std::hex << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::endl;
#endif
        /* can2.2发送can数据帧 */
        if (write(socketcan_, &frame, sizeof(frame)) < 0) // 写入 CAN 消息到套接字
        {
            perror("发送消息失败"); // 输出错误信息
            return false;           // 返回失败
        }
        else
            return true; // 返回成功
    }

    bool recv(uint32_t *id, uint8_t *len, uint8_t *data)
    {
        /* can3.1 接收 CAN 消息 */
        struct can_frame frame; // 定义 CAN 帧结构
        socklen_t len_t = sizeof(frame);
        if (read(socketcan_, &frame, len_t) < 0) // 读取 CAN 消息到套接字
        {
            if (errno == EAGAIN) // 如果没有收到消息，则返回失败
            {
                return false;
            }
            else // 其他错误，则输出错误信息
            {
                perror("接收消息失败");
                return false;
            }
        }

        /* can3.2返回CAN 消息 */
        *id = frame.can_id;             // 获取 CAN 标识符
        *len = frame.can_dlc;           // 获取数据长度
        memcpy(data, frame.data, *len); // 获取数据
                                        /* can3.3输出接收的数据 */
#ifdef __TEST__
        std::cout << "接收数据内容: ";
        std::cout << std::hex << *id << " ";
        for (size_t i = 0; i < *len; ++i)
        {
            std::cout << std::hex << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::endl;
#endif
        return true; // 返回成功
    }
};

#endif // CAN_TOOL_HPP
