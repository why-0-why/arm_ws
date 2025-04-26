#include "can_tool/can_tool.hpp" // 包含头文件

#define __TEST__ // 定义宏，用于测试代码

int main()
{
    /*模板*/
    CanTool can("vcan0", 1000000);                                         // 创建 CAN 工具对象
    uint8_t command[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00}; // 定义测试命令
    // 创建接收数据线程
    uint32_t recv_id;
    uint8_t recv_len;
    uint8_t recv_data;
    // 接收线程超时时间设为 1000 ms
    std::thread recv_Thread([&]() -> void
                            {
                                    while(1)
                                    {
                                        can.recv(&recv_id, &recv_len, &recv_data);
                                    } });
    usleep(150000);              // 暂停 150 毫秒
    can.send(0x601, 8, command); // 发送测试消息
    return 0;
}
