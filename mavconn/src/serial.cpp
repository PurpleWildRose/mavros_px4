#include <serial.h>
#include <cassert>
#include <thread_utils.h>
#include <console_printf.h>

#if defined(__linux__)
#include <linux/serial.h>
#endif

namespace mavconn{

/**
 * Boost 库提供的错误处理类(如: 串口操作错误（如 /dev/ttyACM0 设备不存在、权限不足）/网络通信错误（如 UDP/TCP 连接失败、发送数据时连接断开/异步操作错误（如 Boost.Asio 的异步读取 / 写入失败）)
 * @example
 * boost::system::error_code ec;
 * serial_port.open("/dev/ttyACM0", ec);
 * if (ec)
 *      std::cerr << "串口打开失败" << ec.message() << std::endl;
 */
using boost::system::error_code;

/**
 * Boost.Asio 库的核心调度器，Boost 1.66+ 后推荐用 io_context，但老代码仍常用 io_service。
 * 使用场景：
 *      1. 管理串口的异步读写（避免阻塞主线程）
 *      2. 管理 UDP/TCP 网络通信的异步收发
 *      3. 统一调度多个通信链路（如同时处理串口和 UDP 连接）
 * @example
 * io_service io;
 * boost::asio::serial_port serial(io); // 串口对象绑定到 IO 服务
 * std::thread io_thread([&io]() {
 *      io.run();
 * });
 */
using boost::asio::io_service;
// Boost.Asio 提供的缓冲区包装工具
using boost::asio::buffer;
using mavlink::mavlink_message_t;

#define PFX "mavconn: serial"
#define PFXd PFX "%zu"

MAVConnSerial::MAVConnSerial(uint8_t system_id, uint8_t component_id, std::string device,
                                unsigned baudrate, bool hwflow):
        MAVConnInterface(system_id, component_id),  // 初始化基类
        io_service(),
        serial_dev(io_service),
        tx_in_progress(false),
        tx_q{},
        rx_buf{} {
    using SPB = boost::asio::serial_port_base;

    CONSOLE_BRIDGE_logInform(PFXd "device: %s @ %d bps", conn_id, device.c_str(), baudrate);

    try {
        serial_dev.open(device);


    }
    catch (boost::system::system_error &e) {
        throw DeviceError("serial", e);
    }

}

}
