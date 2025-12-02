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

        // Set baudrate and 8N1 mod
		// 8：表示数据位长度为 8 位（每个字符由 8 个二进制位组成）。
		// N：表示无校验位（No parity），即不使用奇偶校验来检测数据传输错误。
		// 1：表示停止位长度为 1 位（用于标记一个数据帧的结束）。
		// 总计每个字符需要传输 10 位（1 起始位 + 8 数据位 + 1 停止位）。
        serial_dev.set_option(SPB::baud_rate(baudrate));
		serial_dev.set_option(SPB::character_size(8));
		serial_dev.set_option(SPB::parity(SPB::parity::none));
		serial_dev.set_option(SPB::stop_bits(SPB::stop_bits::one));

        // 处理硬件流控制（根据 hwflow 参数配置）
        #if BOOST_ASIO_VERSION >= 101200 || !defined(__linux__)
            serial_dev.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
        #elif BOOST_ASIO_VERSION < 101200 && defined(__linux__)
            {
			int fd = serial_dev.native_handle();

			termios tio;
			tcgetattr(fd, &tio);

			// Set hardware flow control settings
			if (hwflow) {
				tio.c_iflag &= ~(IXOFF | IXON);
				tio.c_cflag |= CRTSCTS;
			} else {
				tio.c_iflag &= ~(IXOFF | IXON);
				tio.c_cflag &= ~CRTSCTS;
			}

			// Set serial port to "raw" mode to prevent EOF exit.
			cfmakeraw(&tio);

			// Commit settings
			tcsetattr(fd, TCSANOW, &tio);
		}
        #endif

        // 在 Linux 系统上启用低延迟模式（Low Latency）主要用于减少系统响应时间，适用于实时音频处理、嵌入式控制、游戏服务器等对延迟敏感的场景。
        #if defined(__linux__)
            // Enable low latency mode on Linux
            {
                int fd = serial_dev.native_handle();

                struct serial_struct ser_info;
                ioctl(fd, TIOCGSERIAL, &ser_info);

                ser_info.flags |= ASYNC_LOW_LATENCY;

                ioctl(fd, TIOCSSERIAL, &ser_info);
            }
        #endif
    }
    catch (boost::system::system_error &e) {
        throw DeviceError("serial", e);
    }
}

MAVConnSerial::~MAVConnSerial() {
    close();
}

/**
 * connect
 * @brief serial端口启用建立连接
 *
 * @param cb_handle_message     传输消息的回调函数
 * @param cb_handle_closed_port 端口关闭的回调函数
 */
void MAVConnSerial::connect(const ReceivedCb &cb_handle_message,
                        const ClosedCb &cb_handle_closed_port = ClosedCb()) {
    message_received_cb = cb_handle_message;
    port_close_cb = cb_handle_closed_port;

    // io_service.post 将一个任务添加到 io_service 中，任务是调用 MAVConnSerial::do_read 方法。
	// std::bind(&MAVConnSerial::do_read, this) 将当前对象（this）与 do_read 方法绑定，确保在任务执行时能正确调用 this->do_read()。
	io_service.post(std::bind(&MAVConnSerial::do_read, this));

    // 创建了一个新线程来运行 io_service.run()。该线程会启动异步 I/O 的执行。
	// io_service.run() 会启动 I/O 处理，并持续等待和处理任务，直到 io_service 停止。也就是说，这个线程会在后台一直运行，处理所有异步 I/O 请求。
	// utils::set_this_thread_name("mserial%zu", conn_id) 设置当前线程的名称为 "mserial0", "mserial1" 等，以便在调试时更容易识别不同的线程。conn_id 是一个连接标识符，通常用于区分不同的连接。
	io_thread = std::thread([this] () {
				utils::set_this_thread_name("mserial%zu", conn_id);
				io_service.run();
			});
}

/**
 * close
 * @brief serial端口关闭
 */
void MAVConnSerial::close() {
    // 1. 加锁：保证关闭操作的线程安全
    lock_guard lock(mutex);

    // 2. 检查是否已关闭：避免重复关闭（防止二次操作报错）
    if (!is_open())
        return;

    // 3. 取消串口上所有未完成的异步操作（关键！）
    serial_dev.cancel();
    // 4. 关闭串口设备（释放硬件资源）
    serial_dev.close();
    // 5. 停止IO调度器（终止异步操作循环）
    io_service.stop();

    // 6. 等待IO线程结束（避免线程泄漏）
    if (io_thread.joinable())
        io_thread.join();

    // 7. 重置IO调度器（可选：支持后续重新打开串口）
    io_service.reset();

    // 8. 触发端口关闭回调（通知上层代码）
    if (port_close_cb)
        port_close_cb();
}

/**
 * send_bytes
 * @brief 发送一组指定长度的字符
 *
 * @param bytes     要写入到字符
 * @param length    写入的字符长度
 */
void MAVConnSerial::send_bytes(const uint8_t *bytes, size_t length) {
    // 1. 校验串口状态：若未打开，直接报错返回
    if (!is_open()) {
        CONSOLE_BRIDGE_logError(PFXd "send: channel closed.", conn_id);
        return;
    }

    {
        // 2. 临界区：线程安全操作发送队列（加锁保护）
        lock_guard lock(mutex);

        // 3. 队列溢出保护：超过最大长度则抛出异常（避免内存泄漏）
        if (tx_q.size() >= MAX_TXQ_SIZE)
            throw std::length_error("MAVConnSerial::send_bytes: TX queue overflow!");

        // emplace_back 用于向 tx_q 队列的末尾添加一个新的元素。
        // 4. 将字节流加入发送队列（高效构造，无额外拷贝）
        tx_q.emplace_back(bytes, length);
    }

    // 5. 触发异步发送：通过 io_service 投递 do_write 任务（非阻塞）,调用一次send_bytes，便会将do_write投递到io_service的异步线程中。
    io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

/**
 * send_message
 * @brief 发送一条完整的mavlink消息  (mavlink::mavlink_message_t)
 *
 * @param message   特定功能的mavlink消息
 */
void MAVConnSerial::send_message(const mavlink::mavlink_message_t *message) {
    assert(message != nullptr);

    if (!is_open()){
        CONSOLE_BRIDGE_logError(PFXd "send: channel closed.", conn_id);
        return;
    }

    {
        lock_guard lock(mutex);

        if (tx_q.size() >= MAX_TXQ_SIZE)
            throw std::length_error("MAVConnSerial::send_message: TX queue overflow");
        tx_q.emplace_back(message);
    }

    io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

/**
 * send_message
 * @brief 发送一条完整的mavlink消息  (mavlink::Message)
 *
 * @param message       mavlink消息
 * @param source_compid 组件id
 */
void MAVConnSerial::send_message(const mavlink::Message &message, const uint8_t source_compid) {
    if (!is_open()) {
        CONSOLE_BRIDGE_logError(PFXd "send: channel closed", conn_id);
        return;
    }

    {
        lock_guard lock(mutex);

        if (tx_q.size() >= MAX_TXQ_SIZE)
            throw std::length_error("MAVConnSerial channel overflow");

        tx_q.emplace_back(message, get_status_p(), sys_id, source_compid);
    }

    io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

/**
 * do_read
 * @brief 实现了 “持续无阻塞读取 → 解析数据 → 循环读取” 的闭环逻辑，是 MAVLink 消息接收的底层支撑
 */
void MAVConnSerial::do_read() {
    // 1. 捕获自身共享指针：确保回调执行时，当前实例不被析构
    auto sthis = shared_from_this();

    // 2. 发起 Boost.Asio 异步读取（核心接口）
    serial_dev.async_read_some(
        buffer(rx_buf), // 3. 绑定接收缓冲区：读取到的数据存入 rx_buf
        // 4. 读取完成后的回调函数（lambda 表达式）
        [sthis](error_code error, size_t bytes_transferred) {
            // 5. 错误处理：读取失败（如串口断开、IO 错误）
            if (error) {
                CONSOLE_BRIDGE_logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
                sthis->close();
                return;
            }

            // 6. 读取成功：解析接收缓冲区的数据
            // rx_buf.data()：缓冲区起始地址；rx_buf.size()：缓冲区总大小；bytes_transferred：实际读取字节数
            sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);

            // 7. 递归调用 do_read()：发起下一次异步读取，实现“持续读取”
            sthis->do_read();
        }
    );
}

/**
 * do_write
 * @brief  异步发送队列数据 的核心实现，逻辑围绕 “有序发送、队列管理、错误处理” 三大核心，确保发送队列 tx_q 中的数据按入队顺序可靠发送，同时避免并发发送导致的消息乱序
 */
void MAVConnSerial::do_write(bool check_tx_state)
{
    // 1. 快速检查：若需要检查发送状态，且当前正在发送，则直接返回（避免重复发送）
    if (check_tx_state && tx_in_progress)
        return;

    // 2. 加锁：保护发送队列 tx_q 和状态 tx_in_progress（线程安全）
    lock_guard lock(mutex);
    // 3. 队列空则返回：无数据可发送
    if (tx_q.empty())
        return;

    // 4. 标记发送中：防止其他线程/任务重复发起发送
    tx_in_progress = true;
    // 5. 捕获自身共享指针：确保回调执行时实例不被析构
    auto sthis = shared_from_this();
    // 6. 取队列首元素：引用传递，避免拷贝（buf_ref 是待发送数据）
    auto &buf_ref = tx_q.front();

    // 7. 发起 Boost.Asio 异步发送（核心接口）
    serial_dev.async_write_some(
        buffer(buf_ref.dpos(), buf_ref.nbytes()), // 8. 绑定待发送数据（当前未发送的部分）
        // 9. 发送完成后的回调函数
        [sthis, &buf_ref] (error_code error, size_t bytes_transferred) {
            // 断言：发送的字节数不能超过当前待发送的字节数（调试用，避免逻辑错误）
            assert(bytes_transferred <= buf_ref.len);

            // 10. 错误处理：发送失败（如串口断开、IO 错误）
            if (error) {
                CONSOLE_BRIDGE_logError(PFXd "write: %s", sthis->conn_id, error.message().c_str());
                sthis->close(); // 关闭串口，清理资源
                return;
            }

            // 11. 更新发送统计（如累计发送字节数，用于监控）
            sthis->iostat_tx_add(bytes_transferred);

            // 12. 加锁：修改队列和发送状态（线程安全）
            lock_guard lock(sthis->mutex);

            // 13. 极端情况：队列已空（理论上不会发生，双重保险）
            if (sthis->tx_q.empty()) {
                sthis->tx_in_progress = false;
                return;
            }

            // 14. 更新当前数据的发送偏移：标记已发送的字节
            buf_ref.pos += bytes_transferred;
            // 15. 若当前数据已全部发送（未发送字节数为 0），从队列移除
            if (buf_ref.nbytes() == 0) {
                sthis->tx_q.pop_front();
            }

            // 16. 若队列还有数据，继续发送下一个（check_tx_state 设为 false，跳过状态检查）
            if (!sthis->tx_q.empty())
                sthis->do_write(false);
            else
                // 队列空，重置发送状态
                sthis->tx_in_progress = false;
        });
}

} //namespace mavconn
