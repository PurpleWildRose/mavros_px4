#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <interface.h>
#include <msgbuffer.h>

namespace mavconn{

// 允许类实例通过 shared_from_this() 方法，安全地获取指向自身的 std::shared_ptr，避免智能指针管理混乱（如重复析构、悬垂指针）。
class MAVConnSerial: public MAVConnInterface, public std::enable_shared_from_this<MAVConnSerial> {
    public:
        static constexpr auto DEFAULT_DEVICE = "/dev/ttyACM0";
        static constexpr auto DEFAULT_BAUDRATE = 115200;

        // 通过硬件信号协调串口收发双方的数据流，避免因一方发送速度过快、另一方处理不及时导致的数据丢失（缓冲区溢出）。
        MAVConnSerial(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
                        std::string device = DEFAULT_DEVICE, unsigned baudrate = DEFAULT_BAUDRATE, bool hwflow = false);
        virtual ~MAVConnSerial();

        // 重写基类 MAVConnInterface 的纯虚函数
        void connect(const ReceivedCb &cb_handle_message,
                        const ClosedCb &cb_handle_closed_port = ClosedCb()) override;

        void close() override;

        void send_message(const mavlink::mavlink_message_t *message) override;
        void send_message(const mavlink::Message &message, const uint8_t source_compid) override;
        void send_bytes(const uint8_t *bytes, size_t length) override;

        inline bool is_open() override {
            return serial_dev.is_open();
        }

    private:
        // Boost.Asio 的 I/O 服务，用于异步操作
        boost::asio::io_service io_service;
        // 运行 io_service 的线程（处理异步事件）
        std::thread io_thread;

        // 串口设备对象，封装串口操作。
        boost::asio::serial_port serial_dev;

        // 标记是否正在发送数据（避免并发发送冲突）。
        std::atomic<bool> tx_in_progress;
        // 发送队列（存储待发送的消息缓冲区）。
        std::deque<MsgBuffer> tx_q;
        // 接收缓冲区（存储从串口读取的原始字节）。
        std::array<uint8_t, MsgBuffer::MAX_SIZE> rx_buf;
        // 递归互斥锁，用于保护发送队列等共享资源。
        std::recursive_mutex mutex;

        void do_read();

        void do_write(bool check_tx_state);

};

}
