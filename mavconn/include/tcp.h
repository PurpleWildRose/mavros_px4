#pragma once

#include <list>
#include <array>
#include <cstring>
#include <atomic>
#include <thread>
#include <boost/asio.hpp>
#include <msgbuffer.h>
#include <interface.h>

namespace mavconn {

class MAVConnTCPClient: public MAVConnInterface, public std::enable_shared_from_this<MAVConnTCPClient> {
    public:
        static constexpr auto DEFAULT_SERVER_HOST = "loaclhost";
        static constexpr auto DEFAULT_SERVER_PORT = 5760;

        MAVConnTCPClient(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
                    std::string server_host = DEFAULT_SERVER_HOST, unsigned short server_port = DEFAULT_SERVER_PORT);

        MAVConnTCPClient(uint8_t system_id, uint8_t component_id, boost::asio::io_service &server_io);
        ~MAVConnTCPClient();

         // 重写基类 MAVConnInterface 的纯虚函数
        void connect(const ReceivedCb &cb_handle_message,
                        const ClosedCb &cb_handle_closed_port = ClosedCb()) override;
        void close() override;

        void send_bytes(const uint8_t *bytes, size_t length) override;
        void send_message(const mavlink::mavlink_message_t *message) override;
        void send_message(const mavlink::Message &message, const uint8_t source_component) override;

        inline bool is_open() {
            return socket.is_open();
        }
    private:
        friend class MAVConnTCPServer;

        boost::asio::io_service io_service;
        // 当没有待处理的异步任务且无 work 对象关联时，run() 会立即返回;
        // 当创建 work 对象并关联到 io_service 后，run() 会一直阻塞，直到：
        //      1. 直到显式调用io_service.stop()强制退出
        //      2. 所有 work 对象被销毁 / 重置（优雅退出）
        std::unique_ptr<boost::asio::io_service::work> io_work;
        std::thread io_thread;

        //  Boost.Asio 对 TCP 套接字 的封装
        boost::asio::ip::tcp::socket socket;
        // Boost.Asio 对 TCP 通信端点 的封装，本质是 “IP 地址 + 端口号” 的组合
        boost::asio::ip::tcp::endpoint server_ep;

        std::atomic<bool> is_destroying;

        std::atomic<bool> tx_in_progress;
        std::deque<MsgBuffer> tx_q;
        std::array<uint8_t, MsgBuffer::MAX_SIZE> rx_buf;
        std::recursive_mutex mutex;

        void client_connected(size_t channel);

        void do_recv();
        void do_send();
};

class MAVConnTCPServer: public MAVConnInterface, public std::enable_shared_from_this<MAVConnTCPServer> {
    public:
        static constexpr auto DEFAULT_BIND_HOST = "localhost";
        static constexpr auto DEFAULT_BIND_PORT = 5760;

    public:
        MAVConnTCPServer(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
                            std::string bind_host = DEFAULT_BIND_HOST, unsigned short bind_port = DEFAULT_BIND_PORT);
        ~MAVConnTCPServer();

        void connect(const ReceivedCb &cb_handle_message, const ClosedCb &cb_handle_closed_port = ClosedCb()) override;
        void close() override;

        void send_bytes(const uint8_t *bytes, size_t length) override;
        void send_message(const mavlink::mavlink_message_t *message) override;
        void send_message(const mavlink::Message &message, uint8_t source_compid) override;

        mavlink::mavlink_status_t get_status() override;
        IOStat get_iostat() override;
        inline bool is_open() {
            return acceptor.is_open();
        }

    private:
        // Boost.Asio 的核心 IO 服务对象，是所有异步操作（如监听、读写）的 “事件循环”，负责调度和执行异步回调。
        boost::asio::io_service io_service;
        // TCP 服务器的 “监听器”，绑定到指定 IP 和端口，负责接收客户端的连接请求。
        boost::asio::ip::tcp::acceptor acceptor;
        // 定义服务器监听的 IP 地址和端口（如 127.0.0.1:5760）
        boost::asio::ip::tcp::endpoint bind_ep;

        // 核心作用是 “防止 io_service.run() 无任务时退出”。
        // std::unique_ptr 是一种独占式智能指针
        // boost::asio::io_service::work 核心作用是阻止 io_service（或 io_context）在没有待处理任务时自动停止运行，是控制异步 IO 服务生命周期的关键工具。
        std::unique_ptr<boost::asio::io_service::work> io_work;
        std::thread io_thread;

        std::atomic<bool> is_destroying;

        std::list<std::shared_ptr<MAVConnTCPClient>> client_list;
        std::recursive_mutex mutex;

        void do_acept();
        void client_close(std::weak_ptr<MAVConnTCPClient> weak_instp);
};

} //namespace mavconn
