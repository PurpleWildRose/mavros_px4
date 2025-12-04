#include <tcp.h>
#include <console_printf.h>
#include <thread_utils.h>

#if BOOST_VERSION >= 107000
#define GET_IO_SERVICE(s) ((boost::asio::io_context&)(s).get_executor().context())
#else
#define GET_IO_SERVICE(s) ((s).get_io_service())
#endif

namespace mavconn{
    using boost::system::error_code;

    #define PFX "mavconn: tcp"
    #define PFXd PFX "%zu"

    /**
     * resolve_address_tcp
     * @brief  TCP 地址解析工具，核心作用是将域名 / IP 地址（host）和端口（port）解析为可直接使用的 tcp::endpoint 对象
     *
     * @param io Boost.Asio 的 IO 服务对象（驱动解析操作）
     * @param chan 通道编号（日志中用于标识来源，如 MAVLink 通道）
     * @param host 待解析的主机地址（IP / 域名）
     * @param ep 输出参数，存储解析后的最终端点（IP + 端口）
     */
    static bool resolve_address_tcp(boost::asio::io_service &io, size_t chan, std::string host,
                                    unsigned short port, boost::asio::ip::tcp::endpoint &ep) {
        bool result = false;
        // 创建 TCP 地址解析器(必须关联 io_service)
        // 负责将主机名 / 域名 + 端口解析为可直接使用的 tcp::endpoint（TCP 端点）的工具
        boost::asio::ip::tcp::resolver resolver(io);
        boost::system::error_code ec;

        // 构建解析查询对象
        // 这里第二个参数传空字符串 ""
        //      · 函数后续会手动给 endpoint 设置 port，无需 resolver 解析端口；
        //      · 若传端口字符串（如 "5760"），resolver 也会解析，但函数设计上希望端口由外部指定，更灵活。
        boost::asio::ip::tcp::resolver::query query(host, "");

        auto fn = [&](const boost::asio::ip::tcp::endpoint &q_ep) {
            ep = q_ep;
            // 修改端口号为port
            ep.port(port);
            result = true;
            CONSOLE_BRIDGE_logDebug(PFXd "host %s resolved as %s", chan, host.c_str(), utils::to_string_ss(ep).c_str());
        };

        #if BOOST_ASIO_VERSION >= 101200
            // resolve返回的是一个迭代器
            for (auto q_ep : resolver.resolve(query, ec)) fn(q_ep);
        #else
            std::for_each(resolver.resolve(query, ec), tcp::resolver::iterator(), fn);
        #endif

        if (ec) {
            CONSOLE_BRIDGE_logError(PFXd "resolve error: %s", chan, ec.message().c_str());
            result = false;
        }

        return result;
    }

    MAVConnTCPClient::MAVConnTCPClient(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
                    std::string server_host = DEFAULT_SERVER_HOST, unsigned short server_port = DEFAULT_SERVER_PORT):
            MAVConnInterface(system_id, component_id),
            io_service(),
            io_work(new boost::asio::io_service::work(io_service)),
            socket(io_service),
            is_destroying(false),
            tx_in_progress(false),
            tx_q{},
            rx_buf{} {
        if (!resolve_address_tcp(io_service, conn_id, server_host, server_port, server_ep))
                throw DeviceError("tcp: resolve", "Bind address resolve failed!");

        CONSOLE_BRIDGE_logInform(PFXd "Server address: %s", conn_id, utils::to_string_ss(server_ep).c_str());

        try {
            socket.open(boost::asio::ip::tcp::v4());
            socket.connect(server_ep);
        }
        catch (boost::system::system_error &e) {
            throw DeviceError("tcp", e);
        }
    }

    MAVConnTCPClient::MAVConnTCPClient(uint8_t system_id, uint8_t component_id, boost::asio::io_service &server_io):
            MAVConnInterface(system_id, component_id),
            socket(server_io),
            is_destroying(false),
            tx_in_progress(false),
            tx_q{},
            rx_buf{} {
        // 等待server端回调client_connected
    }

    /**
     * client_connected
     * @brief boost将do_recv添加到
     */
    void MAVConnTCPClient::client_connected(size_t channel) {
        CONSOLE_BRIDGE_logInform(PFXd "Got client, id %zu, address %s", channel, conn_id, utils::to_string_ss(server_ep).c_str());

        GET_IO_SERVICE(socket).post(std::bind(&MAVConnTCPClient::do_recv, this));
    }

    /**
     *
     */
    void MAVConnTCPClient::connect(const ReceivedCb &cb_handle_message,
                        const ClosedCb &cb_handle_closed_port = ClosedCb()) {
        message_received_cb = cb_handle_message;
        port_close_cb = cb_handle_closed_port;

        GET_IO_SERVICE(socket).post(std::bind(&MAVConnTCPClient::do_recv(), this));
    }


} //name mavconn
