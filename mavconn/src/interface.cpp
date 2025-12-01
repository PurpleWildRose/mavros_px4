#include <interface.h>
#include <console_printf.h>

namespace mavconn{

#define PFX "mavconn: "

using mavlink::mavlink_message_t;
using mavlink::mavlink_status_t;

std::once_flag MAVConnInterface::init_flag;
std::unordered_map<mavlink::msgid_t, const mavlink::mavlink_msg_entry_t*> MAVConnInterface::message_entries;
std::atomic<size_t> MAVConnInterface::conn_id_counter {0};

MAVConnInterface::MAVConnInterface(uint8_t system_id, uint8_t component_id):
    sys_id(system_id),
    comp_id(component_id),
    m_parse_status{},
    m_buffer{},
    m_mavlink_status{},
    last_rx_total_bytes(0),
    last_tx_total_bytes(0),
    last_iostat(stead_clock::now()) {
    // 原子类型的成员函数，用于原子性地将指定值加到原子变量中，并返回操作前的旧值，是实现无锁自增 / 累加操作的核心接口，确保多线程环境下的操作原子性，避免数据竞争。
    conn_id = conn_id_counter.fetch_add(1);
    std::call_once(init_flag, init_msg_entry);
}

/**
 * get_status
 * @brief 获取当前mavlink获取的状态，主要是看其是否满足成为一个完整的mavlink消息
 */
mavlink_status_t MAVConnInterface::get_status() {
    return m_parse_status;
}

/**
 * get_iostat
 * @param 通过 get_iostat() 返回 IOStat 结构体，包含：
 *      累计收发字节数（tx_total_bytes/rx_total_bytes）；
 *      实时收发速率（tx_speed/rx_speed，单位 B/s，通过周期性计算字节差实现）。
 */
MAVConnInterface::IOStat MAVConnInterface::get_iostat() {
    std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
    IOStat stat;

    stat.tx_total_bytes = tx_total_bytes;
    stat.rx_total_bytes = rx_total_bytes;
    auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
    auto d_rx = stat.rx_total_bytes = last_rx_total_bytes;

    last_tx_total_bytes = stat.tx_total_bytes;
    last_rx_total_bytes = stat.rx_total_bytes;

    auto now = stead_clock::now();
    auto dt = now - last_iostat;
    last_iostat = now;

    float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

    stat.tx_speed = d_tx / dt_s;
    stat.rx_speed = d_rx / dt_s;

    return stat;
}

/**
 * iostat_tx_add
 * @brief 将通过端口发送出的字节数添加到内部变量，方便统计端口传输数据
 */
void MAVConnInterface::iostat_tx_add(size_t bytes) {
    tx_total_bytes += bytes;
}

/**
 * iostat_rx_add
 * @brief 将通过端口接收的字节数添加到内部变量，方便统计端口传输数据
 */
void MAVConnInterface::iostat_rx_add(size_t bytes) {
    rx_total_bytes += bytes;
}

/**
 * parse_buffer
 * @brief 解析消息
 *
 * @param pfx   用户自定义消息
 * @param buf   存储解析的数组
 * @param bufsize   存储解析数组长度
 * @param bytes_received    准备解析的字节数
 */
void MAVConnInterface::parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received) {
    mavlink_message_t msg;

    // 避免溢出
    assert(bufsize >= bytes_received);

    iostat_rx_add(bytes_received);

    for (; bytes_received > 0; bytes_received--) {
        auto c = *buf++;

        auto msg_received = static_cast<Framing>(mavlink::mavlink_frame_char_buffer(&m_buffer, &m_parse_status, c, &msg, &m_mavlink_status));

        // 正常情况下，除了incomplete之外的（ok，bad crc， bad_sign)
        if (msg_received != Framing::incomplete) {
            log_recv(pfx, msg, msg_received);

            // 实现 “可选回调” 机制 —— 仅当回调函数被绑定（非空）时，才触发消息处理逻辑，避免调用空回调导致异常。
            if (message_received_cb) {
                message_received_cb(&msg, msg_received);
            }
        }
    }
}

/**
 * log_recv
 * @brief 主要用于将接受解析消息打印出来，编译诊断mavlink在传输时的错误。
 *
 * @param pfx   用户自定义消息头
 * @param msg   解析的完整消息，可能完整/错误CRC校验/错误签名
 */
void MAVConnInterface::log_recv(const char *pfx, mavlink_message_t &msg, Framing frame) {
    const char *framing_str = (frame == Framing::ok) ? "OK" :
                                (frame == Framing::bad_crc) ? "!CRC" :
                                (frame == Framing::bad_signature) ? "!SIG" : "ERR";
    const char *proto_version_str = (msg.magic == MAVLINK_STX) ? "V2" : "V1";

    CONSOLE_BRIDGE_logDebug("%s%zu: recv: %s %4s Message-Id: %u [%u bytes] IDs: %u.%u seq: %u",
                pfx, conn_id,
                proto_version_str,
                framing_str,
                msg.msgid, msg.len, msg.sysid, msg.compid, msg.seq);
}

/**
 * log_send
 * @brief 主要是将发送的消息打印出来，编译诊断mavlink在传输时的错误。
 *
 * @param pfx   用户自定消息
 * @param msg   编码准备发送的消息(mavlink::mavlink_message_t)
 */
void MAVConnInterface::log_send(const char *pfx, mavlink::mavlink_message_t *msg) {
    const char *proto_version_str = (msg->magic == MAVLINK_STX) ? "V2" : "V1";

    CONSOLE_BRIDGE_logDebug("%s%zu: send: %s Message-Id: %u [%u bytes] IDs: %u.%u seq: %u",
                pfx, conn_id,
                proto_version_str,
                msg->msgid, msg->len, msg->sysid, msg->compid, msg->seq);
}

/**
 * log_send_obj
 * @brief 主要是将发送的消息打印出来，编译诊断mavlink在传输时的错误。
 *
 * @param pfx   用户自定义消息
 * @param msg   编码准备发送的消息(mavlink::Msssage)
 */
void MAVConnInterface::log_send_obj(const char *pfx, const mavlink::Message &msg) {
    CONSOLE_BRIDGE_logDebug("%s%zu: send: %s",
                pfx, conn_id,
                msg.to_yaml().c_str());
}

/**
 * send_message_ignore_drop
 * @brief 发送一条mavlink消息，带缓存。
 *
 * @param message   mavlink::mavlink_message_t 打包后的消息
 */
void MAVConnInterface::send_message_ignore_drop(const mavlink::mavlink_message_t *message) {
    try {
        send_message(message);
    }
    catch (std::length_error &e) {
        CONSOLE_BRIDGE_logError(PFX "%zu DROPPEND Message-Id %u [%u bytes] IDs: %u.%u seq: %u: %s",
                conn_id,
                message->msgid, message->len, message->sysid, message->compid, message->seq,
                e.what());
    }
}

/**
 * send_message_ignore_drop
 * @brief 发送一条mavlink消息，带缓存。
 *
 * @param msg       mavlink::Mavlink 打包后的消息
 * @param source_id 来自的组件的消息（组件id）
 */
void MAVConnInterface::send_message_ignore_drop(const mavlink::Message &msg, uint8_t source_id) {
    try {
        send_message(msg, source_id);
    }
    catch (std::length_error &e) {
        CONSOLE_BRIDGE_logError(PFX "%zu: DROPPED Message %s: %s",
                conn_id,
                msg.get_name().c_str(),
                e.what());
    }
}

/**
 * set_protocol_version
 * @brief 设置mavlink解析的版本信息
 *
 * @param prot  mavlink版本信息(v1/v2)
 */
void MAVConnInterface::set_protocol_version(Protocol prot) {
    if (prot == Protocol::V1) {
        m_mavlink_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        m_parse_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    } else {
        m_mavlink_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
        m_parse_status.flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    }
}

/**
 * get_protocol_version
 * @brief 返回当前mavlink的版本信息
 */
Protocol MAVConnInterface::get_protocol_version() {
    if (m_mavlink_status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
        return Protocol::V1;
    else
        return Protocol::V2;
}

/**
 * url_parse_host
 * @brief 主要功能是从输入的字符串中提取主机名（或 IP 地址）和端口号，并在解析失败或格式不完整时使用默认值。
 *
 * @param host      用户输入的字符串，此字符串最好包含完整信息，便于分割（如/dev/ttyACM0:115200）
 * @param host_out  分割后的host口
 * @param port_out  分割后的port口
 * @param def_host  预定义的host口
 * @param def_port  预定义的port口
 */
static void url_parse_host(std::string host, std::string &host_out, int &port_out,
                            const std::string def_host, const int def_port) {
    std::string port;

    auto sep_it = std::find(host.begin(), host.end(), ":");
    // 无法在host中找到分隔符
    if (sep_it == host.end()) {
        if (!host.empty()) {
            host_out = host;
            port_out = def_port;
        }
        else {
            host_out = def_host;
            port_out = def_port;
        }

        return;
    }
    if (sep_it == host.begin()) {
        host_out =def_host;
    }
    else {
        host_out.assign(host.begin(), sep_it);
    }

    port.assign(sep_it + 1, host.end());
    port_out = std::stoi(port);
}

/**
 * url_parse_query
 * @brief 解析 URL 查询参数的工具函数，主要功能是从查询字符串中提取 sysid（系统 ID）和 compid（组件 ID），这两个参数在 MAVLink 协议中用于标识消息的发送方或接收方。
 * @param query     需要查询的字符串（如XXXidx=1,2)
 * @param sysid     查询到的sysid
 * @param compid    查询到的compid
 */
static void url_parse_query(std::string query, uint8_t &sysid, uint8_t &compid) {
    const std::string ids_end("ids=");
    std::string sys, comp;

    if (query.empty()) {
        return;
    }

    // 使用 std::search 查找 query 字符串中是否包含 "ids="。std::search 返回 ids= 子字符串的起始迭代器（ids_it）。
    auto ids_it = std::search(query.begin(), query.end(), ids_end.begin(), ids_end.end());
    if (ids_it == query.end()) {
        CONSOLE_BRIDGE_logWarn(PFX "URL: unknown query arguments.");
        return;
    }
    // 通过 std::advance 将迭代器 ids_it 向前移动 "ids=" 字符串的长度，跳过 "ids=" 部分，指向系统 ID 的开始位置。
    std::advance(ids_it, ids_end.length());
    auto comma_it = std::find(ids_it, query.end(), ",");
    if (comma_it == query.end()) {
        CONSOLE_BRIDGE_logError(PFX "URL: no comma in ids = query.");
        return;
    }

    //提取sys_id 与 comp_id
    sys.assign(ids_it, comma_it);
    comp.assign(comma_it + 1, query.end());

    sysid = std::stoi(sys);
    compid = std::stoi(comp);

    CONSOLE_BRIDGE_logDebug(PFX "URL: found system/component id = [%u, %u]", sysid, compid);
}

/**
 * url_parse_serial
 * @brief 解析一个串口连接的 URL 路径和查询参数，并返回一个封装了 MAVConnSerial 实例的智能指针（std::shared_ptr<MAVConnInterface>）。
 */
static MAVConnInterface::Ptr url_parse_serial(std::string path, std::string query,
                uint8_t system_id, uint8_t component_id, bool hwflow) {
    std::string dev_path;
    int baudrate;

    url_parse_host(path, dev_path, baudrate, );
    url_parse_query(query, system_id, component_id);

    // return std::make_shared<>(system_id, component_id)
}




} // namespace mavconn
