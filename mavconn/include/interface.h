#pragma once

#include <boost/system/system_error.hpp>

#include <atomic>
#include <cassert>
#include <chrono>
#include <deque>
#include <vector>
#include <thread>
#include <mutex>
#include <vector>
#include <stdexcept>  // std::logic_error(逻辑错误，如参数非法、越界) std::runtime_error(运行时错误，如内存不足、IO失败) std::range_error(数值范围错误) std::overflow_error(算数上溢) std::underflow_error(算数下溢) std::system_error(系统错误)
#include <sstream>
#include <unordered_map>
#include <mavlink_dialect.h>

namespace mavconn{

    // C++11 中引入的时间相关类，它属于 std::chrono 命名空间。steady_clock 用来表示一种稳定的时钟，它的时间不会受到系统时间调整（比如夏令时、手动调整时间等）的影响。steady_clock 可以用于测量经过的时间间隔。
    // 简而言之，steady_clock 适用于那些要求高精度且不会回退的时间测量，比如性能分析、任务计时等场景。
    /**
     * ...
     * auto start_time = steady_clock::now();
     * for (int i = 0; i < 1000000; i++);
     * auto end_time = steady_clock::now();
     *
     * auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
     *
     * std::cout << "task execute time: " << duration.count() << std::end;
     * ...
     */
    using stead_clock = std::chrono::steady_clock;
    // std::lock_guard 是一个模板类，它的作用是保证在程序中某个特定范围内（通常是一个代码块）对一个互斥量（mutex）进行加锁和解锁操作。
    // std::lock_guard 会在创建时自动锁定互斥量，并在其生命周期结束时自动解锁，因此它是一个基于 RAII（资源获取即初始化）的工具，可以有效避免因为忘记解锁而导致的死锁。
    // std::recursive_mutex 是一种特殊的互斥量，它允许同一个线程多次加锁同一个互斥量，而不会造成死锁。通常用于递归调用需要多次加锁的情况。
    // 当你使用 std::lock_guard<std::recursive_mutex> 时，lock_guard 会确保锁的加解锁过程是自动的、线程安全的。
    using lock_guard = std::lock_guard<std::recursive_mutex>;

    //! sSame as @p mavlink::minimal::MAV_COMPONENT::MAV_COMP_ID_UDP_BRIDGE
    static constexpr auto MAV_COMP_ID_UDP_BRIDGE = 240;

    //! Same as @p mavlink::mavlink_framing_t
    enum class Framing : uint8_t {
        // 帧不完整（数据未接收全）
        incomplete = mavlink::MAVLINK_FRAMING_INCOMPLETE,
        // 帧完整且 CRC 校验通过
        ok = mavlink::MAVLINK_FRAMING_OK,
        // CRC 校验失败
        bad_crc = mavlink::MAVLINK_FRAMING_BAD_CRC,
        // 签名错误（仅 MAVLink v2.0 支持）
        bad_signature = mavlink::MAVLINK_FRAMING_BAD_SIGNATURE
    };

    // 指定 MAVLink 协议版本
    enum class Protocol : uint8_t {
        // MAVLink v1.0（无签名、最大载荷 255 字节）
        V1 = 1,
        // MAVLink v2.0（支持签名、更大载荷280字节、扩展消息 ID）
        V2 = 2
    };

    /**
     * DeviceError
     * @brief Common exception for communication error
     * 		  专门用于处理通信设备错误（如串口打开失败、网络连接超时）
     */
    class DeviceError : public std::runtime_error {
        public:
            template<typename T>
            DeviceError(const char* module, T msg):
                std::runtime_error(make_message(module, msg)){}

            template<typename T>
            std::string make_message(const char *module, T msg) {
                std::ostringstream ss;
                ss << "DeviceError: " << module << ": " << msg_to_string(msg);

                return ss.str();
            }

            static std::string msg_to_string(const char* description) {
                return description;
            }

            // ::strerror() 是一个 C 标准库函数，它接受一个错误码，并返回一个对应的错误信息字符串，通常是操作系统定义的错误描述。
            static std::string msg_to_string(int errnum) {
                // 输出类似 Invalid argument
                return ::strerror(errnum);
            }

            // boost::system::system_error 是 Boost 库中的一个类，表示系统级的错误。它通常用来表示与操作系统交互时发生的错误，包含了错误码和一个描述该错误的字符串。
	        // what() 函数返回一个 C 风格字符串（const char*），该字符串包含错误的描述信息。
            static std::string msg_to_string(boost::system::system_error &err) {
                // 输出类似 No such file or directory
                return err.what();
            }
    };

    class MAVConnInterface {
        public:
            MAVConnInterface(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);

        private:
            // 删除了 MAVConnInterface 类的拷贝构造函数。这意味着你不能拷贝该类的对象。删除拷贝构造函数的一个常见原因是防止不小心拷贝一个管理资源的对象，尤其是当类包含资源管理（如内存、文件句柄、网络连接等）时，这样可以避免潜在的资源泄漏或重复销毁资源。
	        MAVConnInterface(const MAVConnInterface&) = delete;

        public:
            // std::function 是一个通用的函数包装器，允许你使用各种可调用对象（如普通函数、Lambda 表达式、成员函数等）。
            // ReceivedCb 是一个回调函数类型的别名，表示一个接受两个参数的函数
            using ReceivedCb = std::function<void (const mavlink::mavlink_message_t *message, const Framing framing)>;
            // ClosedCb 是一个没有参数和返回值的回调函数类型的别名。它通常表示连接关闭时的回调函数，void 参数和返回值表明该回调不需要任何输入或返回任何值。
            using ClosedCb = std::function<void (void)>;
            // 表示对 MAVConnInterface 对象的共享所有权。shared_ptr 会在最后一个引用被销毁时自动删除对象，避免内存泄漏。 禁止拷贝 ≠ 禁止共享
            // shared_ptr 共享：多个 shared_ptr 指向同一个对象实例，通过引用计数管理生命周期，内存中仅一份数据。
            // Ptr p1 = std::make_shared<MAVConnInterface>();：创建对象并由 p1 管理；
            // Ptr p2 = p1;：共享同一个对象，引用计数 + 1，并未拷贝对象；
            // p1->send_message(...); 和 p2->send_message(...);：操作的是同一个对象实例。
            using Ptr = std::shared_ptr<MAVConnInterface>;
            // 表示一个对 MAVConnInterface 对象的常量共享所有权，保证对象不会被修改。Ptr与ConstPtr引用计数机制完全相同（拷贝指针时引用计数 + 1，析构时 - 1），都实现了对象的安全共享，但ConstPtr通过const约束强化了 “只读” 语义
            using ConstPtr = std::shared_ptr<MAVConnInterface const>;
            // 表示对 MAVConnInterface 对象的弱引用。weak_ptr 不影响对象的引用计数，它用于解决循环引用问题。在 shared_ptr 引用计数为 0 时，weak_ptr 会变为过期，访问时需要调用 lock() 来检查是否有效。
            using WeakPtr = std::weak_ptr<MAVConnInterface>;

            // IO数据传输数据端口
            struct IOStat{
                size_t tx_total_bytes;
                size_t rx_total_bytes;
                float tx_speed;
                float rx_speed;
            };

            /**
             * @brief Establish connection, automatically called by open_url()
             * 			纯虚函数定义了 “连接” 的生命周期操作（建立、关闭、发送）
             * 			建立连接，并注册两个回调函数：
             * 				- ReceivedCb：接收消息的回调
             * 				- ClosedCb：连接关闭的回调
             *
             * 			连接成功后，后续接收的 MAVLink 消息会通过 ReceivedCb 通知上层。
             */
            virtual void connect(const ReceivedCb &cb_handle_message, const ClosedCb &cb_handle_closed_port = ClosedCb()) = 0;

            /**
             * @brief Close connection.
             * 			纯虚函数定义了 “连接” 的生命周期操作（建立、关闭、发送）
             * 			关闭连接（释放资源，如关闭串口、断开 TCP 连接）
             * 				调用后 is_open() 需返回 false，且不再接收 / 发送消息。
             */
            virtual void close() = 0;

            /**
             * @brief Send message (mavlink_message_t)
             *			纯虚函数定义了 “连接” 的生命周期操作（建立、关闭、发送）
            * Can be used to forward messages from other connection channel.
            *			发送已序列化的 MAVLink 消息（mavlink_message_t 类型）
            * @note Does not do finalization!
            *			不负责消息序列化，仅转发；若发送队列满（超过 MAX_TXQ_SIZE），抛 std::length_error。
            * @throws std::length_error  On exceeding Tx queue limit (MAX_TXQ_SIZE)
            * @param[in] *message  not changed
            */
            virtual void send_message(const mavlink::mavlink_message_t *msg) = 0;

            /**
             * @brief Send message (child of mavlink::Message)
             * 			mavlink::Message 通常是一个基类或接口，用于统一表示所有 MAVLink 消息类型，提供了消息操作的通用接口。
             *			纯虚函数定义了 “连接” 的生命周期操作（建立、关闭、发送）
            * Does serialization inside.
            * System and Component ID = from this object.
            *
            * @throws std::length_error  On exceeding Tx queue limit (MAX_TXQ_SIZE)
            * @param[in] &message  not changed
            */
            virtual void send_message(const mavlink::Message &message) {
                send_message(message, this->comp_id);
            }

            /**
             * @brief Send message (child of mavlink::Message)
             *			发送 MAVLink 消息对象（如 mavlink::common::HEARTBEAT）
            * Does serialization inside.
            * System ID = from this object.
            * Component ID passed by argument.
            *		自动完成消息序列化（填充系统 ID、组件 ID），组件 ID 可通过参数指定。
            * @throws std::length_error  On exceeding Tx queue limit (MAX_TXQ_SIZE)
            * @param[in] &message  not changed
            * @param[in] src_compid  sets the component ID of the message source
            */
            virtual void send_message(const mavlink::Message &message, const uint8_t src_comp_id) = 0;

            /**
             * @brief Send raw bytes (for some quirks)
             * 			发送原始字节流（用于处理非标准 MAVLink 数据，如自定义协议扩展）
             * @throws std::length_error  On exceeding Tx queue limit (MAX_TXQ_SIZE)
             * 			同样受发送队列大小限制，满则抛异常
             */
            virtual void send_bytes(const uint8_t *bytes, size_t length) = 0;

            /**
             * @brief Send message and ignore possible drop due to Tx queue limit
             */
            void send_message_ignore_drop(const mavlink::mavlink_message_t *message);

            /**
             * @brief Send message and ignore possible drop due to Tx queue limit
             *			当发送队列满时，不抛异常，直接丢弃当前消息（适用于非关键消息，如日志、调试信息）。
                        普通 send_message 会抛 std::length_error，需上层捕获；此接口适合对可靠性要求低的场景。
            * System and Component ID = from this object.
            */
            void send_message_ignore_drop(const mavlink::Message &message) {
                send_message_ignore_drop(message, this->comp_id);
            }

            /**
             * @brief Send message and ignore possible drop due to Tx queue limit
             *
             * System ID = from this object.
             * Component ID passed by argument.
             */
            void send_message_ignore_drop(const mavlink::Message &message, uint8_t comp_id);

            ReceivedCb message_received_cb;
            ClosedCb port_close_cb;

            // 返回 mavlink_status_t（MAVLink 原生结构体），包含已接收的消息数、丢包数、解析错误数等
            virtual mavlink::mavlink_status_t get_status();

            /**
             * 通过 get_iostat() 返回 IOStat 结构体，包含：
             * 累计收发字节数（tx_total_bytes/rx_total_bytes）；
             * 实时收发速率（tx_speed/rx_speed，单位 B/s，通过周期性计算字节差实现）。
             */
            virtual IOStat get_iostat();

            // 判断连接是否处于 “已打开” 状态
	        // 上层通过此接口检查连接有效性（如发送前先判断 is_open()）
            virtual bool is_open() = 0;

            inline uint8_t get_system_id() {
                return this->sys_id;
            }

            inline uint8_t get_component_id() {
                return this->comp_id;
            }

            inline void set_system_id(const uint8_t sysid) {
                this->sys_id = sysid;
            }

            inline void set_component_id(const uint8_t compid) {
                this->comp_id = compid;
            }

            void set_protocol_version(Protocol prot);
            Protocol get_protocol_version();

            /**
             * @brief open_url(...) / open_url_no_connect(...)
             * 静态工厂函数，根据 URL 字符串创建具体的连接实例（如 udp://localhost:14550 创建 UDP 连接，
             * serial:///dev/ttyUSB0:57600 创建串口连接），无需上层直接实例化子类，符合 “工厂模式” 设计，
             * 简化连接创建流程
             */
            /**
             * @brief Construct connection from URL
             *
             * Supported URL schemas:
             * - serial://
             * - udp://
             * - tcp://
             * - tcp-l://
             *
             * Please see user's documentation for details.
             * 目的是启动与指定 URL 的连接，并设置接收到消息和连接关闭时的回调处理。
             * open_url_no_connect 是 open_url 的版本，但是它不立即进行连接（即不调用 connect()）。该版本可能用于初始化连接设置，但连接的建立将在后续某个时刻由用户或程序控制。
             *
             * @param[in] url           resource locator
             * @param[in] system_id     optional system id
             * @param[in] component_id  optional component id
             * @return @a Ptr to constructed interface class,
             *         or throw @a DeviceError if error occured.
             */
            static Ptr open_url(std::string url, uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
                            const ReceivedCb &cb_handle_message = ReceivedCb(),
                            const ClosedCb &cb_port_close_cb = ClosedCb());

            /**
             * @brief version of open_url() which do not perform connect()
             */
            static Ptr open_url_no_connect(std::string url, uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);

            static std::vector<std::string> get_known_dialects();

        protected:
            uint8_t sys_id;     //!< Connection System Id
            uint8_t comp_id;    //!< Connection Component Id

            //! Maximum mavlink packet size + some extra bytes for padding.
	        // （最大数据包长度）MAVLINK_MAX_PACKET_LEN 是 MAVLink 协议规定的最大载荷长度（v1.0 为 255，v2.0 为 267）,主要是MsgBuffer最大MAX_SIZE
            static constexpr size_t MAX_PACKET_SIZE = MAVLINK_MAX_PACKET_LEN + 16;

            //! Maximum count of transmission buffers.
	        // (最大发送队列长度），防止发送消息过多导致内存溢出；队列满时普通发送接口抛异常，提醒上层控制发送频率。
            static constexpr size_t MAX_TXQ_SIZE = 1000;

            //! This map merge all dialect mavlink_msg_entry_t structs. Needed for packet parser.
            // 存储所有 MAVLink 消息的元信息（键：消息 ID msgid_t，值：消息元信息 mavlink_msg_entry_t）。
            // 元信息包含消息长度、字段定义、CRC 种子等，是消息序列化 / 反序列化的核心依据                 \
            // 		由 init_msg_entry() 函数（在 mavlink_helpers.cpp 中自动生成）初始化             \
            // 		且通过 std::once_flag init_flag 确保仅初始化一次（线程安全）。
            static std::unordered_map<mavlink::msgid_t, const mavlink::mavlink_msg_entry_t*> message_entries;

            //! Channel number used for logging.
            size_t conn_id;

            // 获取当前mavlink解析状态指针
            inline mavlink::mavlink_status_t *get_status_p() {
                return &m_parse_status;
            }
            // 获取当前mavlink解析buffer指针
            inline mavlink::mavlink_message_t *get_buffer_p() {
                return &m_buffer;
            }
            /**
             * Parse buffer and emit massage_received.
             * 		通用的 MAVLink 字节流解析函数(子类接收原始字节后调用)
             * 			1. 将原始字节（如串口读取的字节、UDP 接收的数据包）填入解析缓冲区
             * 			2. 调用 MAVLink 原生解析函数（mavlink_parse_char）解析帧；
             * 			3. 解析完成后，通过 ReceivedCb 回调上层（传入消息指针和帧状态 Framing）。
             * @param pfx,       // 日志前缀，用于标识日志的来源
             * @param buf,          // 输入的字节缓冲区，包含接收到的数据
             * @param ufsize,  // 缓冲区的大小
             * @param bytes_received  // 接收到的字节数
             */
            void parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received);

            void iostat_tx_add(size_t bytes);
            void iostat_rx_add(size_t bytes);

            void log_recv(const char *pfx, mavlink::mavlink_message_t &msg, Framing frame);
            void log_send(const char *pfx, mavlink::mavlink_message_t *msg);
            void log_send_obj(const char *pfx, const mavlink::Message &msg);

        private:
            friend const mavlink::mavlink_msg_entry_t *mavlink::mavlink_get_msg_entry(uint8_t megid);

            // MAVLink 协议中用于存储解析状态信息的数据结构，主要用于跟踪 MAVLink 消息的解析过程、帧同步状态和错误统计等
            mavlink::mavlink_status_t m_parse_status;
            // MAVLink 协议中用于表示完整消息的数据结构，定义了 MAVLink 消息的所有核心字段，是消息传输和解析的核心载体
            mavlink::mavlink_message_t m_buffer;
            mavlink::mavlink_status_t m_mavlink_status;

            // 确保无锁的线程安全访问，避免竞态条件
            std::atomic<size_t> tx_total_bytes, rx_total_bytes;

            // 使用 std::recursive_mutex（递归互斥锁）保护 IO 统计数据（iostat_mutex），
	        // 允许同一线程多次加锁（如在回调中调用统计接口）。
            std::recursive_mutex iostat_mutex;
            size_t last_tx_total_bytes, last_rx_total_bytes;

            // std::chrono::system_clock：系统时钟（可修改，对应现实时间）；
            // std::chrono::steady_clock：稳定时钟（不可修改，适合测量时间间隔）；
            // std::chrono::high_resolution_clock：高精度时钟（通常是 steady_clock 或 system_clock 的别名）。
            std::chrono::time_point<stead_clock> last_iostat;

            // 为每个连接实例分配唯一的 conn_id（从 0 开始递增），用于日志区分不同连接。
            static std::atomic<size_t> conn_id_counter;

            // std::call_once 实现多线程环境下某个函数仅执行一次的语义，确保即使多个线程同时调用，目标函数也只会被执行一次，常用于单例初始化、全局资源加载等场景
            static std::once_flag init_flag;

            static void init_msg_entry();
    };
}
