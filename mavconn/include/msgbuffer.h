#pragma once

#include <cassert>
#include <mavlink_dialect.h>

namespace mavconn{
class MsgBuffer{
    public:
        MsgBuffer():
            len(0),
            pos(0) {};

        // 在 C++11 之前，具有单个形参的构造函数被视为转换构造函数，因为它接受另一种类型的值并由此创建该类型的新实例。
        // explicit是一个关键字，专门用于修饰类的构造函数。它的主要作用是禁止编译器进行隐式类型转换，确保对象的创建必须通过显式调用构造函数来完成 。
        // 必须显式创建Animal对象   XXX(MsgBuffer(?mavlink_msg))
        explicit MsgBuffer(const mavlink::mavlink_message_t *msg):
            pos(0) {
                // 将mavlink消息零拷贝到buffer中
                len = mavlink::mavlink_msg_to_send_buffer(data, msg);
                assert(len < MAX_SIZE);
            };

        MsgBuffer(const mavlink::Message &obj, mavlink::mavlink_status_t *status, uint8_t sysid, uint8_t compid):
            pos(0) {
                mavlink::mavlink_message_t msg;
                mavlink::MsgMap map(msg);

                auto mi = obj.get_message_info();

                // 将消息的字段数据写入到 mavlink::MsgMap 缓冲区中，为后续的 MAVLink 消息打包（mavlink_finalize_message_buffer）和字节流转换（mavlink_msg_to_send_buffer）做准备。
                // mavlink_finalize_message_buffer 转换为字节流前，完成消息的协议合规性封装
                obj.serialize(map);
                mavlink::mavlink_finalize_message_buffer(&msg, sysid, compid, status, mi.min_length, mi.length, mi.crc_extra);

                len = mavlink::mavlink_msg_to_send_buffer(data, &msg);
                assert(len < MAX_SIZE);
            }

        MsgBuffer(const uint8_t *bytes, ssize_t nbytes):
            len(nbytes),
            pos(0) {
                assert(0 < nbytes && nbytes < MAX_SIZE);
                memcpy(data, bytes, nbytes);
            }

        ~MsgBuffer() {
            pos = 0;
            len = 0;
        }

        uint8_t *dpos() {
            return data + pos;
        }

        ssize_t nbytes() {
            return len - pos;
        }

    private:
        // MAVLINK_MAX_PACKET_LEN 代表着Mavlink最大的负载消息（v2: 12~280     v1: 8~263）
        static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
        uint8_t data[MAX_SIZE];
        // 有效数据长度（受payload的影响 payload max = 255）
        ssize_t len;
        // 在读取时，读取到的字节
        ssize_t pos;
};
}
