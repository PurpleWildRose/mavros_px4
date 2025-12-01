#include <interface.h>

#ifndef CONSOLE_BRIDGE_logDebug
#include <console_printf.h>
#else
#include <console_bridge/console.h>
#endif

using mavconn::MAVConnInterface;

/**
 * init_msg_entry
 * @brief 提供给上层调用接口，将入口信息加载到MAVConnInterface中的message_entries里的map格式数据。
 *
 * 其只在初始实例时调用一次。
 */
void MAVConnInterface::init_msg_entry() {
    CONSOLE_BRIDGE_logDebug("mavconn: Initialize message_entries map");

    // load的lambda表达式
    auto load = [&](const char *dialect, const mavlink::mavlink_msg_entry_t &e) {
        // 寻找当前unordered_map是否存在遍历的msgid.
        auto it = message_entries.find(e.msgid);
        // 查询到在unordered_map中的存在msgid
        if (it != message_entries.end()) {
            // 对比两条mavlink消息类型（crc ... 等格式）
            if (memcmp(&e, it->second, sizeof(e) != 0)) {
                CONSOLE_BRIDGE_logDebug("mavconn: init: message from %s, MSG-ID %d ignored! Table has different entry.", dialect, e.msgid);
            }
            else {
                CONSOLE_BRIDGE_logDebug("mavconn: init: message from %s, MSG_ID %d in table.", dialect, e.msgid);
            }
        }
        else {
            CONSOLE_BRIDGE_logDebug("mavconn: init: add message entry for %s, MSG-ID %d.", dialect, e.msgid);
            message_entries[e.msgid] = &e;
        }
    };

    for (auto &e : mavlink::common::MESSAGE_ENTRIES)        load("common", e);
    for (auto &e : mavlink::standard::MESSAGE_ENTRIES)      load("standard", e);
}

/**
 * get_known_dialects
 * @brief 获取调用mavlink消息的类。
 */
std::vector<std::string> MAVConnInterface::get_known_dialects() {
    return {
        "common",
        "standard"
    };
}

const mavlink::mavlink_msg_entry_t* mavlink::mavlink_get_msg_entry(uint8_t msgid) {
    auto it = MAVConnInterface::message_entries.find(msgid);
    if (it != MAVConnInterface::message_entries.end()) {
        return it->second;
    }
    else {
        return nullptr;
    }
}


