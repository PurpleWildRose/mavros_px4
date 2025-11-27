/**
 * @brief MAVConn message buffer class (internal)
 * @file msgbuffer.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <cassert>
#include <mavconn/mavlink_dialect.h>

namespace mavconn {
/**
 * @brief Message buffer for internal use in libmavconn.
 * 			用于 MAVLink 消息的内部缓冲处理。
 */
struct MsgBuffer {
	//! Maximum buffer size with padding for CRC bytes (280 + padding)
	static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
	uint8_t data[MAX_SIZE];
	ssize_t len;
	ssize_t pos;

	MsgBuffer() :
		len(0),		// 表示缓冲区尚未存储任何数据。
		pos(0)
	{ }

	/**
	 * @brief Buffer constructor from mavlink_message_t.
	 * 			explicit 是一个关键字，用于修饰单参数构造函数（或只有第一个参数无默认值、其余参数有默认值的构造函数），其核心作用是禁止隐式类型转换，仅允许显式调用构造函数，从而避免意外的类型转换导致的逻辑错误。
	 */
	explicit MsgBuffer(const mavlink::mavlink_message_t *msg) :
		pos(0)
	{
		len = mavlink::mavlink_msg_to_send_buffer(data, msg);
		// paranoic check, it must be less than MAVLINK_MAX_PACKET_LEN
		assert(len < MAX_SIZE);
	}

	/**
	 * @brief Buffer constructor for mavlink::Message derived object.
	 */
	MsgBuffer(const mavlink::Message &obj, mavlink::mavlink_status_t *status, uint8_t sysid, uint8_t compid) :
		pos(0)
	{
		mavlink::mavlink_message_t msg;
		mavlink::MsgMap map(msg);

		auto mi = obj.get_message_info();

		obj.serialize(map);
		mavlink::mavlink_finalize_message_buffer(&msg, sysid, compid, status, mi.min_length, mi.length, mi.crc_extra);

		len = mavlink::mavlink_msg_to_send_buffer(data, &msg);
		// paranoic check, it must be less than MAVLINK_MAX_PACKET_LEN
		assert(len < MAX_SIZE);
	}

	/**
	 * @brief Buffer constructor for send_bytes()
	 * @param[in] nbytes should be less than MAX_SIZE
	 */
	MsgBuffer(const uint8_t *bytes, ssize_t nbytes) :
		len(nbytes),
		pos(0)
	{
		assert(0 < nbytes && nbytes < MAX_SIZE);
		memcpy(data, bytes, nbytes);
	}

	virtual ~MsgBuffer() {
		pos = 0;
		len = 0;
	}

	uint8_t *dpos() {
		return data + pos;
	}

	ssize_t nbytes() {
		return len - pos;
	}
};
}	// namespace mavconn
