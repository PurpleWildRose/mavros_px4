/**
 * @brief MAVConn Serial link class
 * @file serial.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <mavconn/interface.h>
#include <mavconn/msgbuffer.h>

namespace mavconn {
/**
 * @brief Serial interface
 */
class MAVConnSerial : public MAVConnInterface,
	public std::enable_shared_from_this<MAVConnSerial> {
public:
	// 编译期常量，其值在编译时即可确定，编译器会将其视为常量值直接嵌入代码中（类似宏定义，但类型安全）。
	static constexpr auto DEFAULT_DEVICE = "/dev/ttyACM0";
	static constexpr auto DEFAULT_BAUDRATE = 115200;
	// static constexpr auto DEFAULT_BAUDRATE = 57600;

	/**
	 * Open and run serial link.
	 *
	 * @param[in] device    TTY device path
	 * @param[in] baudrate  serial baudrate
	 * @param[in] hwflow    hardware flow control
	 */
	MAVConnSerial(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE,
			std::string device = DEFAULT_DEVICE, unsigned baudrate = DEFAULT_BAUDRATE, bool hwflow = false);
	virtual ~MAVConnSerial();

	// 连接串口并注册回调：消息接收回调和端口关闭回调
	void connect(
			const ReceivedCb &cb_handle_message,
			const ClosedCb &cb_handle_closed_port = ClosedCb()) override;
	void close() override;

	// 发送 mavlink_message_t 类型消息
	void send_message(const mavlink::mavlink_message_t *message) override;
	// 发送 mavlink::Message 类型消息（更面向对象的消息类型）
	void send_message(const mavlink::Message &message, const uint8_t source_compid) override;
	// 直接发送字节流（用于低级别数据传输）
	void send_bytes(const uint8_t *bytes, size_t length) override;

	inline bool is_open() override {
		return serial_dev.is_open();
	}

private:
	// Boost.Asio 的 I/O 服务，用于异步操作。
	boost::asio::io_service io_service;
	// 运行 io_service 的线程（处理异步事件）。
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

	// 内部函数，处理异步读取（从串口接收数据）。
	void do_read();
	// 内部函数，处理异步写入（向串口发送数据）。
	void do_write(bool check_tx_state);
};
}	// namespace mavconn
