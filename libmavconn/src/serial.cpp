/**
 * @brief MAVConn Serial link class
 * @file serial.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016,2018 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <cassert>

#include <mavconn/console_bridge_compat.h>
#include <mavconn/thread_utils.h>
#include <mavconn/serial.h>

#if defined(__linux__)
#include <linux/serial.h>
#endif

namespace mavconn {

using boost::system::error_code;
using boost::asio::io_service;
using boost::asio::buffer;
using mavlink::mavlink_message_t;


#define PFX	"mavconn: serial"
#define PFXd	PFX "%zu: "


MAVConnSerial::MAVConnSerial(uint8_t system_id, uint8_t component_id,
		std::string device, unsigned baudrate, bool hwflow) :
	MAVConnInterface(system_id, component_id),
	io_service(),
	serial_dev(io_service),
	tx_in_progress(false),
	tx_q {},
	rx_buf {}
{
	using SPB = boost::asio::serial_port_base;

	CONSOLE_BRIDGE_logInform(PFXd "device: %s @ %d bps", conn_id, device.c_str(), baudrate);

	try {
		serial_dev.open(device);

		// Set baudrate and 8N1 mode

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
		// Flow control setting in older versions of Boost.ASIO is broken, use workaround (below) for now.
		serial_dev.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
#elif BOOST_ASIO_VERSION < 101200 && defined(__linux__)
		// Workaround to set some options for the port manually. This is done in
		// Boost.ASIO, but until v1.12.0 (Boost 1.66) there was a bug which doesn't enable relevant
		// code. Fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
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
	catch (boost::system::system_error &err) {
		throw DeviceError("serial", err);
	}
}

MAVConnSerial::~MAVConnSerial()
{
	close();
}

void MAVConnSerial::connect(
		const ReceivedCb &cb_handle_message,
		const ClosedCb &cb_handle_closed_port)
{
	message_received_cb = cb_handle_message;
	port_closed_cb = cb_handle_closed_port;

	// give some work to io_service before start
	// io_service.post 将一个任务添加到 io_service 中，任务是调用 MAVConnSerial::do_read 方法。
	// std::bind(&MAVConnSerial::do_read, this) 将当前对象（this）与 do_read 方法绑定，确保在任务执行时能正确调用 this->do_read()。
	io_service.post(std::bind(&MAVConnSerial::do_read, this));

	// run io_service for async io
	// 创建了一个新线程来运行 io_service.run()。该线程会启动异步 I/O 的执行。
	// io_service.run() 会启动 I/O 处理，并持续等待和处理任务，直到 io_service 停止。也就是说，这个线程会在后台一直运行，处理所有异步 I/O 请求。
	// utils::set_this_thread_name("mserial%zu", conn_id) 设置当前线程的名称为 "mserial0", "mserial1" 等，以便在调试时更容易识别不同的线程。conn_id 是一个连接标识符，通常用于区分不同的连接。
	io_thread = std::thread([this] () {
				utils::set_this_thread_name("mserial%zu", conn_id);
				io_service.run();
			});
}

void MAVConnSerial::close()
{
	lock_guard lock(mutex);
	if (!is_open())
		return;

	serial_dev.cancel();
	serial_dev.close();

	io_service.stop();

	if (io_thread.joinable())
		io_thread.join();

	io_service.reset();

	if (port_closed_cb)
		port_closed_cb();
}

void MAVConnSerial::send_bytes(const uint8_t *bytes, size_t length)
{
	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnSerial::send_bytes: TX queue overflow");

		// emplace_back 用于向 tx_q 队列的末尾添加一个新的元素。
		tx_q.emplace_back(bytes, length);
	}
	io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

void MAVConnSerial::send_message(const mavlink_message_t *message)
{
	assert(message != nullptr);

	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	log_send(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnSerial::send_message: TX queue overflow");

		tx_q.emplace_back(message);
	}
	io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

void MAVConnSerial::send_message(const mavlink::Message &message, const uint8_t source_compid)
{
	if (!is_open()) {
		CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
		return;
	}

	log_send_obj(PFX, message);

	{
		lock_guard lock(mutex);

		if (tx_q.size() >= MAX_TXQ_SIZE)
			throw std::length_error("MAVConnSerial::send_message: TX queue overflow");
		
		// 序列化消息并加入队列
		// sys_id：发送方的系统 ID（uint8_t），标识消息来自哪个系统（如无人机、地面站）。
		// component_id：发送方的组件 ID（uint8_t），标识消息来自哪个组件（如飞控、摄像头等）。
		tx_q.emplace_back(message, get_status_p(), sys_id, source_compid);
	}
	// 触发发送...
	io_service.post(std::bind(&MAVConnSerial::do_write, shared_from_this(), true));
}

void MAVConnSerial::do_read(void)
{
	// 获取自身共享指针
	auto sthis = shared_from_this();
	// 发起异步读取
	serial_dev.async_read_some(
			// buffer(rx_buf) 指定接收缓冲区
			buffer(rx_buf),
			// 完成处理程序（lambda 表达式），当读取操作完成（或出错）时被调用。
			// error_code error：读取操作的错误状态（成功时为 0）。
			// size_t bytes_transferred：实际读取到的字节数。
			[sthis] (error_code error, size_t bytes_transferred) {
				if (error) {
					CONSOLE_BRIDGE_logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
					sthis->close();
					return;
				}
				// total complete bytes.
				// std::cout << "Receive : " << bytes_transferred << " bytes" << std::endl;
				sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
				sthis->do_read();
			});
}

void MAVConnSerial::do_write(bool check_tx_state)
{
	if (check_tx_state && tx_in_progress)
		return;

	lock_guard lock(mutex);
	if (tx_q.empty())
		return;

	tx_in_progress = true;
	auto sthis = shared_from_this();
	auto &buf_ref = tx_q.front();
	serial_dev.async_write_some(
			buffer(buf_ref.dpos(), buf_ref.nbytes()),
			[sthis, &buf_ref] (error_code error, size_t bytes_transferred) {
				assert(bytes_transferred <= buf_ref.len);

				if (error) {
					CONSOLE_BRIDGE_logError(PFXd "write: %s", sthis->conn_id, error.message().c_str());
					sthis->close();
					return;
				}

				// 更新发送统计
				sthis->iostat_tx_add(bytes_transferred);
				lock_guard lock(sthis->mutex);

				if (sthis->tx_q.empty()) {
					sthis->tx_in_progress = false;
					return;
				}

				buf_ref.pos += bytes_transferred;
				if (buf_ref.nbytes() == 0) {
					sthis->tx_q.pop_front();
				}

				if (!sthis->tx_q.empty())
					sthis->do_write(false);
				else
					sthis->tx_in_progress = false;
			});
}
}	// namespace mavconn
