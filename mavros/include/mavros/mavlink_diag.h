/**
 * @brief Mavlink diag class
 * @file mavlink_diag.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <diagnostic_updater/diagnostic_updater.h>	// diagnostic_updater::DiagnosticTask 是 ROS (Robot Operating System) 中 diagnostic_updater 包的一个类，专门用于定期执行任务并报告系统的诊断信息。它是 diagnostic_updater 组件的一部分，通常用于在机器人系统中进行健康检查和性能监控。
#include <mavconn/interface.h>

namespace mavros {
// diagnostic_updater 提供了一种机制，允许用户定期获取诊断信息并发布这些信息，通常是通过 ROS 消息进行发布。这些信息可以包括温度、电池电量、系统负载、传感器状态等。
class MavlinkDiag : public diagnostic_updater::DiagnosticTask
{
public:
	explicit MavlinkDiag(std::string name);

	// 这个函数是定期运行的诊断任务核心，用于更新连接的诊断状态。
	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

	// 设置 MAVLink 连接接口。
	void set_mavconn(const mavconn::MAVConnInterface::Ptr &link) {
		weak_link = link;
	}

	// 设置连接状态（true 表示连接，false 表示断开）。
	void set_connection_status(bool connected) {
		is_connected = connected;
	}

private:
	// weak_link：这是一个弱引用，指向一个 MAVConnInterface 的共享指针，代表与 MAVLink 通信的连接。弱引用确保即使连接对象被销毁，MavlinkDiag 仍然可以安全地工作，而不会导致内存泄漏或悬空指针。
	mavconn::MAVConnInterface::WeakPtr weak_link;
	// last_drop_count：上次报告时接收到的丢包数。这个变量用于检测自上次报告以来丢失的包数量。
	unsigned int last_drop_count;
	// is_connected：一个原子变量，表示 MAVLink 连接的状态。它用于跟踪连接是否成功建立。
	std::atomic<bool> is_connected;
};
}	// namespace mavros

