/**
 * @brief Mavlink diag class
 * @file mavlink_diag.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavlink_diag.h>

using namespace mavros;

// 允许你为任务指定一个名称，该名称将在诊断信息中显示。
MavlinkDiag::MavlinkDiag(std::string name) :
	diagnostic_updater::DiagnosticTask(name),
	last_drop_count(0),
	is_connected(false)
{ }

void MavlinkDiag::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (auto link = weak_link.lock()) {			// 尝试获取 weak_link 的共享指针。如果 weak_link 已经指向的对象被销毁，lock() 将返回 nullptr，此时诊断任务会报告 "not connected"。
		auto mav_status = link->get_status();	// 从 MAVConnInterface 获取 MAVLink 连接的状态，包括接收到的数据包数量、丢失的数据包数量、缓冲区溢出、解析错误等信息。
		auto iostat = link->get_iostat();

		// 将诊断信息添加到 diagnostic_updater::DiagnosticStatusWrapper 中，addf 是带格式化字符串的添加函数。
		// add 方法用于将诊断任务添加到更新器中。
		stat.addf("Received packets:", "%u", mav_status.packet_rx_success_count);
		stat.addf("Dropped packets:", "%u", mav_status.packet_rx_drop_count);
		stat.addf("Buffer overruns:", "%u", mav_status.buffer_overrun);
		stat.addf("Parse errors:", "%u", mav_status.parse_error);
		stat.addf("Rx sequence number:", "%u", mav_status.current_rx_seq);
		stat.addf("Tx sequence number:", "%u", mav_status.current_tx_seq);

		stat.addf("Rx total bytes:", "%u", iostat.rx_total_bytes);
		stat.addf("Tx total bytes:", "%u", iostat.tx_total_bytes);
		stat.addf("Rx speed:", "%f", iostat.rx_speed);
		stat.addf("Tx speed:", "%f", iostat.tx_speed);

		// 包统计：如果当前丢包数量大于上次报告的丢包数量，显示丢包的具体数量。
		if (mav_status.packet_rx_drop_count > last_drop_count)
			// summary 是 diagnostic_updater::DiagnosticStatusWrapper 的成员函数，它用于记录当前任务的诊断状态（OK、WARN、ERROR）。
			stat.summaryf(1, "%d packeges dropped since last report",
				mav_status.packet_rx_drop_count - last_drop_count);
		else if (is_connected)
			stat.summary(0, "connected");
		else
			// link operational, but not connected
			stat.summary(1, "not connected");

		last_drop_count = mav_status.packet_rx_drop_count;
	} else {
		stat.summary(2, "not connected");
	}
}


/**
 * EXample:
 * cxy@cxy-T6AD:~/catkin_ws$  rostopic echo /diagnostics
 * 
header:
  stamp: ...  # 时间戳
status:
  - name: "sensor/imu"  # 诊断项名称
    level: 0  # 0=OK, 1=WARN, 2=ERROR, 3=STALE
    message: "IMU working normally"  # 状态描述
    values:  # 详细键值对
      - key: "temperature"
        value: "35.2 C"
      - key: "sampling_rate"
        value: "200 Hz"
  - name: "system/cpu"
    level: 1  # 警告
    message: "CPU usage high"
    values:
      - key: "usage"
        value: "85%"

 */
