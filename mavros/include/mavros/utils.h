/**
 * @brief some useful utils
 * @file utils.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavutils
 * @{
 *  @brief Some useful utils
 */
/*
 * Copyright 2014,2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <Eigen/Geometry>
#include <mavconn/thread_utils.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavconn/mavlink_dialect.h>

#include <ros/console.h>

// OS X compat: missing error codes
#ifdef __APPLE__
#define EBADE 50	/* Invalid exchange */
#define EBADFD 81	/* File descriptor in bad state */
#define EBADRQC 54	/* Invalid request code */
#define EBADSLT 55	/* Invalid slot */
#endif

namespace mavros {
namespace utils {
using mavconn::utils::format;

/**
 * Possible modes of timesync operation
 *
 * Used by UAS class, but it can't be defined inside because enum is used in utils.
 * 无人机系统中，飞控（FCU，如 PX4/ArduPilot）和 ROS 节点（如 MAVROS）通常有各自的时钟（FCU 用硬件时钟，ROS 用系统软件时钟）。
 * 若两者时钟偏差较大，会导致传感器数据的时间戳错位（如 IMU 数据标着 “10:00:01”，GPS 标着 “10:00:03”），进而影响 EKF（扩展卡尔曼滤波）等姿态解算模块的精度。
 * 
 * 1. ROS → FCU：MAVROS 定期发送MAVLINK_MSG_ID_TIMESTAMP消息，携带 ROS 当前时间戳（ros_time）和发送时刻的本地单调时钟（local_monotonic_time）。
 * 2. FCU → ROS：FCU 接收后，立即返回MAVLINK_MSG_ID_TIMESTAMP消息，携带 FCU 当前时间戳（fcu_time）和 ROS 发送的local_monotonic_time（用于 MAVROS 匹配请求）。
 * 3. 计算偏差：MAVROS 根据 “发送 - 接收” 的时间差，计算 ROS 与 FCU 的时钟偏差（offset = fcu_time - (ros_send_time + round_trip_time/2)）。
 * 4. 校准 ROS 时钟：MAVROS 将偏差应用到 ROS 的Time接口，使ros::Time::now()返回的时间接近 FCU 时钟。
 */
enum class timesync_mode {
	/**
	 * 	禁用时间同步功能
	 * 		1. 仅用于调试（如验证 “无同步时数据偏差影响”）；
	 * 		2. 简单测试场景（无需高精度姿态解算）
	 */
	NONE = 0,	//!< Disabled

	/**
	 * 	通过 MAVLink 协议的TIMESYNC消息实现同步（最常用）
	 * 		主流场景：ROS 与 FCU 通过串口 / 网口（如 USB、UDP）通信（如地面站、机载计算机与飞控交互）
	 */
	MAVLINK,	//!< Via TIMESYNC message

	/**
	 * 	板载同步（ROS 与 FCU 共享同一硬件时钟）
	 * 		板载计算场景：ROS 节点与 FCU 运行在同一硬件（如 PX4 的 FMU+Onboard Computer 一体化模块）
	 * 
	 * 	1. 共享时钟源：ROS 与 FCU 直接使用硬件的同一时钟（如 MCU 的定时器、系统晶振）；
	 * 	2. 零偏差：无需通过消息交互计算偏移，时钟天然一致，同步精度最高（微秒级）
	 * 	3. 依赖硬件：仅适用于 ROS 与 FCU 物理集成的场景（如定制化机载计算机）。
	 */
	ONBOARD,

	/**
	 * 	透传同步（ROS 不直接校准自身时钟，而是转发第三方时钟给 FCU）
	 * 		多设备协同场景：存在外部高精度时钟源（如 GNSS 时间、硬件 PTP 时钟），需统一所有设备时钟
	 * 
	 * 	1. 时钟透传：ROS 接收外部高精度时钟（如 GNSS 的 UTC 时间），不调整自身时钟，而是将该时钟通过 MAVLink 转发给 FCU；
	 * 	2. 统一基准：使 FCU 与外部时钟源对齐，ROS 则通过其他方式（如 NTP）与该时钟源同步，间接实现三者时钟一致；
	 * 	3. 高精度需求：适用于多无人机协同、长航时作业（需绝对时间同步）。
	 */
	PASSTHROUGH,
};

/**
 * Helper to get enum value from strongly typed enum (enum class).
 * 	通用模板函数 enum_value，核心作用是将任意枚举类型（包括强类型枚举 enum class）的值，安全转换为其对应的底层整数类型（如 int、uint8_t 等）
 */
template<typename _T>
constexpr typename std::underlying_type<_T>::type enum_value(_T e)
{
	return static_cast<typename std::underlying_type<_T>::type>(e);
}

/**
 * Get string repr for timesync_mode
 * 	要获取 timesync_mode 枚举的字符串
 */
std::string to_string(timesync_mode e);

/**
 * @brief Retrieve alias of the orientation received by MAVLink msg.
 * 	要获取 mavlink dialect 枚举的字符串
 */
std::string to_string(mavlink::common::MAV_SENSOR_ORIENTATION e);

std::string to_string(mavlink::minimal::MAV_AUTOPILOT e);
std::string to_string(mavlink::minimal::MAV_TYPE e);
std::string to_string(mavlink::minimal::MAV_STATE e);
std::string to_string(mavlink::minimal::MAV_COMPONENT e);
std::string to_string(mavlink::common::MAV_ESTIMATOR_TYPE e);
std::string to_string(mavlink::common::ADSB_ALTITUDE_TYPE e);
std::string to_string(mavlink::common::ADSB_EMITTER_TYPE e);
std::string to_string(mavlink::common::MAV_MISSION_RESULT e);
std::string to_string(mavlink::common::MAV_FRAME e);
std::string to_string(mavlink::common::MAV_DISTANCE_SENSOR e);
std::string to_string(mavlink::common::LANDING_TARGET_TYPE e);

/**
 * Helper to call to_string() for enum _T
 * 	模板辅助函数 to_string_enum，用于将整数转换为枚举类型后，再获取该枚举值的字符串表示
 * 
 * Example: std::string mode_str = to_string_enum<timesync_mode>(received_int);
 */
template<typename _T>
std::string to_string_enum(int e)
{
	return to_string(static_cast<_T>(e));
}

/**
 * @brief Function to match the received orientation received by MAVLink msg
 *        and the rotation of the sensor relative to the FCU.
 * 
 * 	通过发送 MAV_SENSOR_ORIENTATION 枚举值让fcu确定传感器的quaterniond
 */
Eigen::Quaterniond sensor_orientation_matching(mavlink::common::MAV_SENSOR_ORIENTATION orientation);

/**
 * @brief Retrieve sensor orientation number from alias name.
 * 
 * 	将传感器朝向的字符串表示转换为对应的 MAV_SENSOR_ORIENTATION 枚举值（整数形式）
 */
int sensor_orientation_from_str(const std::string &sensor_orientation);

/**
 * @brief Retrieve timesync mode from name
 * 	要获取 timesync_mode (timesync_mode) 枚举的字符串
 */
timesync_mode timesync_mode_from_str(const std::string &mode);

/**
 * @brief Retreive MAV_FRAME from name
 * 	将MAVLink 协议中坐标帧字符串显式转换成用户定义的一维数组 MAV_FRAME 格式
 */
mavlink::common::MAV_FRAME mav_frame_from_str(const std::string &mav_frame);

/**
 * @brief Retreive MAV_TYPE from name
 */
mavlink::minimal::MAV_TYPE mav_type_from_str(const std::string &mav_type);

/**
 * @brief Retrieve landing target type from alias name
 */
mavlink::common::LANDING_TARGET_TYPE landing_target_type_from_str(const std::string &landing_target_type);

}	// namespace utils
}	// namespace mavros
