/**
 * @brief MAVROS Plugin context
 * @file mavros_uas.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2014,2015,2016,2017 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <array>
#include <mutex>
#include <atomic>
#include <type_traits>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mavconn/interface.h>
#include <mavros/utils.h>
#include <mavros/frame_tf.h>

#include <GeographicLib/Geoid.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

/**
 * @brief UAS（Unmanned Aerial System，无人机系统）类是 MAVROS 插件的核心上下文，负责管理无人机（FCU，飞行控制单元）的连接、状态数据、坐标变换、时间同步等关键功能，为所有 MAVROS 插件（如 IMU、GPS、系统状态插件）提供统一的数据访问和工具接口。
 */
// UAS 类是 MAVROS 插件的 “数据中枢” 和 “工具箱”，主要解决以下问题：
//		1. 统一数据存储：集中管理 FCU 发送的关键数据（如 IMU、GPS、心跳信息），避免插件间数据分散。
//		2. FCU 连接抽象：封装 FCU 的通信接口，让插件无需关注底层通信细节（如串口、UDP）。
//		3. 工具函数封装：提供模式字符串转换、坐标变换、时间同步、地理坐标转换等通用功能，减少插件重复代码。
// 		4. 状态与事件通知：通过回调函数传递连接状态、能力变化等事件，实现插件间联动。
namespace mavros {
/**
 * @brief helper accessor to FCU link interface
 */
#define UAS_FCU(uasobjptr)                              \
	((uasobjptr)->fcu_link)

/**
 * @brief helper accessor to diagnostic updater
 */
#define UAS_DIAG(uasobjptr)                             \
	((uasobjptr)->diag_updater)


/**
 * @brief UAS for plugins
 *
 * This class stores some useful data and
 * provides fcu connection, mode stringify utilities.
 *
 * Currently it stores:
 * - FCU link interface
 * - FCU System & Component ID pair
 * - Connection status (@a mavplugin::SystemStatusPlugin)
 * - Autopilot type (@a mavplugin::SystemStatusPlugin)
 * - Vehicle type (@a mavplugin::SystemStatusPlugin)
 * - IMU data (@a mavplugin::IMUPubPlugin)
 * - GPS data (@a mavplugin::GPSPlugin)
 */
class UAS {
	// 按功能可分为 数据更新与获取、通信与状态管理、工具函数 三类
public:
	// common enums used by UAS
	using MAV_TYPE = mavlink::minimal::MAV_TYPE;					// 无人机类型（如多旋翼、固定翼）
	using MAV_AUTOPILOT = mavlink::minimal::MAV_AUTOPILOT;			// 飞控类型（如 PX4、ArduPilot）
	using MAV_MODE_FLAG = mavlink::minimal::MAV_MODE_FLAG;			// 飞行模式标志（如解锁、HIL 模式）
	using MAV_STATE = mavlink::minimal::MAV_STATE;
	using MAV_CAP = mavlink::common::MAV_PROTOCOL_CAPABILITY;		// “协议能力枚举”（MAV_PROTOCOL_CAPABILITY），用于标识无人机自动驾驶仪（或其他 MAVLink 设备）支持的 MAVLink 协议功能。每个枚举值对应一个独立的 “能力标志”，通过 位运算 可组合表示设备的综合能力（例如，同时支持任务指令和参数传输）。
	using timesync_mode = utils::timesync_mode;

	// other UAS aliases
	using ConnectionCb = std::function<void(bool)>;					// 连接状态变化回调（参数：是否连接）
	using CapabilitiesCb = std::function<void(MAV_CAP)>;			// 飞控能力变化回调（参数：新能力）
	using lock_guard = std::lock_guard<std::recursive_mutex>;
	using unique_lock = std::unique_lock<std::recursive_mutex>;

	UAS();
	~UAS() {};

	/**
	 * @brief FCU 通信接口指针（抽象串口 / UDP/TCP 连接，插件通过它发送 / 接收 MAVLink 消息）
	 */
	mavconn::MAVConnInterface::Ptr fcu_link;

	/**
	 * @brief ROS 诊断更新器（用于发布飞控连接状态、数据健康度等诊断信息）
	 */
	diagnostic_updater::Updater diag_updater;

	/**
	 * @brief Return connection status
	 */
	inline bool is_connected() {
		return connected;
	}

	/* -*- HEARTBEAT data -*- */

	/**
	 * 在接收到飞控 HEARTBEAT 消息时更新无人机类型 (MAV_TYPE)、飞控类型 (MAV_AUTOPILOT) 和基本飞行模式标志 (base_mode)。
	 */
	void update_heartbeat(uint8_t type_, uint8_t autopilot_, uint8_t base_mode_);

	/**
	 * 根据心跳或超时信息更新连接状态 connected，并触发注册的回调。
	 */
	void update_connection_status(bool conn_);

	/**
	 * @brief 注册连接状态变化回调函数，例如飞控上线/掉线时通知其他插件。
	 */
	void add_connection_change_handler(ConnectionCb cb);

	/**
	 * @brief Returns vehicle type
	 */
	inline MAV_TYPE get_type() {
		std::underlying_type<MAV_TYPE>::type type_ = type;
		return static_cast<MAV_TYPE>(type_);
	}

	/**
	 * @brief Returns autopilot type
	 */
	inline MAV_AUTOPILOT get_autopilot() {
		std::underlying_type<MAV_AUTOPILOT>::type autopilot_ = autopilot;
		return static_cast<MAV_AUTOPILOT>(autopilot_);
	}

	/**
	 * @brief Returns arming status
	 *
	 * @note There may be race condition between SET_MODE and HEARTBEAT.
	 */
	inline bool get_armed() {
		uint8_t base_mode_ = base_mode;
		return base_mode_ & utils::enum_value(MAV_MODE_FLAG::SAFETY_ARMED);
	}

	/**
	 * @brief Returns HIL status
	 */
	inline bool get_hil_state() {
		uint8_t base_mode_ = base_mode;
		return base_mode_ & utils::enum_value(MAV_MODE_FLAG::HIL_ENABLED);
	}

	/* -*- FCU target id pair -*- */

	/**
	 * @brief Return communication target system
	 */
	inline uint8_t get_tgt_system() {
		return target_system;	// not changed after configuration
	}

	/**
	 * @brief Return communication target component
	 */
	inline uint8_t get_tgt_component() {
		return target_component;// not changed after configuration
	}

	inline void set_tgt(uint8_t sys, uint8_t comp) {
		target_system = sys;
		target_component = comp;
	}


	/* -*- IMU data -*- */

	/**
	 * @brief 更新缓存中的 ENU（East-North-Up）坐标系 IMU 数据。
	 */
	void update_attitude_imu_enu(sensor_msgs::Imu::Ptr &imu);

	/**
	 * @brief 更新缓存中的 NED（North-East-Down）坐标系 IMU 数据。
	 */
	void update_attitude_imu_ned(sensor_msgs::Imu::Ptr &imu);

	/**
	 * @brief 获取最近缓存的 ENU 坐标系下的 IMU 数据。
	 */
	sensor_msgs::Imu::Ptr get_attitude_imu_enu();

	/**
	 * @brief 获取最近缓存的 NED 坐标系下的 IMU 数据。
	 */
	sensor_msgs::Imu::Ptr get_attitude_imu_ned();

	/**
	 * @brief 获取 ENU 姿态四元数（方向信息）
	 * @return orientation quaternion [ENU]
	 */
	geometry_msgs::Quaternion get_attitude_orientation_enu();

	/**
	 * @brief 获取 NED 姿态四元数（方向信息）
	 * @return orientation quaternion [NED]
	 */
	geometry_msgs::Quaternion get_attitude_orientation_ned();

	/**
	 * @brief 获取 ENU 角速度向量
	 * @return vector3 [ENU]
	 */
	geometry_msgs::Vector3 get_attitude_angular_velocity_enu();

	/**
	 * @brief 获取 NED 角速度向量
	 * @return vector3 [NED]
	 */
	geometry_msgs::Vector3 get_attitude_angular_velocity_ned();


	/* -*- GPS data -*- */

	// 存储最新的 GPS 定位数据、水平/垂直精度、定位类型和可见卫星数。
	// GPS 定位+精度信息（eph：水平精度，epv：垂直精度）
	void update_gps_fix_epts(sensor_msgs::NavSatFix::Ptr &fix,
		float eph, float epv,
		int fix_type, int satellites_visible);

	/**
	 * get_gps_epts
	 * 
	 * @brief GPS 精度与状态数据的读取接口，用于从 UAS 类存储的缓存中获取 GPS 模块的关键性能指标（EPH、EPV）和定位状态（Fix Type、卫星数量），供其他插件或模块使用。
	 * @param eph 水平定位精度（Estimated Horizontal Precision），单位：米（m）;表示 GPS 水平方向（经纬度）的定位误差范围（1σ 标准差），值越小精度越高。例：eph=2.5 表示水平定位误差大概率在 2.5 米内。
	 * @param epv 垂直定位精度（Estimated Vertical Precision），单位：米（m）;表示 GPS 垂直方向（高度）的定位误差范围（1σ 标准差），值越小精度越高。例：epv=5.0 表示垂直定位误差大概率在 5.0 米内。
	 * @param fix_type GPS 定位类型（Fix Type），无单位（枚举值）; [- 0：无定位（No Fix） /  - 1：2D 定位（仅经纬度，无高度） / - 2：3D 定位（经纬度 + 高度） / - 3：差分 GPS（DGPS，精度更高） / - 4：RTK 固定解（厘米级精度）]
	 * @param satellites_visible 可见卫星数量（Satellites Visible），无单位（整数）;表示当前 GPS 模块能搜索到的卫星总数（非所有卫星都参与定位，需满足信号质量要求），数量越多通常定位越稳定。
	 */
	void get_gps_epts(float &eph, float &epv, int &fix_type, int &satellites_visible);

	// 获取最近的 GPS 定位数据 (NavSatFix)。
	sensor_msgs::NavSatFix::Ptr get_gps_fix();

	/* -*- GograpticLib utils -*- */

	/**
	 * @brief 地球大地水准面模型（用于 AMSL 高度与 WGS84 椭球面高度的转换）
	 *
	 * That class loads egm96_5 dataset to RAM, it is about 24 MiB.
	 */
	std::shared_ptr<GeographicLib::Geoid> egm96_5;

	/**
	 * @brief Conversion from height above geoid (AMSL)
	 * to height above ellipsoid (WGS-84) / 将大地水准面高度（AMSL）转换为椭球面高度（WGS-84）。
	 */
	template <class T>
	inline double geoid_to_ellipsoid_height(T lla)
	{
		if (egm96_5)
			return GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5)(lla->latitude, lla->longitude);
		else
			return 0.0;
	}

	/**
	 * @brief Conversion from height above ellipsoid (WGS-84)
	 * to height above geoid (AMSL) / 椭球面高度 → 大地水准面高度。
	 */
	template <class T>
	inline double ellipsoid_to_geoid_height(T lla)
	{
		if (egm96_5)
			return GeographicLib::Geoid::ELLIPSOIDTOGEOID * (*egm96_5)(lla->latitude, lla->longitude);
		else
			return 0.0;
	}

	/* -*- transform -*- */

	tf2_ros::Buffer tf2_buffer;										// TF2 坐标变换缓存与监听器（解析 ROS 中的坐标变换，如机体坐标系→世界坐标系）
	tf2_ros::TransformListener tf2_listener;
	tf2_ros::TransformBroadcaster tf2_broadcaster;					// 动态坐标变换发布器（发布无人机实时位姿变换, 如 map -> base_link）
	tf2_ros::StaticTransformBroadcaster tf2_static_broadcaster;		// 静态坐标变换发布器（发布固定变换，如 GPS 天线→机体中心）

	/**
	 * @brief Add static transform. To publish all static transforms at once, we stack them in a std::vector.
	 * 			/ 将静态坐标变换存入向量（批量发布前的缓存）。
	 *
	 * @param frame_id    parent frame for transform
	 * @param child_id    child frame for transform
	 * @param tr          transform
	 * @param vector      vector of transforms
	 */
	void add_static_transform(const std::string &frame_id, const std::string &child_id, const Eigen::Affine3d &tr, std::vector<geometry_msgs::TransformStamped>& vector);

	/**
	 * @brief Publishes static transform.发布单个静态坐标变换。
	 *
	 * @param frame_id    parent frame for transform
	 * @param child_id    child frame for transform
	 * @param tr          transform
	 */
	void publish_static_transform(const std::string &frame_id, const std::string &child_id, const Eigen::Affine3d &tr);

	/* -*- time sync -*- */

	/**
	 * @brief 设置飞控时间与 ROS 时间的偏移量。
	 */
	inline void set_time_offset(uint64_t offset_ns) {
		time_offset = offset_ns;
	}

	/**
	 * @brief 获取飞控时间与 ROS 时间的偏移量。
	 */
	inline uint64_t get_time_offset(void) {
		return time_offset;
	}

	/**
	 * @brief 设置时间同步模式（例如基于心跳或时间戳同步）。
	 */
	inline void set_timesync_mode(timesync_mode mode) {
		tsync_mode = mode;
	}

	/**
	 * @brief 获取时间同步模式（例如基于心跳或时间戳同步）。
	 */
	inline timesync_mode get_timesync_mode(void) {
		return tsync_mode;
	}

	/* -*- autopilot version -*- */

	/**
	 * @brief 获取飞控的 MAVLink 能力掩码，如是否支持参数协议、任务上传等。
	 */
	uint64_t get_capabilities();

	/**
	 * @brief 检查飞控是否支持某一项或多项 MAVLink 能力（通过位运算判断）。
	 *
	 * @param capabilities can accept a multiple capability params either in enum or int from
	 */
	template<typename T>
	bool has_capability(T capability){
		static_assert(std::is_enum<T>::value, "Only query capabilities using the UAS::MAV_CAP enum.");
		return get_capabilities() & utils::enum_value(capability);
	}

	/**
	 * @brief 检查飞控是否支持某一项或多项 MAVLink 能力（通过位运算判断）。
	 *
	 * @param capabilities can accept a multiple capability params either in enum or int from
	 */
	template<typename ... Ts>
	bool has_capabilities(Ts ... capabilities){
		bool ret = true;
		std::initializer_list<bool> capabilities_list{has_capability<Ts>(capabilities) ...};
		for (auto has_cap : capabilities_list) ret &= has_cap;
		return ret;
	}

	/**
	 * @brief 更新飞控的 MAVLink 能力掩码，如是否支持参数协议、任务上传等。
	 */
	void update_capabilities(bool known, uint64_t caps = 0);

	/**
	 * @brief 注册能力变化回调，例如 PX4 从不支持→支持任务上传时通知插件。
	 *
	 * @param cb A void function that takes a single mavlink::common::MAV_PROTOCOL_CAPABILITY(MAV_CAP) param
	 */
	void add_capabilities_change_handler(CapabilitiesCb cb);

	/**
	 * @brief Compute FCU message time from time_boot_ms or time_usec field
	 *
	 * Uses time_offset for calculation / 根据飞控时间戳和偏移量计算 ROS 时间戳。
	 *
	 * @return FCU time if it is known else current wall time.
	 */
	ros::Time synchronise_stamp(uint32_t time_boot_ms);
	ros::Time synchronise_stamp(uint64_t time_usec);

	/**
	 * @brief Create message header from time_boot_ms or time_usec stamps and frame_id.
	 *
	 * Setting frame_id and stamp are pretty common, this little helper should reduce LOC.
	 * 生成带同步时间戳和 frame_id 的标准 ROS 消息头。
	 * 
	 * @param[in] frame_id    frame for header
	 * @param[in] time_stamp  mavlink message time
	 * @return Header with syncronized stamp and frame id
	 */
	template<typename T>
	inline std_msgs::Header synchronized_header(const std::string &frame_id, const T time_stamp) {
		std_msgs::Header out;
		out.frame_id = frame_id;
		out.stamp = synchronise_stamp(time_stamp);
		return out;
	}

	/* -*- utils -*- */

	/**
	 * Helper template to set target id's of message.
	 */
	template<typename _T>
	inline void msg_set_target(_T &msg) {
		msg.target_system = get_tgt_system();
		msg.target_component = get_tgt_component();
	}

	/**
	 * @brief Check that sys/comp id's is my target
	 */
	inline bool is_my_target(uint8_t sysid, uint8_t compid) {
		return sysid == get_tgt_system() && compid == get_tgt_component();
	}

	/**
	 * @brief Check that system id is my target
	 */
	inline bool is_my_target(uint8_t sysid) {
		return sysid == get_tgt_system();
	}

	/**
	 * @brief Check that FCU is APM
	 */
	inline bool is_ardupilotmega() {
		return MAV_AUTOPILOT::ARDUPILOTMEGA == get_autopilot();
	}

	/**
	 * @brief Check that FCU is PX4
	 */
	inline bool is_px4() {
		return MAV_AUTOPILOT::PX4 == get_autopilot();
	}

	/**
	 * @brief Represent FCU mode as string
	 *
	 * Port pymavlink mavutil.mode_string_v10
	 *
	 * Supported FCU's:
	 * - APM:Plane
	 * - APM:Copter
	 * - PX4
	 *
	 * @param[in] base_mode    base mode
	 * @param[in] custom_mode  custom mode data
	 */
	std::string str_mode_v10(uint8_t base_mode, uint32_t custom_mode);

	/**
	 * @brief Lookup custom mode for given string
	 *
	 * Complimentary to @a str_mode_v10()
	 *
	 * @param[in]  cmode_str   string representation of mode
	 * @param[out] custom_mode decoded value
	 * @return true if success
	 */
	bool cmode_from_str(std::string cmode_str, uint32_t &custom_mode);

	inline void set_base_link_frame_id(const std::string frame_id) {
		base_link_frame_id = frame_id;
	}
	inline void set_odom_frame_id(const std::string frame_id) {
		odom_frame_id = frame_id;
	}
	inline void set_map_frame_id(const std::string frame_id) {
		map_frame_id = frame_id;
	}
	inline std::string get_base_link_frame_id() {
		return base_link_frame_id;
	}
	inline std::string get_odom_frame_id() {
		return odom_frame_id;
	}
	inline std::string get_map_frame_id() {
		return map_frame_id;
	}

	/**
	 * @brief 批量发布之前缓存的所有静态坐标变换。
	 */
	void setup_static_tf();

private:
	std::recursive_mutex mutex;

	std::atomic<uint8_t> type;
	std::atomic<uint8_t> autopilot;
	std::atomic<uint8_t> base_mode;

	uint8_t target_system;
	uint8_t target_component;

	std::atomic<bool> connected;
	std::vector<ConnectionCb> connection_cb_vec;
	std::vector<CapabilitiesCb> capabilities_cb_vec;

	sensor_msgs::Imu::Ptr imu_enu_data;					// IMU 数据缓存（存储 ENU 坐标系的 IMU 数据）
	sensor_msgs::Imu::Ptr imu_ned_data;					// IMU 数据缓存（存储 NED 坐标系的 IMU 数据）

	sensor_msgs::NavSatFix::Ptr gps_fix;				// GPS 定位数据缓存（WGS84 坐标系的经纬度、高度）
	float gps_eph;
	float gps_epv;
	int gps_fix_type;
	int gps_satellites_visible;

	std::atomic<uint64_t> time_offset;					// 飞控与 ROS 的时间偏移（纳秒，用于同步飞控时间与 ROS 时间）
	timesync_mode tsync_mode;

	std::atomic<bool> fcu_caps_known;
	std::atomic<uint64_t> fcu_capabilities;				// 飞控支持的能力掩码（如是否支持 MAVLink 2.0、是否有 GPS）

	std::string base_link_frame_id, odom_frame_id, map_frame_id;
};
}	// namespace mavros
