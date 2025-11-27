/**
 * @brief Frame transformation utilities
 * @file frame_tf.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Eddy Scott <scott.edward@aurora.aero>
 * @author Nuno Marques <n.marques21@hotmail.com>
 *
 * @addtogroup nodelib
 * @{
 */
/*
 * Copyright 2016,2017 Vladimir Ermakov.
 * Copyright 2017,2018 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 * 专注于坐标系变换与数据格式转换，是无人机导航与控制中 “空间数据标准化” 的关键工具。
 */

#pragma once

#include <array>
#include <Eigen/Eigen> 								// Eigen 库（线性代数与几何计算核心库），用于矩阵、向量、四元数的运算（无人机坐标系变换的数学基础）。
#include <Eigen/Geometry>
#include <ros/assert.h>								// ROS 的断言宏（如 ROS_ASSERT_MSG），用于调试时检查参数合法性（如协方差矩阵大小是否匹配）。

// for Covariance types								// 引入与 “空间数据” 相关的 ROS 消息类型，用于后续定义 “与 ROS 消息匹配的数据类型”
#include <sensor_msgs/Imu.h>						// IMU 消息（包含角速度协方差，用于定义 3x3 协方差类型）。
#include <geometry_msgs/Point.h>					// 几何消息（点、向量、四元数、带协方差的位姿，用于定义坐标、姿态与协方差类型）。
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace mavros {
namespace ftf {
// 协方差矩阵用于描述数据的不确定性（如位置误差、姿态误差），ROS 消息中协方差以 “一维数组” 存储（行优先），此处定义与 ROS 消息对应的类型别名，方便后续操作：

//! Type matching rosmsg for 3x3 covariance matrix
// 3x3 协方差矩阵类型，匹配 sensor_msgs/Imu 消息中 “角速度协方差” 的类型（_angular_velocity_covariance_type 本质是 boost::array<double, 9>，9 个元素对应 3x3 矩阵的行优先存储）。
// 用途：描述 3 维数据的不确定性（如角速度误差、线速度误差）。
using Covariance3d = sensor_msgs::Imu::_angular_velocity_covariance_type;

//! Type matching rosmsg for 6x6 covariance matrix
// 6x6 协方差矩阵类型，匹配 geometry_msgs/PoseWithCovariance 消息中 “位姿协方差” 的类型（_covariance_type 是 boost::array<double, 36>，36 个元素对应 6x6 矩阵）。
// 用途：描述 “位置（3 维）+ 姿态（3 维）” 的联合不确定性。
using Covariance6d = geometry_msgs::PoseWithCovariance::_covariance_type;

//! Type matching rosmsg for 9x9 covariance matrix
// 9x9 协方差矩阵类型，自定义为 boost::array<double, 81>（81=9x9）。
// 用途：描述更高维度数据的不确定性（如 “位置 + 速度 + 姿态” 的联合误差）。
using Covariance9d = boost::array<double, 81>;

/*          Eigen 库操作矩阵时需 “Eigen 矩阵类型”，但 ROS 协方差是 “一维数组”，此处通过 Eigen::Map 将 “一维数组” 映射为 “Eigen 矩阵”（零拷贝，直接操作原数据），避免数据拷贝的开销              */

//! Eigen::Map for Covariance3d
// Eigen::Map：Eigen 的映射类，将 “外部内存中的数据” 视为 Eigen 矩阵 / 向量。
// Eigen::RowMajor：指定矩阵按 “行优先” 存储（与 ROS 消息的协方差存储格式一致，避免数据错乱）。
using EigenMapCovariance3d = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;
using EigenMapConstCovariance3d = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >;

//! Eigen::Map for Covariance6d
using EigenMapCovariance6d = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;
using EigenMapConstCovariance6d = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >;

//! Eigen::Map for Covariance9d
using EigenMapCovariance9d = Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor> >;
using EigenMapConstCovariance9d = Eigen::Map<const Eigen::Matrix<double, 9, 9, Eigen::RowMajor> >;

/**
 * @brief Orientation transform options when applying rotations to data
 * 			定义枚举类型，明确 “从哪个坐标系变换到哪个坐标系”，避免函数参数歧义（如 “NED→ENU” 和 “ENU→NED” 是完全相反的变换）。
 * 
 *		Static Transformation 的缩写，代表 “固定坐标系间的变换”（变换关系不随时间变化）。
 */
enum class StaticTF {
	/* NED_TO_ENU：从 NED 坐标系（北 - 东 - 下，无人机常用惯性系）变换到 ENU 坐标系（东 - 北 - 上，ROS 标准惯性系）。 */
	NED_TO_ENU,		//!< will change orientation from being expressed WRT NED frame to WRT ENU frame
	ENU_TO_NED,		//!< change from expressed WRT ENU frame to WRT NED frame
	/* AIRCRAFT_TO_BASELINK：从 机体坐标系（Aircraft）（原点在机身重心，X 轴向前）变换到 基准坐标系（Baselink）（原点在机器人底座，X 轴向前，常用于多传感器校准）。 */
	AIRCRAFT_TO_BASELINK,	//!< change from expressed WRT aircraft frame to WRT to baselink frame
	BASELINK_TO_AIRCRAFT,	//!< change from expressed WRT baselnk to WRT aircraft
	/* ABSOLUTE_FRAME_*：“绝对帧下的变换”—— 此处 “绝对帧” 指局部惯性系（如 NED/ENU），变换时需考虑姿态在绝对帧中的参考（而非仅坐标系间的相对旋转）。 */
	ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK,//!< change orientation from being expressed in aircraft frame to baselink frame in an absolute frame of reference.
	ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT,//!< change orientation from being expressed in baselink frame to aircraft frame in an absolute frame of reference
};

/**
 * @brief Orientation transform options when applying rotations to data, for ECEF.
 * 			枚举值用途：实现 ECEF 与 ENU 坐标系的转换（如将 GPS 输出的 ECEF 坐标转换为 ROS 常用的 ENU 坐标）
 */
enum class StaticEcefTF {
	/* ECEF：Earth-Centered, Earth-Fixed（地心地固坐标系），是全球统一的绝对坐标系（原点在地球质心，X 轴指向本初子午线与赤道交点，Z 轴指向北极）。 */
	ECEF_TO_ENU,		//!< change from expressed WRT ECEF frame to WRT ENU frame
	ENU_TO_ECEF		//!< change from expressed WRT ENU frame to WRT ECEF frame
};

namespace detail {
/**
 * @brief Transform representation of attitude from 1 frame to another
 * (e.g. transfrom attitude from representing  from base_link -> NED
 *               to representing base_link -> ENU)
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 * 姿态（四元数）的坐标系变换（如将 “相对于 NED 的姿态” 转换为 “相对于 ENU 的姿态”）。
 */
Eigen::Quaterniond transform_orientation(const Eigen::Quaterniond &q, const StaticTF transform);

/**
 * @brief Transform data expressed in one frame to another frame.
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 * 向量（如位置、速度）的坐标系变换 —— 通过四元数 q（源坐标系到目标坐标系的旋转）实现向量旋转。
 */
Eigen::Vector3d transform_frame(const Eigen::Vector3d &vec, const Eigen::Quaterniond &q);

/**
 * @brief Transform 3x3 convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 * 协方差矩阵的变换需遵循 “误差传播公式”（cov_new = q * cov_old * q^T，q^T 是四元数的共轭），此处封装该逻辑。
 */
Covariance3d transform_frame(const Covariance3d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform 6x6 convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance6d transform_frame(const Covariance6d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform 9x9 convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance9d transform_frame(const Covariance9d &cov, const Eigen::Quaterniond &q);

/**
 * @brief Transform data expressed in one frame to another frame.
 *
 * General function. Please use specialized variants.
 */
Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const StaticTF transform);

/**
 * @brief Transform 3d convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance3d transform_static_frame(const Covariance3d &cov, const StaticTF transform);

/**
 * @brief Transform 6d convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance6d transform_static_frame(const Covariance6d &cov, const StaticTF transform);

/**
 * @brief Transform 9d convariance expressed in one frame to another
 *
 * General function. Please use specialized enu-ned and ned-enu variants.
 */
Covariance9d transform_static_frame(const Covariance9d &cov, const StaticTF transform);

/**
 * @brief Transform data expressed in one frame to another frame
 * with additional map origin parameter.
 *
 * General function. Please use specialized variants.
 */
Eigen::Vector3d transform_static_frame(const Eigen::Vector3d &vec, const Eigen::Vector3d &map_origin, const StaticEcefTF transform);

}	// namespace detail

// -*- frame tf -*-

/**
 * @brief Transform from attitude represented WRT NED frame to attitude
 *		  represented WRT ENU frame
 * 		  姿态变换接口（Orientation Transform）
 * 	模板参数 T：支持 Eigen::Quaterniond（Eigen 四元数）、geometry_msgs::Quaternion（ROS 四元数）等多种姿态类型，提高函数通用性。
 * 	inline：内联函数，避免函数调用的开销（坐标系变换是高频操作，如 IMU 数据每毫秒需变换一次）。
 * 	用户调用 transform_orientation_ned_enu(q) 即可直接将 “NED 系下的姿态” 转为 “ENU 系下的姿态”，无需关心内部枚举传递。
 */
template<class T>
inline T transform_orientation_ned_enu(const T &in) {
	return detail::transform_orientation(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform from attitude represented WRT ENU frame to
 *		  attitude represented WRT NED frame
 */
template<class T>
inline T transform_orientation_enu_ned(const T &in) {
	return detail::transform_orientation(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform from attitude represented WRT aircraft frame to
 *		  attitude represented WRT base_link frame
 */
template<class T>
inline T transform_orientation_aircraft_baselink(const T &in) {
	return detail::transform_orientation(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform from attitude represented WRT baselink frame to
 *		  attitude represented WRT body frame
 */
template<class T>
inline T transform_orientation_baselink_aircraft(const T &in) {
	return detail::transform_orientation(in, StaticTF::BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform from attitude represented WRT aircraft frame to
 *		  attitude represented WRT base_link frame, treating aircraft frame 
 *		  as in an absolute frame of reference (local NED).
 */
template<class T>
inline T transform_orientation_absolute_frame_aircraft_baselink(const T &in) {
	return detail::transform_orientation(in, StaticTF::ABSOLUTE_FRAME_AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform from attitude represented WRT baselink frame to
 *		  attitude represented WRT body frame, treating baselink frame 
 *		  as in an absolute frame of reference (local NED).
 */
template<class T>
inline T transform_orientation_absolute_frame_baselink_aircraft(const T &in) {
	return detail::transform_orientation(in, StaticTF::ABSOLUTE_FRAME_BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform data expressed in NED to ENU frame.
 * 		  向量 / 坐标变换接口（Frame Transform）
 * 
 * 支持的数据类型 T：Eigen::Vector3d（Eigen 向量）、geometry_msgs::Point（ROS 点）、geometry_msgs::Vector3（ROS 向量）等。
 * ECEF↔ENU 接口：需额外传入 map_origin（ENU 系的原点，以经纬度海拔表示），因为 ENU 是 “局部坐标系”（原点在某个地理位置），ECEF 是 “全局坐标系”，转换需知道局部原点的 ECEF 坐标。
 * 
 * 场景：当两个坐标系的变换关系随时间变化（如无人机飞行时，机体坐标系相对于 NED 系的姿态不断变化），需传入 “实时姿态四元数 q”（源坐标系到目标坐标系的旋转）。
 * 假设条件：注释明确 “q 代表源坐标系到目标坐标系的旋转”（如 q 是 aircraft→NED 的旋转，则函数将 aircraft 系的向量转为 NED 系），避免用户传入错误的四元数。
 */
template<class T>
inline T transform_frame_ned_enu(const T &in) {
	return detail::transform_static_frame(in, StaticTF::NED_TO_ENU);
}

/**
 * @brief Transform data expressed in ENU to NED frame.
 *
 */
template<class T>
inline T transform_frame_enu_ned(const T &in) {
	return detail::transform_static_frame(in, StaticTF::ENU_TO_NED);
}

/**
 * @brief Transform data expressed in Aircraft frame to Baselink frame.
 *		   基于姿态的动态变换接口
 *	
 *	当两个坐标系的变换关系随时间变化（如无人机飞行时，机体坐标系相对于 NED 系的姿态不断变化），需传入 “实时姿态四元数 q”（源坐标系到目标坐标系的旋转）。
 *	
 */
template<class T>
inline T transform_frame_aircraft_baselink(const T &in) {
	return detail::transform_static_frame(in, StaticTF::AIRCRAFT_TO_BASELINK);
}

/**
 * @brief Transform data expressed in Baselink frame to Aircraft frame.
 *
 */
template<class T>
inline T transform_frame_baselink_aircraft(const T &in) {
	return detail::transform_static_frame(in, StaticTF::BASELINK_TO_AIRCRAFT);
}

/**
 * @brief Transform data expressed in ECEF frame to ENU frame.
 *
 * @param in          local ECEF coordinates [m]
 * @param map_origin  geodetic origin [lla]
 * @returns local ENU coordinates [m].
 */
template<class T>
inline T transform_frame_ecef_enu(const T &in, const T &map_origin) {
	return detail::transform_static_frame(in, map_origin, StaticEcefTF::ECEF_TO_ENU);
}

/**
 * @brief Transform data expressed in ENU frame to ECEF frame.
 *
 * @param in          local ENU coordinates [m]
 * @param map_origin  geodetic origin [lla]
 * @returns local ECEF coordinates [m].
 */
template<class T>
inline T transform_frame_enu_ecef(const T &in, const T &map_origin) {
	return detail::transform_static_frame(in, map_origin, StaticEcefTF::ENU_TO_ECEF);
}

/**
 * @brief Transform data expressed in aircraft frame to NED frame.
 * Assumes quaternion represents rotation from aircraft frame to NED frame.
 */
template<class T>
inline T transform_frame_aircraft_ned(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in NED to aircraft frame.
 * Assumes quaternion represents rotation from NED to aircraft frame.
 */
template<class T>
inline T transform_frame_ned_aircraft(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in aircraft frame to ENU frame.
 * Assumes quaternion represents rotation from aircraft frame to ENU frame.
 */
template<class T>
inline T transform_frame_aircraft_enu(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to aircraft frame.
 * Assumes quaternion represents rotation from ENU to aircraft frame.
 */
template<class T>
inline T transform_frame_enu_aircraft(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in ENU to base_link frame.
 * Assumes quaternion represents rotation from ENU to base_link frame.
 */
template<class T>
inline T transform_frame_enu_baselink(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

/**
 * @brief Transform data expressed in baselink to ENU frame.
 * Assumes quaternion represents rotation from basel_link to ENU frame.
 */
template<class T>
inline T transform_frame_baselink_enu(const T &in, const Eigen::Quaterniond &q) {
	return detail::transform_frame(in, q);
}

// -*- utils -*-
// 封装 “姿态格式转换”“MAVLink 数据格式转换” 等常用工具，解决 “不同框架间数据格式不兼容” 的问题。

/**
 * @brief Convert euler angles to quaternion.
 */
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @return quaternion, same as @a tf::quaternionFromRPY() but in Eigen format.
 */
inline Eigen::Quaterniond quaternion_from_rpy(const double roll, const double pitch, const double yaw) {
	return quaternion_from_rpy(Eigen::Vector3d(roll, pitch, yaw));
}

/**
 * @brief Convert quaternion to euler angles
 *
 * Reverse operation to @a quaternion_from_rpy()
 */
Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond &q);

/**
 * @brief Convert quaternion to euler angles
 */
inline void quaternion_to_rpy(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw)
{
	const auto rpy = quaternion_to_rpy(q);
	roll = rpy.x();
	pitch = rpy.y();
	yaw = rpy.z();
}

/**
 * @brief Get Yaw angle from quaternion
 *
 * Replacement function for @a tf::getYaw()
 */
double quaternion_get_yaw(const Eigen::Quaterniond &q);

/**
 * @brief Store Quaternion to MAVLink float[4] format
 *
 * MAVLink uses wxyz order, wile Eigen::Quaterniond uses xyzw internal order,
 * so it can't be stored to array using Eigen::Map.
 */
template <typename _Scalar, typename std::enable_if<std::is_floating_point<_Scalar>::value, bool>::type = true>
inline void quaternion_to_mavlink(const Eigen::Quaternion<_Scalar> &q, std::array<float, 4> &qmsg)
{
	qmsg[0] = q.w();
	qmsg[1] = q.x();
	qmsg[2] = q.y();
	qmsg[3] = q.z();
}

/**
 * @brief Convert Mavlink float[4] quaternion to Eigen
 */
inline Eigen::Quaterniond mavlink_to_quaternion(const std::array<float, 4> &q)
{
	return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}

/**
 * @brief Convert covariance matrix to MAVLink float[n] format
 */
template<class T, std::size_t SIZE>
inline void covariance_to_mavlink(const T &cov, std::array<float, SIZE> &covmsg)
{
	std::copy(cov.cbegin(), cov.cend(), covmsg.begin());
}

/**
 * @brief Convert upper right triangular of a covariance matrix to MAVLink float[n] format
 */
template<class T, std::size_t ARR_SIZE>
inline void covariance_urt_to_mavlink(const T &covmap, std::array<float, ARR_SIZE> &covmsg)
{
	auto m = covmap;
	std::size_t COV_SIZE = m.rows() * (m.rows() + 1) / 2;
	ROS_ASSERT_MSG(COV_SIZE == ARR_SIZE,
				"frame_tf: covariance matrix URT size (%lu) is different from Mavlink msg covariance field size (%lu)",
				COV_SIZE, ARR_SIZE);

	auto out = covmsg.begin();

	for (size_t x = 0; x < m.cols(); x++) {
		for (size_t y = x; y < m.rows(); y++)
			*out++ = m(y, x);
	}
}

/**
 * @brief Convert MAVLink float[n] format (upper right triangular of a covariance matrix)
 * to Eigen::MatrixXd<n,n> full covariance matrix
 */
template<class T, std::size_t ARR_SIZE>
inline void mavlink_urt_to_covariance_matrix(const std::array<float, ARR_SIZE> &covmsg, T &covmat)
{
	std::size_t COV_SIZE = covmat.rows() * (covmat.rows() + 1) / 2;
	ROS_ASSERT_MSG(COV_SIZE == ARR_SIZE,
				"frame_tf: covariance matrix URT size (%lu) is different from Mavlink msg covariance field size (%lu)",
				COV_SIZE, ARR_SIZE);

	auto in = covmsg.begin();

	for (size_t x = 0; x < covmat.cols(); x++) {
		for (size_t y = x; y < covmat.rows(); y++) {
			covmat(x, y) = static_cast<double>(*in++);
			covmat(y, x) = covmat(x, y);
		}
	}
}

// [[[cog:
// def make_to_eigen(te, tr, fields):
//     cog.outl("""//! @brief Helper to convert common ROS geometry_msgs::{tr} to Eigen::{te}""".format(**locals()))
//     cog.outl("""inline Eigen::{te} to_eigen(const geometry_msgs::{tr} r) {{""".format(**locals()))
//     cog.outl("""\treturn Eigen::{te}({fl});""".format(te=te, fl=", ".join(["r." + f for f in fields])))
//     cog.outl("""}""")
//
// make_to_eigen("Vector3d", "Point", "xyz")
// make_to_eigen("Vector3d", "Vector3", "xyz")
// make_to_eigen("Quaterniond", "Quaternion", "wxyz")
// ]]]
//! @brief Helper to convert common ROS geometry_msgs::Point to Eigen::Vector3d
inline Eigen::Vector3d to_eigen(const geometry_msgs::Point r) {
	return Eigen::Vector3d(r.x, r.y, r.z);
}
//! @brief Helper to convert common ROS geometry_msgs::Vector3 to Eigen::Vector3d
inline Eigen::Vector3d to_eigen(const geometry_msgs::Vector3 r) {
	return Eigen::Vector3d(r.x, r.y, r.z);
}
//! @brief Helper to convert common ROS geometry_msgs::Quaternion to Eigen::Quaterniond
inline Eigen::Quaterniond to_eigen(const geometry_msgs::Quaternion r) {
	return Eigen::Quaterniond(r.w, r.x, r.y, r.z);
}
// [[[end]]] (checksum: 1b3ada1c4245d4e31dcae9768779b952)
}	// namespace ftf
}	// namespace mavros
