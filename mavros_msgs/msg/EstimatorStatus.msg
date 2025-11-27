# Current autopilot estimator state
# 估计器状态消息
# 发布飞控状态估计器（如 EKF/UKF）的工作状态（滤波收敛性、传感器权重、误差范围），判断定位是否可靠
#
# https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS_FLAGS

std_msgs/Header header
bool attitude_status_flag

bool velocity_horiz_status_flag
bool velocity_vert_status_flag

bool pos_horiz_rel_status_flag
bool pos_horiz_abs_status_flag

bool pos_vert_abs_status_flag
bool pos_vert_agl_status_flag

bool const_pos_mode_status_flag

bool pred_pos_horiz_rel_status_flag
bool pred_pos_horiz_abs_status_flag

bool gps_glitch_status_flag
bool accel_error_status_flag