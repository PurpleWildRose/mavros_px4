# VIBRATION message data
# 振动消息
# 振动消息	发布无人机机身振动数据（X/Y/Z 轴振动加速度、振动频率），用于判断电机不平衡、螺旋桨损坏等问题
# @description: Vibration levels and accelerometer clipping

std_msgs/Header header

geometry_msgs/Vector3 vibration		# 3-axis vibration levels
float32[3] clipping				# Accelerometers clipping