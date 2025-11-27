# STATUSTEXT message representation
# 状态文本消息
# 飞控或机载系统的文本提示（如 “GPS 信号弱”“电池低电量”“校准成功”），用于用户交互或日志记录 
#
# https://mavlink.io/en/messages/common.html#STATUSTEXT

# Severity levels
uint8 EMERGENCY = 0
uint8 ALERT = 1
uint8 CRITICAL = 2
uint8 ERROR = 3
uint8 WARNING = 4
uint8 NOTICE = 5
uint8 INFO = 6
uint8 DEBUG = 7

# Fields
std_msgs/Header header
uint8 severity
string text
