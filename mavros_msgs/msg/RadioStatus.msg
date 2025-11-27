# RADIO_STATUS message
# 遥控器状态消息
# 发布遥控器（RC）的连接质量（信号强度、丢包率、遥控器电池电压），是遥控器断连报警的关键数据

std_msgs/Header header

# message data
uint8 rssi
uint8 remrssi
uint8 txbuf
uint8 noise
uint8 remnoise
uint16 rxerrors
uint16 fixed

# calculated
float32 rssi_dbm
float32 remrssi_dbm
