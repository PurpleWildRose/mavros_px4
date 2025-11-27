#Follows https://mavlink.io/en/messages/common.html#CELLULAR_STATUS specification
# 蜂窝网络状态消息
# 若无人机使用 4G/5G 模块，发布蜂窝网络连接状态（信号强度、网络类型、IP 地址）

uint8 status
uint8 failure_reason
uint8 type
uint8 quality
uint16 mcc
uint16 mnc
uint16 lac