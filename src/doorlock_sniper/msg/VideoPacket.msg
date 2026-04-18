# 流式视频数据包 - 固定150字节
uint64 sequence_id      # 全局包序号
uint64 timestamp_ns     # ROS2 时间戳
uint8[150] data         # 固定150字节负载（最后一包不足则补零）