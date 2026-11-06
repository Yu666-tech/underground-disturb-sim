coal_platform_interfaces/RunContext context # 原raw上下文
string observation_id # 原帧ID
string decision_id # 对应Decision
string sensor_id # 原传感器ID
float32 angle_min_rad # 原值rad
float32 angle_max_rad # 原值rad
float32 angle_increment_rad # 原值rad
float32 time_increment_s # 原值s
float32 scan_time_s # 原值s
float32 range_min_m # 原值m
float32 range_max_m # 原值m
uint32 beam_count # 原值N
float32[] observed_ranges_m # N项；无返回或丢点+Inf，输入损坏NaN
float32[] intensities # 空或N项
float32[] truth_ranges_m
