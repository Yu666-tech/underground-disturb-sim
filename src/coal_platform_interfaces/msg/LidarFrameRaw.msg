coal_platform_interfaces/RunContext context # frame_id=laser_link；其余空间信息通过同戳TF到map
string observation_id # 一帧一个ID；逐束用beam_index关联
string sensor_id # 激光传感器ID
float32 angle_min_rad # 首束角度rad，0朝传感器+X，正角向+Y
float32 angle_max_rad # 末束角度rad
float32 angle_increment_rad # 相邻束rad，N>1时为(max-min)/(N-1)
float32 time_increment_s # 束间采样时间s；当前Gazebo基线0
float32 scan_time_s # 帧间标称时间s，当前0.1
float32 range_min_m # m
float32 range_max_m # m
uint32 beam_count # 当前720；只支持二维单层
float32[] truth_ranges_m # N项，理想几何扫描；当前直接来自零噪声Gazebo raw，非解析单姿态重算
float32[] raw_ranges_m # N项，无扰动输入；当前与truth_ranges一致
float32[] intensities # 空数组或N项，未知不编造强度
bool[] raw_valid # N项；有限且在量程内才true
uint8[] raw_status # N项：0 HIT，1 NO_RETURN，2 INPUT_INVALID，3 BELOW_MIN
