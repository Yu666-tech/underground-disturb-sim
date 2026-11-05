coal_platform_interfaces/RunContext context # frame_id=map
string observation_id # 唯一真值采样ID：run_id/robot_id/ground_truth/sequence
string child_frame_id # base_link
geometry_msgs/Pose pose # map 中机体基准位姿，m与单位四元数xyzw
geometry_msgs/Twist twist # 线速度m/s、角速度rad/s，均在map表达
float64 roll_rad # 与pose一致的ZYX欧拉分解roll
float64 pitch_rad # 与pose一致的ZYX欧拉分解pitch
float64 yaw_rad # 与pose一致的ZYX欧拉分解yaw
