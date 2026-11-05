coal_platform_interfaces/RunContext context # frame_id=map，与truth同采样时刻
string work_mode # IDLE/TRAVELLING/CUTTING/UNKNOWN
string zone_id # 当前区域；无区时NONE，按priority最大规则唯一选区
bool is_moving # 实测线速度模长>0.001 m/s
float64 commanded_linear_speed_mps # 控制目标有符号机体前向速度m/s；非前向轨迹见下项
geometry_msgs/Vector3 commanded_velocity_map_mps # map目标速度向量，m/s；采煤机沿Y运动用此项
float64 commanded_yaw_rate_radps # 绕map Z的目标角速度rad/s
