coal_platform_interfaces/RunContext context # 继承raw，frame_id=laser_link
string observation_id # 原帧ID
string decision_id # run内唯一
string params_version # C配置版本
uint64 seed # run种子
string work_mode # 消费的RobotState工况
string zone_id # 消费的RobotState区域
float64 target_concentration_mg_m3 # mg/m^3
float64 actual_concentration_mg_m3 # mg/m^3
bool dust_active # C状态机输出
string cause # 可读原因
uint32 normal_count # 状态0数量
uint32 disturbed_count # 状态1数量
uint32 dropout_count # 状态2数量
uint32 false_return_count # 状态3数量
uint32 no_return_count # 状态4数量
uint32 invalid_count # 状态5数量；六项总和等于N
bool valid # 判定是否可用
string sensor_id
uint32 total_beams
