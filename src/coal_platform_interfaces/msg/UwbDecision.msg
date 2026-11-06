coal_platform_interfaces/RunContext context # 继承raw，frame_id=map
string observation_id # 原raw观测ID
string decision_id # run内唯一
string params_version # B配置版本
uint64 seed # 本run配置种子；逐观测随机流由ID确定
string query_status # OK/INVALID_REQUEST/SCENE_MISMATCH/VERSION_MISMATCH/TIME_UNAVAILABLE/UNSUPPORTED/INTERNAL_ERROR/NOT_CALLED
string propagation_state # LOS/NLOS/OUTAGE/INPUT_INVALID/UNKNOWN
string cause # 可读原因；不依赖其文本做状态解析
uint32 trigger_mask # bit0 raw_invalid bit1 out_of_range bit2 direct_blocked bit3 multipath_active bit4 query_failed
bool multipath_active # 多径分量实际启用
bool nlos_active # NLOS分量实际启用
bool outage_active # 无有效观测
uint32 blocking_count # 查询直达阻挡体数量
uint32 reflector_count # 几何反射候选数量
float64 multipath_error_m # 本次叠加多径误差m；未启用为0
float64 nlos_error_m # 本次叠加NLOS误差m；未启用为0
float64 random_noise_m # 本次随机噪声m；未启用为0
bool valid # 本次判定是否可用；查询失败不默认为LOS
string anchor_id
