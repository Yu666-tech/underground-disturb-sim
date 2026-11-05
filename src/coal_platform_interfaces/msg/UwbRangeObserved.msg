coal_platform_interfaces/RunContext context # 原raw上下文，保持采样时间
string observation_id # 原raw观测ID
string decision_id # 对应Decision ID
string tag_id # 原标签ID
string anchor_id # 原锚点ID
geometry_msgs/Point tag_position_m # 原raw位置，map
geometry_msgs/Point anchor_position_m # 原raw位置，map
float64 truth_range_m # 原真值m
float64 raw_range_m # 原raw m
float64 observed_range_m # 注入后m；无效NaN
float64 error_m # observed-truth；无效NaN
string state # 与对应decision propagation_state一致
bool valid # 有效观测才true
