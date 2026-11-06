coal_platform_interfaces/RunContext context # 原raw上下文
string observation_id # 原帧ID
string decision_id # 对应Decision
uint8[] beam_states # N项：0 NORMAL 1 DISTURBED 2 DROPOUT 3 FALSE_RETURN 4 NO_RETURN 5 INPUT_INVALID
float32[] delta_range_m # observed-raw；任一值非有限时NaN
bool[] observed_valid # N项；有限且量程内为true
uint32[] beam_index # N explicit indices corresponding to beam_states
