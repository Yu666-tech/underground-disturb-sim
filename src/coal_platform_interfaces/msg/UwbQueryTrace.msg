# Actual query attempts, paired with the resulting UWB decision; no synthetic query for invalid raw.
coal_platform_interfaces/RunContext context
string observation_id
string decision_id
string request_id
string tag_id
string anchor_id
string nlos_type # none/soft/hard; none is not proof of LOS when valid=false
bool valid # Corresponding decision validity, not just query success
string status # QueryRay status; transport exceptions use INTERNAL_ERROR
coal_platform_interfaces/Intersection[] direct_intersections
coal_platform_interfaces/ReflectorCandidate[] reflector_candidates
float64 processing_time_ms
string error_message
