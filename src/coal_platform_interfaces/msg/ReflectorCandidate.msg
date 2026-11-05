string geometry_id # 候选几何ID，不表示反射路径成立
string geometry_type # BOX/MESH
geometry_msgs/Point closest_point_m # 几何表面上距请求线段最近点，map
float64 distance_to_link_m # 最近点到有限线段的距离m
geometry_msgs/Vector3 normal # 最近表面单位法向，map
string material # 材料
string reflection_level # LOW/MEDIUM/HIGH/UNKNOWN
bool is_dynamic # 动态标记
