coal_platform_interfaces/SimHeader header # stamp=采样时刻，frame_id 见各消息约定
string schema_version # 协议语义版本，本包 0.2.0
string robot_id # 机器人标识；查询场景级请求也填写发起机器人
uint64 sequence # 每生产者每数据流每run递增，从0开始
