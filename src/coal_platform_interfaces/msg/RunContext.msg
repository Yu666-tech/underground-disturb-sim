std_msgs/Header header # stamp=采样时刻，frame_id 见各消息约定
string schema_version # 协议语义版本，本包 0.1.1
string experiment_id # 实验配置标识，非空
string run_id # 每次运行唯一；时钟重置必须换新值
string scene_id # 场景包标识，大小写敏感
string scene_version # 场景内容版本，SemVer
string robot_id # 机器人标识；查询场景级请求也填写发起机器人
uint64 sequence # 每生产者每数据流每run递增，从0开始
