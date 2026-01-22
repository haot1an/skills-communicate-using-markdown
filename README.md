flowchart TB

%% ========== 入口 ==========
START([收到 station_id]) --> LOCK{callback_running_ 为 true ?}
LOCK -- 是 --> IGN([忽略本次触发])
LOCK -- 否 --> SETLOCK[callback_running_ 置 true]

%% ========== 命令路由 ==========
SETLOCK --> ROUTE{station_id 是否为特殊命令 ?}
ROUTE -- 是 --> CMD[执行测试/复位/状态命令\nfetch_bag reset status test_detect 等]
CMD --> UNLOCK_END[callback_running_ 置 false 结束]
ROUTE -- 否 --> FIRST{首次回调 is_first_callback_ ?}
FIRST -- 是 --> PREP[先执行 fetchBagTask 取袋预备]
FIRST -- 否 --> GO

PREP --> GO[进入一次 Goal 作业流程]

%% ========== 扫描建图 ==========
GO --> OBS1[到观测位1 获取点云]
OBS1 --> OBS2[到观测位2 获取点云]
OBS2 --> OBS3[到观测位3 获取点云]
OBS3 --> FUSE[TF 到 base_link 融合 + 下采样]
FUSE --> PUBCLOUD[发布 fused_cloud 更新 OctoMap/MoveIt]
PUBCLOUD --> WAITMAP([等待地图生效])

%% ========== 目标获取 ==========
WAITMAP --> MODE{是否 test_mode ?}

MODE -- 是 --> TESTSUB
MODE -- 否 --> FILESUB

subgraph TESTSUB[测试模式：视觉点]
direction TB
T1[到测试观测位] --> T2[调用 peach_detect_position_service]
T2 --> T3[相机系点 TF 到 base_link]
T3 --> T4[工作空间过滤]
end

subgraph FILESUB[文件模式：waypoints]
direction TB
F1[读取 ur_waypoints.txt 解析] --> F2[按 station_id 选 goal_id]
F2 --> F3[map TF 到 base_link]
F3 --> F4[工作空间过滤]
end

TESTSUB --> TARGETS[得到 all_targets]
FILESUB --> TARGETS

%% ========== 排序与可视化 ==========
TARGETS --> SORT[按 z_base 升序排序]
SORT --> MARK[发布 peach_markers 可视化]

%% ========== 循环处理每个桃子 ==========
MARK --> LOOP_START([遍历每个 target])

subgraph PER[单个桃子：粗定位 -> 精定位 -> 套袋]
direction TB

P0[冻结观测时刻 TF] --> P1[计算粗定位相机目标位姿]
P1 --> P2[粗定位：环采样候选 + IK 过滤]
P2 --> P3{MTC 粗定位成功 ?}
P3 -- 是 --> P4[粗定位完成]
P3 -- 否 --> P3b[回退：MoveIt 直规划到粗定位 pose]
P3b --> P3c{MoveIt 成功 ?}
P3c -- 否 --> PSKIP([跳过该桃子])
P3c -- 是 --> P4

P4 --> Q1[精定位：调用姿态服务 得到 position+quaternion]
Q1 --> Q2[相机系 pose TF 到 base_link pose]
Q2 --> Q3[精定位：环采样候选 + IK 过滤]
Q3 --> Q4[unwrap 关节角 防止跨 2pi 翻转]
Q4 --> Q5[按位姿距离重排候选]
Q5 --> Q6[发布 mtc_axes 可视化]
Q6 --> Q7{MTC 精定位成功 ?}
Q7 -- 否 --> PSKIP
Q7 -- 是 --> BAG0[套袋动作开始]

BAG0 --> BAG1[bagging 前进\n优先 CartesianPath\n失败回退 PoseTarget]
BAG1 --> BAG2[IO 触发套袋 DOUT1 脉冲]
BAG2 --> BAG3[bagging 后退]
BAG3 --> CNT[bag_count 加 1]
CNT --> THR{达到补袋阈值 ?}
THR -- 是 --> GETBAG[fetchBagTask 补袋]
GETBAG --> OK{补袋成功 ?}
OK -- 是 --> CLR[bag_count 清零]
OK -- 否 --> KEEP[保持计数]
THR -- 否 --> DONE([单桃子完成])
CLR --> DONE
KEEP --> DONE
end

LOOP_START --> PER --> LOOP_NEXT([下一个 target 或结束循环])

%% ========== 收尾 ==========
LOOP_NEXT --> CLEAR[调用 clear_octomap 清图]
CLEAR --> CARNOTI{是否启用 car_done ?}
CARNOTI -- 是 --> DONECAR[publishUrDone\n到行使位 + 发布 ur_done]
CARNOTI -- 否 --> ENDONLY([结束不通知车体])

DONECAR --> UNLOCK[callback_running_ 置 false]
ENDONLY --> UNLOCK
UNLOCK --> END([任务结束])
