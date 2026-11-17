# 运行记录与复算 — v0.7.6

运行 `.venv/Scripts/python.exe scripts/verify_industrial_scenes.py`，按 seed=42、2 ms 物理步
执行四个内置工位，每个最多 50,000 步，目标满足后在 500 步批次边界停止。
输出 `artifacts/industrial-scenes-v076`：四份可导入场景 JSON、打包 ZIP、完整运行记录与
summary.json（实际指标、接触/相机验证、最终物体位置、源码/模型/配置/任务哈希、复算命令）。
每份记录在同环境重新积分并比较完整规范 JSON；这验证固定案例，不代表任意编辑布局均可成功。
测试另检查配置独立性、文件往返、确定性初态、实际码垛高度与避障场景的采样规划。
图示仅用于浏览布局，报告中的 frames/trace/metrics 来自实际 MuJoCo 执行。

v0.7.5 性能测量在 `tests/browser_responsiveness.cjs`，以 PERF_LABEL 区分版本；
测量期间不能同时运行重负载，以免混淆墙钟结果。浏览器显示缓存不进入物理记录。
`scripts/verify_responsiveness.py --output <directory>` 用相同输入和每机器人 12,000 步预算，
分别在冻结 v0.7.4 wheel 与新环境运行。两机器人完整物理字段哈希相等，各自严格复算通过。
JSON 场景是可编辑初始配置，不能当作恢复任意时刻 qpos / qvel 的运行记录。

运行 `.venv/Scripts/python.exe scripts/verify_drag_feedback.py` 可复现受阻、抬高绕过、
松手恢复物理、再交给机械臂抓取。完整数据保存至 artifacts/drag-feedback-v074。
`tests/fixtures/object_drag_v073.json` 从独立安装的 0.7.3 wheel 冻结了障碍/物体/桌面/夹指
四种情况、12 份物理状态；新反馈必须保留相同 qpos、qvel、ctrl、时间。新增字段记录的是
几何拒绝，不是实际碰撞或接触力。world.blocking 为当前交互反馈，结束时置 null；
历史首次阻挡数据保留在事件中，录制画面回放不叠加现场的阻挡提示。


运行 `.venv/Scripts/python.exe scripts/verify_object_interaction.py`，验证两种机构 × 蓝/绿物体的
抓取、手动移走、再次抓取和严格复算；记录保存于 artifacts/object-interaction-v073。
动作记录区分 select_object 与 object grab(mode=direct_position)，手动移位不能当作自主算法效果。
物体高度曲线包含手动干预，放置成功仍需双指抓取、搬运与相机验证。

历史 v0.7.2 施力证据：运行 `.venv/Scripts/python.exe scripts/verify_force_continuity.py --output artifacts/force-continuity-after`
可复现搬运中施力。v0.7.1 基线位于 artifacts/force-continuity-before，保持相同初始配置、
输入、种子和 16,000 步预算。比较实际阶段、夹爪指令、目标周期数、完成步号和误差；当前结果严格复算。
只有 body 外力保留任务；force_started / force_released 事件记录当时的阶段与目标物体。

施力卡住的修复前记录保留在 `artifacts/force-repeat-before.json`。执行
`.venv/Scripts/python.exe scripts/verify_force_recovery.py` 会用相同初始配置、输入步号和步数预算
运行当前版本，再独立严格复算当前结果，输出 `artifacts/force-recovery-comparison.json`。
这属于跨版本回归对照，不会把旧源码的记录标成当前版本严格复算成功。
world 新增最后一次 recovery，报告新增最近 80 次 recoveries：实际几何检查数、接触对、路径与原因。

新增 `studio-world-v1`：config 是不可变初始配置，effective_config 是最后的运行设置；
actions 保存每个外力拖动、末端目标、设置变更、任务开关的精确整数步号。
模型、源码、版本、配置、任务哈希保留。最终 world 字段包含任务状态及实际外力向量/作用点。
帧/曲线保留最近约 40 秒，图像最多 40 张，IK/规划各最多 80 次；指标累计全程，输入最多 50,000 条。
超过输入上限仍可松手/挂起任务，应保存并新建场景。完整输入重建同样的滚动记录并比较全部规范 JSON。

复算：`.venv/Scripts/python.exe scripts/replay_world.py --record artifacts/studio_<id>/run.json`。
物理验证：`.venv/Scripts/python.exe scripts/verify_world.py`，两个机构 × 三种任务，先做同配置批处理基线，
历史 v0.7.0 通过弹簧移走物体后自动恢复；保留证据在 artifacts/world-final-validation。
当前物体直接拖动证据使用上面的独立脚本与目录，避免覆盖历史版本。
这是确定性场景回归，不是通用成功率或所有场景的性能承诺。旧 `studio-v1` 和算法记录仍由原执行器复算。

以下保留 v0.6 算法记录说明。

algorithm-lab-v1 导出包含 config_hash、task_hash、source_hash、model_hash、robot_hash、
随机种子、引擎/NumPy/包版本、world 坐标系、rad/m 关节单位和精确 replay_request。
lab_id 是全部字段（自身除外）的规范 JSON 内容哈希。浏览器“复算验证”重新计算并比较 ID；
scripts/replay_lab.py 先验证输入内容完整性，再重新计算并比较全部规范 JSON。

IK：140 次迭代，相同位置/姿态权重 1/0.25、步长上限 0.18、回溯策略和收敛阈值。
规划：相同 seed、起终点、场景；450 次采样迭代、12,000 次构型检查、0.028 SI 边采样。
一次采样的工作量因算法不同而不同，不能将迭代数直接当作运行速度排名。
时标：同一起终点和时长、101 个参考采样；没有生成伪造的跟踪误差或控制力矩。
控制：相同参考和时长、2 ms 引擎步；图表 20 ms 采样，RMS/峰值按每个物理步累计。

scripts/verify_algorithms.py 在两个机构上运行全部 13 种方法，共 26 个算法实验，并执行
16 个完整抓取案例。每个算法实验在独立进程中有相同 10 s 墙钟上限；报告 process_wall_s
包含启动、计算和序列化，不作为纯算法耗时。时间测量不写入确定性记录。
每个实验和物理案例均重算一次；成功、失败及完全一致检查都保留。整个回归集不是通用性能排名。

旧三种 IK 与 RRT-Connect 结果冻结于 tests/fixtures/studio_algorithms_v050.json；适配器与
数值测试检查兼容性。新增字段和源码哈希变化意味着旧版本记录的严格复算应拒绝，不能静默
以新实现解释旧数据。只读回放和数据导出仍然可用。

以下为 v0.5 及历史记录规范。

# 可复现实验 — v0.5.0

studio-v1 的不可变 run.json 保存配置、原始模型/源码/机器人身份、算法、种子与版本。
配置中位置/尺寸为 m，角度为 rad；R6 effort 为 N·m、XYZ effort 为 N，夹指 ctrl 为位置参考 m。
控制与物理步长 2 ms；帧/曲线 40 ms，相机计算 100 ms。每秒记录相机图像，另存任务观测关键帧。
所有误差、接触步数和力矩平方积分均累计每个物理步，不能把抽样帧称为完整 500 Hz 数据。

每条手动操作带 step；复算按相同位置插入动作，重新积分并逐字段比对整个报告。
图像噪声/缺失按相同随机种子和帧号生成，规划采样按固定种子，结果不含不可重现的墙钟耗时。
来源、版本或数据改变会触发严格复算失败；失败不能作为新算法优劣证据。
命令/参考时间位于积分前，state/实际几何位于积分后，遵循原合同。

`robostudio experiment replay studio_<ID>` 和网页“复算验证”使用相同实现。
`scripts/verify_studio.py --replay` 验证固定 30 场景：2 种机构，单件/分拣/堆叠、两种障碍规划、
低摩擦重物、控制器、逆解、相机/真值、圆柱/旋转长方体、观测噪声/延迟与固定布局种子。
报告保留所有成功和失败；运行目录中可找到每个 case 的原始记录。

成功标准包括真实双指夹持、抬升检查、放置图像验证和引擎落点/高度复核。
“26/30”等计数只描述这个固定回归集，不是通用抓取成功率；仿真误差不等于实机精度。

以下为仍可访问的 v0.4 /workcell 与更早基线记录规范。

# 可复现实验 — v0.4.0

## 接触抓取记录（workcell-v1）

cell_ 内容 ID 对应单个 run.json。2 ms 积分，30 ms 快照，统计值每步累计；保存完整 IK
迭代与所有场景广义位置、速度、控制输入，机械臂状态/参考/命令分开且有时间与身份。
模型 XML 按配置构造并取哈希；配置、任务、机器人映射、源码和版本均留档。
命令时间是前一积分区间起点，观测时间是区间终点。夹爪 ctrl 是位置参考，机械臂 ctrl 是力/力矩。

写入使用临时文件后原子替换；已有 ID 必须内容一致。加载时核对内容哈希及路径边界。
`robostudio experiment replay cell_<id>` 用原配置重新运行并比较整个文档。
源码或环境改变应拒绝严格重放，不将旧记录迁移成新结果。

运行 `scripts/verify_workcell.py` 可重做两种机构的 20 个案例及全部严格重放。
低摩擦和邻近物体干涉失败也计入报告；没有宣称每个布局都能成功。

以下是原 fake、重力与目标定位记录的规范；其中的“接触禁用”仅指原始 R6 场景。

## 场景与边界

- fake-v1 是工程测试替身，不代表机械臂性能。
- arm-run-v1 是固定 2 s 重力释放：0.002 s 物理周期、0.02 s 零力矩命令周期，
  1001 个完整状态；含独立 FK 与 Jacobian 校验。
- tracking-run-v1 是目标求解、五次关节参考和力矩控制：
  物理/控制周期均 0.002 s，目标到达后继续保持 0.8 s。
  用户时长与自动限速后的实际时长都留有记录。

物理使用原创 R6、固定重力与惯性。接触禁用；网格不是碰撞地面。
结果不构成实机精度、厂商性能或 L0/L1/L2 论文复现结论。

## 身份与内容

每个工件目录有 run.json 和 observations.json。
run.json 保存环境、输入、场景、清单/模型/源码 SHA256、seed、状态、指标、错误和重放命令。
源码哈希覆盖包内 .py/.json/.xml/.html/.css/.js/.svg，换行归一 LF。
模型哈希直接使用原始字节。v0.3.0 未修改 R6 MJCF，但增加清单能力，清单哈希随之改变。

运行 ID 是输入、源码、模型及环境的内容标识。相同输入重复运行复用相同记录；
不同内容不覆盖已有目录。先写临时目录再整体改名，失败不会伪装成成功记录。
环境含 Python、包、MuJoCo、NumPy、OS 与 CPU 架构，不宣称跨环境位级相同。

目标场景每步保存：
实际 RobotState、引擎连杆/TCP 坐标、当时 RobotReference、上一积分区间的 RobotCommand、
参考 TCP 和实际跟踪误差。初始样本的 command 为 null。
command 自带提交时间，因此可以区分施加命令的时间与积分后观测的时间。

网页使用每 10 步一帧（20 ms），最后一帧即使不是整倍数也保留。
目标运动的 JSON 导出包括回放帧和完整 observations；不会将抽样回放冒充完整原始记录。
完整文件还在项目 artifacts 目录。旧重力实验的网页导出仍为摘要加采样帧。

## 指标

| 指标 | 实际含义 |
| --- | --- |
| final_position_error_m | 最后引擎 TCP 与用户目标位置的欧氏距离 |
| final_orientation_error_rad | 最后引擎姿态与目标姿态的最短旋转角 |
| tcp_tracking_rms_m | 每个实际 TCP 相对同一时刻参考 TCP 的 RMS |
| joint_tracking_rms_rad | 全部状态和关节相对参考角度的 RMS |
| peak_effort_nm | 实际提交到电机的力矩最大绝对值 |
| saturated_commands | 被限幅的控制轴次数之和 |
| joint_limit_violations | 每个采样时刻超限轴的总数，容差 1e-6 rad |
| max_playback_fk_error_m | 独立 FK 与引擎 TCP 的最大差异，非跟踪误差 |
| sim_time_s | 仿真积分时间，非墙钟性能 |
| position_rms_rad（fake） | 相对零位的 RMS，非跟踪误差 |

求解失败保存 CONVERGENCE_FAILED，没有运动帧。
终态偏差 >2 mm、朝向误差 >0.01 rad 或记录到限位越界，保存 TRACKING_FAILED；
仍可查看真实运动。基础 PD 对照因缺少重力补偿，当前固定增益下预期未达精度。

## 重放

~~~text
robostudio experiment replay track_<24 hex>
robostudio experiment replay path/to/run.json --output path/to/output
~~~

在某个记录目录也可以执行 run.json 内的相对路径重放命令。
重放只调用本包显式绑定的可信算法，不从文件导入代码。它重新计算完整元数据和观测，
随后逐项规范化比较；改变源码、环境、配置、模型或观测将导致 INTEGRITY_ERROR。

网页「查看回放」只加载已保存观测，不要求它来自当前源码。它不是严格重放认证。
升级前版本的精确复现需要旧源码与旧环境。本项目尚未创建 Git 历史，
哈希可检测改动但不能自动恢复旧源码。

## 固定验收

scripts/verify_m1_motion.py 使用 seed=20261114 的 50 个目标（模型限位 80% 范围），
由引擎生成目标，再用独立几何 DLS 求解，并回到引擎核对位置和旋转残差。
结果如实报告收敛和失败；NOT_CONVERGED 不能证明不可达。

三个用户示例分别比较基础 PD / PD+偏置，保持模型、起始姿态、目标、轨迹、
增益、时间预算和步数一致。实验指标来自真实记录，基线不是人为填入的数字。
基线记录为预期未达精度的实验结果，整个验证脚本本身仍可以通过。
墙钟耗时仅存在于独立 acceptance.json 观察报告，不进入确定性重放标识。

本轮验收详情见 M1_COMPLETION_DELIVERY.md。
