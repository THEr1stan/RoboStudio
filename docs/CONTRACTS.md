# M0 合同与 v1alpha1 清单

可执行 schema 位于 `registry/manifests.py` 与 `core/contracts.py`，不依赖第三方 schema SDK。
它是严格的固定字段校验器，不是任意 JSON Schema 解释器。
有效示例：`examples/manifests/fake_arm2.json`；无效示例缺少 `robot_id`。

## 机器人字段（全部必需）

| 字段 | 类型 / 规则 |
| --- | --- |
| `api_version` | 仅 `v1alpha1` |
| `robot_id`, `family`, `frame_id` | 非空字符串 |
| `qpos_size`, `qvel_size`, `ctrl_size` | 非负整数，拒绝 bool |
| `model` | `path`、`format`、`sha256`；SHA256 必须为 64 位小写十六进制 |
| `capabilities` | 已知能力字符串列表，不允许重复 |
| `joints` | `name`, `kind`, `qpos_indices`, `qvel_indices` |
| `joint_groups` | `group_id`, `joint_names`；组内唯一，引用已知关节 |
| `actuator_mapping` | `name`, `joint_name`, `ctrl_index`, `effort_unit` |

每层都拒绝未知字段。JSON 不允许重复键或 NaN/Infinity。错误包含稳定 `code`、`path` 和原因。
关节名、组 ID、执行器名、注册机器人/插件 ID 分别检查重复。

关节位置和速度索引必须分别覆盖对应维度且不重叠，不能假设 qpos 与 qvel 等长。
`revolute` 与 `prismatic` 为 1/1；`free` 为 7/6。
执行器索引独立覆盖 ctrl；M0 只接受标量关节 effort actuator。自由关节不允许被简单映射成一个力矩标量。
多个执行器可以作用于同一标量关节，FakeBackend 对 effort 求和。

`validate robot` 还会读取清单目录内的本地模型并校验原始字节哈希；拒绝越出该目录的模型路径。
不会联网获取模型，不会执行模型内容。直接调用 `parse_robot` 仅做 schema/结构校验。

## 单位、坐标和数据类型

- `Pose`：位置 m，四元数 wxyz，范数容差 1e-9；相对于 `frame_id` 表达。
- 转动关节 qpos/qvel：rad / rad·s⁻¹；移动关节：m / m·s⁻¹。
- free qpos：xyz(m), wxyz；free qvel：线速度 xyz(m/s), 角速度 xyz(rad/s)。顺序由关节索引元组映射。
- free 位姿/速度的表达坐标由 sample 的 `frame_id` 指定。未来后端必须显式转换本地约定。
- command effort：旋转关节 `N*m`，移动关节 `N`；按 ctrl 索引排列。
- `time_s` 为非负模拟时间，不是系统时间。状态、参考与命令包含 robot_id、robot_hash、model_hash、frame_id。
- `RobotState`、`RobotReference`、`RobotCommand` 是不同类型；后端只接受命令。
- 向量复制成 tuple，capabilities 复制成 frozenset，结果 metrics 复制成只读映射。
- 与机器人相关的维度/身份/frame 校验通过 `validate_for(robot)`；后端在执行前调用它。
- fake 命令时间必须与当前状态一致；无效命令或数值异常不得提交新状态。

## 能力与插件

能力：joint_effort、joint_trajectory、cartesian_ik、base_velocity、navigate_2d、floating_base_control。
“名称被合同识别”不代表实现了对应算法或机器人。

插件字段：`api_version`、`plugin_id`、`requires_capabilities`、`provides_capability`、`parameters`。
每个参数必须声明 `{ "type": "number" 或 "integer", "minimum": 数值, "maximum": 数值 }`。
参数无隐式默认值，必须精确提供所有参数；拒绝非有限数值、bool、超范围或多余字段。
复杂 schema 仍未引入。v0.3.0 的算法执行协议位于 core/motion.py。

Registry 支持独立添加插件清单、根据所需能力匹配机器人；不要求修改 Core，不依据机器人名称选算法。
内置可信插件为 dls_pose_v1、quintic_v1、joint_pd_v1；注册信息通过严格 JSON 清单校验。
运行时显式绑定实现，不按用户文件导入代码。dummy 插件仍只在合同测试中注册。

IKProblem 包含 world Pose、6 维 seed_q_rad、机器人/模型哈希、有限迭代/重启预算及种子。
IKResult 返回 CONVERGED 或 NOT_CONVERGED，以及关节解、位置/朝向残差、迭代与尝试次数。
NOT_CONVERGED 表示有限搜索失败，不代表不可达证明。运动失败区分
CONVERGENCE_FAILED（没有开始运动）和 TRACKING_FAILED（保留真实运动但未达验收标准）。
这是一组新增合同和能力声明，已有 state/reference/command 格式保持兼容。

## 生命周期和错误

`new → load(robot) → loaded → reset(seed) → ready → apply/step/observe → close → closed`。
ready 状态允许 reset；close 幂等；closed 后不能继续使用同一实例。
seed 是 `[0, 2**32)` 的整数；fake 步长为 `(0, 0.1]` 秒。

CLI 成功为 0，保存了失败实验为 1，用户输入/schema/IO/完整性错误为 2。
常见错误：INVALID_MANIFEST、INVALID_MAPPING、INVALID_DIMENSION、FRAME_MISMATCH、MODEL_MISMATCH、
UNKNOWN_CAPABILITY、UNSUPPORTED_CAPABILITY、DUPLICATE_ID、INVALID_LIFECYCLE、INTEGRITY_ERROR。
