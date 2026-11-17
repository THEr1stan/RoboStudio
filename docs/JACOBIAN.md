# R6 Compact TCP Jacobian

范围：原创建模的固定六轴臂，沿用 `R6_MODEL.md` 的模型、TCP 和清单。

## 坐标、单位和求导

J 为 6×6，列依次为 J1…J6（qvel 索引，不是 ctrl 次序）。
行顺序 `[vx,vy,vz,wx,wy,wz]`，全部在 world 表达。`J @ qdot` 的前三项为 TCP
平移速度 m/s，后三项为空间角速度 rad/s，不是 roll/pitch/yaw 的导数。

第 i 列：`Jv_i = z_i × (p_TCP - o_i)`，`Jw_i = z_i`。
独立 FK 给出 world 原点 o_i、轴 z_i 和含非零工具偏移的 TCP。结果为不可变 tuple，
q、qdot 和矩阵内容必须有限。

中心差分步长分别为 1e-4、1e-6 rad；平移块为 `(p(q+h e_i)-p(q-h e_i))/(2h)`。
角速度块先求 R_dot，再提取 `R_dot R^T` 的反对称部分对应向量。
不能用工具坐标的 `R^T R_dot` 或 Euler 角差分代替 world 空间角速度。
限位处对光滑 FK 仍以 ±h 求数学导数，不裁剪限位，不在引擎中越限运动。
奇异姿态下 J 本身有限，本切片不求逆、不宣称逆解存在。

## 三方校验

解析结果与中心差分、MuJoCo `mj_jacSite` 对照。引擎在已完成 forward 的状态上只读查询，
不改变仿真时间、关节、控制或 warm-start。3×nv 平移/旋转块按 `[linear; angular]` 合并并拷贝。

每次物理实验验证零位、一般位形、近腕奇异、全部下限、全部上限，以及 100 个随机位形。
默认验证 seed=42，独立于实验初始扰动的 seed。
阈值为 FD 最大元素差 ≤1e-8、引擎最大元素差 ≤1e-10，两个块分别记录误差及单位。
测试覆盖符号错误、列交换、工具偏移错误、NaN、非法步长/维度，以及 J qdot 的方向差分。
这不涉及真机参数辨识或跨硬件精度测量。

## 单姿态分析

`robostudio jacobian --q <6 rad> --qvel <6 rad/s>` 或网页“末端速度分析”。
网页输入 °/s、显示 mm/s 和 °/s；导出数据始终为 SI。
q 在模型限位内，qdot 每轴 ±π rad/s；此边界不是已验证的硬件速度限制。
网页假设一个关节转动、其余速度为零；CLI/API 可给出六轴速度。
此功能不发送控制指令、不积分成运动轨迹。非零平移方向用固定长度箭头显示，零平移无箭头。

零位、J1=30°/s 时，TCP 线速度 `(0,439.8229725,0) mm/s`、角速度 `(0,0,30) °/s`。
零位 J6 转动不改变 TCP 位置，但工具绕 X 轴旋转。

引擎约定依据：[MuJoCo 官方 Jacobian API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jacsite)。
独立公式、实现和数值对照为本项目工作，不构成论文复现结论。
