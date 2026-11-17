# R6 Compact 模型定义与来源

原创建模，不使用下载的机器人资产或厂家参数。仅用于本项目的理想化仿真实验，
不对应已标定的工业机型。未发布，尚未指定对外分发许可证。

MJCF 位于 `src/robostudio/data/r6_compact.xml`，清单同目录 `.json`；清单记录原始 XML SHA256。
独立参考实现位于 `kinematics.py`，不从 XML 或引擎反推参考答案。前端 Compact 几何与其一致。
所有位置以米、转角以弧度、质量以 kg；frame=world，Z 朝上，固定基座位于原点。
六个关节都是右手旋转，按“局部轴旋转，再沿当前坐标系平移”串联。

| 关节 | 局部轴 | 关节之后的偏移 m | 限位 ° | 连杆质量 kg |
| --- | --- | --- | --- | --- |
| J1 | Z | (0, 0, 0.22) | ±180 | 3.0 |
| J2 | Y | (0.32, 0, 0) | ±120 | 2.5 |
| J3 | Y | (0.28, 0, 0) | ±140 | 1.8 |
| J4 | X | (0.07, 0, 0) | ±180 | 0.5 |
| J5 | Y | (0.07, 0, 0) | ±120 | 0.4 |
| J6 | X | (0.10, 0, 0) | ±180 | 0.3 |

TCP 是 link6 局部 (0.1, 0, 0) 的 `tcp` site。零位 world 位置 (0.84, 0, 0.22)，
旋转矩阵单位阵。图中的装饰夹爪不增加 TCP 偏移或负载；展示外壳不是精确碰撞几何。

惯量由均匀 capsule、上述质量和几何尺寸计算：前三段半径 0.035 m，后三段分别 0.028、
0.025、0.020 m。所有关节阻尼 2 N·m·s/rad，转子惯量 armature=0.02 kg·m²。
这些是明确的假设，未经硬件辨识。积分器 implicitfast，步长 0.002 s，重力 (0,0,-9.81)。
所有 geom 的 contype/conaffinity=0，没有地面或自碰撞；运动可能穿过坐标网格。
关节限位使用引擎软约束，动力学状态可有小幅越界，不应把它解释成硬限位。

qpos/qvel 均按 J1…J6 排列，但 ctrl 顺序故意设为 J2、J1、J3、J4、J5、J6，
通过引擎命名索引与清单逐一核对。motor 为单位齿比、无偏置固定增益纯力矩输入；
ctrl 限制依次 ±100、±100、±60、±20、±20、±10 N·m。超限请求在调用引擎前报错。

六轴记录同时保存 robot/model/source/config/scenario hashes。环境及观察值严格重放校验，
不承诺跨 CPU、平台或引擎版本位级一致。v0.2.1 已提供 TCP 的 world Jacobian 与瞬时速度分析，
约定见 `JACOBIAN.md`。v0.3.0 已增加末端目标求解、限速关节轨迹和 PD 力矩跟踪，
未修改原始物理模型。没有碰撞路径规划、工业参数辨识或真机接口。

引擎/API 参考：[MuJoCo Python 官方文档](https://mujoco.readthedocs.io/en/stable/python.html)、
[MJCF 官方定义](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。
使用 MuJoCo 4.0.1 的最小 `mujoco-core` 发行包，浏览器已有绘图器，无需另装桌面 Studio。
