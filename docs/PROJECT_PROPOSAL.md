# RoboStudio AI：面向机器人算法的论文复现与仿真验证平台

**项目立项书 / 技术方案（v0.3 · 2026-11-14）**  
**状态：拟议，供 Codex 分阶段实施；尚未宣称功能已实现**  
**项目类型：开源个人研发项目 / 机器人算法研究工具 / 仿真验证平台**  
**目标用户：机器人方向学生、研究人员、机器人算法与软件工程师**

> **一句话定位**：上传一篇符合支持范围的机器人算法论文，系统辅助提取可追溯的数学与实验规格，生成可审核的算法插件，在既有机器人模型和统一场景上运行仿真、对比基线并输出可复现实验报告。整个流程从六轴机械臂开始，后续通过相同基础设施扩展轮式、ROS 2 和轮腿机器人。

---

## 0. 执行摘要：项目为什么值得做

### 0.1 从“通用仿真软件”到“有辨识度的研究工具”

单独提供机器人加载、运动规划、可视化和数据曲线，容易与现有仿真工具或 AI 快速生成的 Demo 同质化。RoboStudio AI 的差异化价值在于：

- **Paper-to-Simulation**：将“论文中的算法”转换为“可运行、可审核、可测量的机器人仿真插件”。
- **Algorithm Arena**：将已有实现与论文实现放进相同机器人、相同任务、相同计算预算的基准实验中，自动比较并定位失败场景。
- **Robot Capability Platform**：机器人、控制器、规划器、仿真后端和实验协议解耦，使后续添加新机型和新算法的成本显式、可评估。

核心展示不是“AI 能写代码”，而是**算法从研究描述到系统验证的可追溯工程链条**：论文依据 → 规格 → 代码 → 测试 → 仿真 → 比较 → 失败分析。

### 0.2 项目必须保持的三个边界

1. **仿真先行**：当前缺少真机，所有结果明确标注 Simulation-only；未来硬件接入另行验收。
2. **有限范围的自动复现**：首版仅针对**公式/伪代码清晰、无需训练的数值逆运动学（IK）算法**。论文一键“100% 自动复现”属于长期研发目标。
3. **人机协同**：AI 负责解析与生成候选方案，系统由可执行的测试与度量进行判定；论文描述不充分时必须显式提出缺失信息，允许人工审核后继续。

### 0.3 建议的第一段公开演示

用户在 RoboStudio 中选择内置 6R 机械臂和一篇**纳入支持范围的 IK 论文**：

`导入 PDF → 自动提取方法和公式/来源页码 → 审核参数与坐标约定 → 生成 IK 插件候选 → 在隔离环境测试 → 100~300 组固定种子的 IK 基准 → MuJoCo 中跟踪若干目标 → 普通 DLS 对照 → HTML 复现报告`。

成功结果可以进入“已验证插件库”，失败则留下准确的测试日志和原因。这一演示可独立于 GUI 通过 CLI 完成，降低首次落地成本。

---

## 1. 用户、痛点与应用场景

| 用户 | 当前痛点 | 项目提供的初期价值 |
|---|---|---|
| 机器人研究生 | 阅读论文后需要自行理解公式、搭环境、写代码和画图 | 自动生成可核查的算法规格、标准插件和对比报告 |
| 算法工程师 | 新方法是否适配已有机器人难以预估 | 在现有模型与统一 benchmark 中快速评估适用性 |
| 机器人软件工程师 | 换机型时重复修改大量基础代码 | 机器人包、算法接口、场景和任务分离 |
| 项目作者本人 | 缺少实机，但希望证明算法与交付能力 | 以可复现仿真、性能数据、失败修复和工程文档证明能力 |

**首期真正服务的用户**：项目作者本人，必要时邀请 1~2 名同学在自己的电脑上复现。等最小版本可靠后再做公开用户体验、权限与多用户服务。

---

## 2. 功能范围与明确的非目标

### 2.1 MVP（第一版有价值的完整产品）

- 管理并运行**一个经过验证的六轴机械臂模型**，支持 FK、Jacobian、关节限位、DLS IK 和简单关节伺服；
- MuJoCo 物理仿真（无头运行+原生 Viewer），固定场景、目标和可复现实验记录；
- 数值 IK 插件协议和 DLS 基线；
- 导入**机器可提取文本的 PDF**、页面溯源、算法规格 JSON/YAML 和人工审核；
- 利用可替换的 LLM Provider 从经审核规格生成**受限制的 IK 插件候选**；
- 插件在隔离环境构建、运行合同测试、数学测试、超时与资源约束后接入算法竞技场；
- 自动对比基准与生成包含成功/失败案例的 Markdown/HTML 报告；
- CLI 完成全流程；图形界面只作为后续增强项。

### 2.2 后续（已经预留接口，暂缓实现）

- RRT-Connect、碰撞规划、轨迹时标、虚拟抓放工作站；
- 第二种机器人：差速轮式车，通过无 ROS 的基础场景验证框架扩展性；
- ROS 2 Bridge 与 ROS 小车、Nav2/MoveIt 2 适配；
- 轮腿机器人：浮动基座、接触与稳定控制，属于独立高难度课题；
- 其他类别论文（轨迹优化、MPC、学习控制）、模型自动适配与多 Agent 修复；
- GUI/多用户云端平台/一键发布插件等。

### 2.3 近期明确不实施

任意 PDF 的零人工全自动复现；论文结果原样复现的普遍承诺；任意第三方代码无审核执行；自动下载陌生依赖和大模型权重；真实硬件安全部署；自研物理引擎；大规模 SLAM 与训练系统；在初期同时维护 ROS 2、Gazebo、MuJoCo 三套后端。

---

## 3. 三层“复现”定义：避免误导用户

| 层级 | 术语 | 含义 | 第一版 |
|---|---|---|---|
| L0 | **实现一致性（Implementation Verification）** | 生成的代码满足所理解的公式、接口、数值测试；假设来源可追溯 | **必做** |
| L1 | **迁移验证（Transfer Evaluation）** | 算法在 RoboStudio 自有机械臂和自建任务上与基线进行比较 | **必做** |
| L2 | **论文实验复现（Paper Reproduction）** | 使用论文的机器人、数据、参数、训练和评测协议，在允许误差内重现原实验 | **长期可选** |

报告必须显示复现层级，并且在 L1 场景标记“已迁移测试，尚未重现论文原实验”。论文缺少必要参数时只能输出 `NEEDS_SPECIFICATION` 或按明示假设继续，不得悄然补全并宣称复现成功。

---

## 4. 系统架构

### 4.1 层次与职责

```text
┌─────────────────────────────────────────────────────────────┐
│ CLI / Viewer / Future GUI                                   │
├─────────────────────────────────────────────────────────────┤
│ Applications                                                │
│   Arm Workcell    Diff-drive Navigation    Algorithm Arena   │
├───────────────────────────┬─────────────────────────────────┤
│ Research Pipeline         │ Experiment Runtime              │
│   PDF Ingestion           │   Scenario / Seed / Scheduler    │
│   Evidence Extractor      │   Measurement / Failure Logs     │
│   Algorithm Spec Review   │   Benchmark / Report             │
│   Code Generation         │                                 │
│   Sandbox + Test Gate     │                                 │
├───────────────────────────┴─────────────────────────────────┤
│ Core Contracts: Robot / Capability / Algorithm / Pose / Task│
│                 Registry / Units / Frames / Status          │
├───────────────┬─────────────────┬───────────────────────────┤
│ Robot Packages│ Algorithm Plugins│ SimulationBackend       │
│ arm6 / diff   │ IK / planner / ctl│ MuJoCo / (future)      │
├───────────────┴─────────────────┴───────────────────────────┤
│ Optional ROS 2 Bridge    |   Metrics & Run Artifacts         │
└─────────────────────────────────────────────────────────────┘
```

**依赖方向**：CLI/Research/Applications → 公开 Core API ← 具体 Backend / Robot / Plugin 的实现通过 Registry 注册。Core 不 import MuJoCo、LLM SDK、ROS 或 UI。论文自动化模块调用稳定的算法合同，不负责修改核心架构。

### 4.2 机器人扩展原则

- **机器人包**：模型资产、关节组、运动学/动力学参数、TCP/frame、执行器映射、capabilities、默认控制器配置和验证案例。
- **能力接口**：`cartesian_ik`、`joint_trajectory`、`base_velocity`、`navigate_2d`、`floating_base_control` 等按需注册；**不存在让所有机器人共用同一套 IK 的强制要求**。
- **插件**：算法通过接口注册，声明 `plugin_id`、`api_version`、能力、参数 schema、支持的机器人条件、资源需求和验证状态。
- **后端**：同一状态/命令协议允许未来扩展；首个 MuJoCoBackend 建成并测通后才决定是否增加 Gazebo。ROS Bridge 是通信适配层，不充当通用物理引擎。
- **模型映射**：关节位置 `qpos`、速度 `qvel`、执行器 `ctrl`、逻辑关节组分别显式映射。含自由关节时维数和坐标表示可能不同，禁止简单按数组下标假定一一对应。

### 4.3 坐标和数值约定

- 统一使用 SI：m、rad、s、N、N·m；姿态采用旋转矩阵或归一化四元数，在接口定义中明确顺序。
- 每个 Pose/Twist 带有 `frame_id`；控制任务区分 `robot_state`、`desired_state`、`command`。
- 末端 6D 位姿误差统一定义（位置误差与 SO(3) 对数姿态误差）；Jacobian、IK 和实验指标使用相同 TCP/坐标约定。
- 对位置与姿态误差进行合理加权/缩放，记录阻尼参数、初值、多初值数量与停止原因。
- `physics_dt` 与 `control_dt` 分离，**仿真速度**和**算法实际墙钟耗时**单独计量。

### 4.4 首期接口草案（示意，具体字段以 M0 的合同测试定稿）

```python
from dataclasses import dataclass
from typing import Protocol
import numpy as np

@dataclass(frozen=True)
class Pose:
    position_m: np.ndarray       # shape (3,)
    quaternion_wxyz: np.ndarray # shape (4,), explicit convention
    frame_id: str

@dataclass(frozen=True)
class IKProblem:
    robot_id: str
    group_id: str
    tcp_frame: str
    target: Pose
    seed_q_rad: np.ndarray
    joint_limits_rad: np.ndarray  # shape (n, 2)
    timeout_s: float

@dataclass(frozen=True)
class IKResult:
    status: str  # enum in implementation
    q_rad: np.ndarray | None
    residual_position_m: float | None
    residual_orientation_rad: float | None
    iterations: int
    elapsed_s: float
    diagnostics: dict

class IKSolver(Protocol):
    def solve(self, problem: IKProblem) -> IKResult: ...

class SimulationBackend(Protocol):
    def load(self, scene_id: str, robot_id: str) -> None: ...
    def reset(self, seed: int) -> None: ...
    def observe(self) -> object: ...
    def apply(self, command: object) -> None: ...
    def step(self, dt_s: float) -> None: ...
    def close(self) -> None: ...
```

**重要限制**：这只是接口方向，示意数据类型尚缺少 schema 校验、数组只读/复制、关节名映射、时戳和错误枚举。Codex 必须先通过合同测试确定正式 API，再扩展实现。

---

## 5. Paper-to-Simulation 流程设计

### 5.1 文档输入

首版支持“单篇、文字层可解析的 PDF”；提供标题、DOI/URL、文件 SHA256、论文版本、导入时间。以 PyMuPDF 等工具提取文本，保存**每个算法描述与 PDF 页码的对应关系**。表格、公式和伪代码可能需要人工核对。扫描版 OCR 后续加入，需区分低置信度内容。

**来源规则**：论文提出的公式、约束、超参数、实验条件必须附页码或章节来源；AI 的猜测在数据结构中标 `assumed`，不能伪装成原文事实。

### 5.2 结构化 AlgorithmSpec

最小字段建议：

```yaml
schema_version: "v1alpha1"
source:
  paper_title: "[manual or extracted]"
  sha256: "[64 hex chars]"
  algorithm_section: "Section 3.2"
method:
  family: numerical_ik
  name: "[method name]"
  inputs: [joint_positions, target_pose, kinematic_model]
  outputs: [joint_positions, diagnostics]
  equations:
    - id: "eq_4"
      source_pages: [5]
      expression: "[human-checked transcription]"
  parameters:
    - name: lambda0
      value: null
      status: missing_in_paper
  assumptions:
    - text: "[coordinate convention]"
      provenance: author_inferred  # or paper_explicit / user_confirmed
  evaluation_claims:
    - text: "[claim requiring a measurable test]"
      source_pages: [7]
review:
  status: pending  # pending / approved / rejected
```

AlgorithmSpec 作为“论文→代码”之间的**冻结中间表示**，必须有 schema validation 和版本。第一版人工点击批准（CLI approve）后才能生成并执行插件。

### 5.3 受约束的算法代码生成

- 提供固定 IK 插件模板、输入/输出 API、允许的依赖、DLS 基线与目标测试；
- 先生成**实现计划**（公式与代码函数一一映射），再生成独立工作区的候选代码；
- 每次记录模型供应商、模型名、提示模板版本、规格 hash、文件 diff 和生成时间；
- 生成器仅能修改候选插件目录及测试目录。禁止直接修改 Core、已有基线、真实机器人控制或敏感配置；
- 预算限定重试次数，超出预算交给用户诊断；不得自动宣称“论文复现成功”。

第一版可直接使用你已有的 Codex 作为**人工监督的代码生成执行器**。等流程固定后，再实现独立的 Provider API，避免过早开发复杂 Agent 编排框架。

### 5.4 生成代码的隔离验证

代码属于不可信产物。运行前必须：

1. 做代码差异审核与依赖清单检查，拒绝未批准的命令、网络请求、文件越界与秘密读取；
2. 在隔离的容器/受限执行环境运行，关闭网络、挂载最小化目录、资源和时间上限、非特权用户，限制进程数；
3. 运行格式化/类型检查/最小导入测试和 IK 合同测试；
4. 运行有限差分等数学性质测试、边界测试、随机种子验证；
5. 通过之后才提交给 Arena；不能把候选插件直接加载到长期运行的主进程。

*注：容器是隔离措施之一，仍需根据使用环境审查安全配置。若暂时没有可靠隔离环境，系统只生成候选文件和手动操作说明，暂停自动运行。*

### 5.5 三种结果状态

- `VERIFIED_L0`：算法实现和合同测试通过；
- `TRANSFER_EVALUATED_L1`：已完成自建机器人迁移评测；
- `REPRODUCED_L2`：完整按论文原实验协议验证，人工审定后才可标记。

常见失败状态包括：`PDF_PARSE_FAILED`、`EQUATION_UNCLEAR`、`NEEDS_SPECIFICATION`、`SPEC_REJECTED`、`CODEGEN_FAILED`、`DEPENDENCY_DENIED`、`SANDBOX_FAILED`、`TEST_FAILED`、`INTEGRATION_FAILED`、`EVALUATION_INCOMPLETE`。所有失败必须形成可追踪工件。

---

## 6. Algorithm Arena：公平评测与科学可信度

### 6.1 首个基准套件：六轴 IK

- 机器人模型固定（含版本和 SHA256），采用确定的关节上下限、零位和 TCP；
- 任务集包含由合法关节角 FK 生成的**可达位姿**、奇异附近、限位附近、重复目标、姿态突变；
- 不可达测试集通过明确构造约束和几何条件单独标记，不能用普通 IK 求解失败直接证明物理不可达；
- 所有算法使用相同初值集合、误差容差、计算预算、机器环境、限制与种子；
- 比较指标：求解成功率、位置/姿态残差、迭代数、p50/p95/p99 墙钟耗时、关节变化范数、限位违规、停滞与数值异常占比；
- 如测试“连续目标跟踪”，需单列时间序列评测，避免把静态 IK 搜索耗时误当实时闭环延迟。

**建议起步的验收规模（项目要求，可在首个数据集建立时修订）**：50 个固定基础样例作为 CI 烟雾测试；200 个固定评测位姿作为第一份公开报告；后期新增不公开测试集以减轻针对单一基准过拟合。性能阈值以 DLS 基线和数值测试结果制定，不预先承诺具体成功率。

### 6.2 MuJoCo 动力学跟踪

对每个通过 IK 的候选关节目标，轨迹发生器产生限速/限加速度的关节参考。控制层使用关节 PD（可选重力前馈），在 MuJoCo 中执行并记录真实反馈。区分：

- **运动学可达** vs **无碰撞路径可达** vs **控制执行可行**；
- 简化几何/抓取附着 vs 真实接触抓取；
- 仿真时间 vs 墙钟时间。

当路线规划尚未实现时，任务只能在**预先证明无碰撞的简单路径**上运行，并显示相应假设。

### 6.3 实验报告结构

```text
Run ID / date / software commit / Python-MuJoCo versions
Paper citation + PDF sha256 + page-level evidence
Spec hash + approval status + code diff / model metadata
Robot model hash / TCP / coordinate convention / limits
Test scenario manifest + fixed seeds + machine / time budget
Baseline vs candidate metric table / distribution charts
Successful cases / failed cases / logs / replay commands
Result level: L0 / L1 / L2 (and reason)
Known limitations, manual assumptions, reviewer notes
```

报告提供一条可复制的 `robostudio experiment replay <run_id>` 命令，并确保相关依赖、模型与算法插件版本可获取。对外展示时允许隐藏论文文件全文和私有模型凭据，只公开复现所需的合法元数据和可再分发工件。

---

## 7. 技术选型（优先降低依赖复杂度）

| 子系统 | 第一阶段建议 | 选择理由 / 迁移边界 |
|---|---|---|
| 语言 | Python 3.11+；关键算法可复用 C++/Eigen | Python 方便 AI、实验和 MuJoCo；保留 C++ 算法深度 |
| 仿真 | MuJoCo + 原生 Python binding | 使用成熟的刚体动力学，不自造引擎 |
| 机器人描述 | 首期固定 MJCF + MDH 对照参数 | 先确保一个模型数值正确，再探索 URDF 转换 |
| 算法 | NumPy、SciPy、Eigen（按需） | DLS/规划器先维护可信基线 |
| PDF | PyMuPDF，图片/扫描作为二期 | PDF 文字解析可控；公式保留人工确认 |
| 结构化规格 | Pydantic 或等价 schema validator | 实现 AlgorithmSpec、Manifest 的严格校验 |
| LLM | 可配置 Provider 或 Codex supervised mode | 明确可选依赖；无 API 时可手动规格+插件 |
| 生成执行 | 独立隔离 workspace + 受限容器 | 保护主项目和用户文件 |
| 测试 | pytest、ruff、mypy/pyright 其一、GitHub Actions | 合同测试、数学单测、基准 CI |
| 数据 | YAML/JSON spec + NPZ/CSV logs + HTML/Markdown report | 最小持久化；有规模后再引入 SQLite |
| CLI | Typer 或 argparse | 第一版优先清晰可测试的命令行 |
| ROS 2 | 作为后续独立 Bridge | Ubuntu 24.04 可选择 Jazzy；Ubuntu 26.04 可选择 Lyrical；依开发环境定稿 |
| GUI | 首版采用 MuJoCo Viewer + 报告 | 避免多端 UI 抢占算法开发资源 |

**版本策略**：启动开发时按官方兼容文档选定依赖版本并锁定；本文不硬指定尚未验证的最新小版本。CI 至少验证一个明确的 OS/Python/MuJoCo 组合。ROS 2 工程后续用单独容器与锁定的 Ubuntu/ROS 组合测试。

---

## 8. 仓库目录方案（先创建活跃模块，其余逐步落地）

```text
robostudio/
├── README.md
├── AGENTS.md
├── pyproject.toml
├── uv.lock                        # 选择 uv 时提交
├── docs/
│   ├── PROJECT_PROPOSAL.md        # 本文
│   ├── ARCHITECTURE.md
│   ├── REPRODUCIBILITY.md
│   ├── EXTENDING_ROBOTS.md
│   ├── PAPER_REPRODUCTION_POLICY.md
│   └── adr/
├── tasks/
│   ├── ACTIVE.md
│   ├── M0_BOOTSTRAP.md
│   └── P0_PAPER_PIPELINE.md
├── src/robostudio/
│   ├── core/                     # robot state, frame, commands, typed result
│   ├── registry/                 # robot/plugin manifest
│   ├── backends/                 # backend protocol, MuJoCo impl
│   ├── runtime/                  # clock, control loop, run lifecycle
│   ├── kinematics/               # reusable FK/Jacobian (or plugin)
│   ├── algorithms/               # IK base + reference plugins
│   ├── experiments/              # cases, benchmark, metrics, reports
│   ├── research/                 # PDF, evidence, spec review, codegen
│   ├── execution/                # candidate validation, sandbox controller
│   └── cli.py
├── robot_packages/
│   ├── arm6_demo/
│   └── diff_drive_demo/          # later milestone
├── algorithm_plugins/
│   ├── dls_baseline/
│   └── generated/                # isolated, approved candidates only
├── scenarios/arm6/
├── benchmarks/ik_v1/
├── examples/
│   └── robot_package_template/
├── artifacts/                    # gitignored run outputs
├── native/                        # optional C++ integration later
├── ros2_ws/                       # later, optional
├── tests/
│   ├── unit/
│   ├── contracts/
│   ├── integration/
│   └── e2e/
└── .github/workflows/
```

**反对提前铺满空目录**：M0 只创建实际用到的 core、registry、fake backend、CLI、测试和文档。research/execution/ros2_ws 等目录到对应里程碑再加入。

---

## 9. 开发路线：里程碑、可见成果与退出条件

下表按**一个有 AI 辅助的研究生/开发者、兼职推进**估计。工作量为宽松的开发小时级估算，受现有代码质量、模型可用性、环境等影响很大。每一阶段验收后才进入下一阶段。

| 阶段 | 估算工作量 | 核心实现 | 可展示结果 / DoD |
|---|---:|---|---|
| **M0 工程底座** | 15–30h | 工程包、接口、插件清单、fake backend、CLI、合同测试/CI | `list/validate`、错误配置测试、可重复 fake run |
| **M1 六轴仿真与可信算法** | 35–70h | 6R 模型、MuJoCo、FK/Jacobian/DLS、PD、日志 | FK 数值对照、Jacobian 差分、目标跟踪视频/报告 |
| **M2 IK Algorithm Arena** | 25–50h | 固定任务集、批量求解、误差/耗时/失败报告 | DLS 的 50 个 CI+200 个基准案例，可 replay |
| **P0 论文规格提取（人工审核）** | 25–45h | PDF 解析、页码溯源、AlgorithmSpec、CLI approve | 一篇明确 IK 论文可生成结构化规格并记录缺失字段 |
| **P1 生成 IK 插件候选** | 45–90h | 受约束生成、独立目录、执行隔离、自动测试 | 一篇受支持论文生成插件，验证失败也可准确解释 |
| **P2 一键跑通受控工作流** | 30–60h | 状态机、重试预算、自动对比和报告、演示脚本 | 手动审核一次后完整 Paper→Spec→Plugin→Arena→Report |
| **M3 机械臂规划工作站** | 50–100h | 碰撞检测、RRT-Connect、时间参数化、简化抓放 | 预设障碍物场景中完成无碰撞搬运，失败可诊断 |
| **M4 第二种机器人** | 35–70h | 差速车包、2D 导航/控制、复用 runtime/experiments | 新车接入成本可记录，核心 API 无机器人名硬编码 |
| **M5 ROS 2 桥接** | 30–70h | topic、frame、command、clock、配置与桥接测试 | 与选定 ROS 2 环境交互，纯 Python 场景继续可运行 |
| **R1 轮腿及高级论文** | 单独立项 | 浮动基座、接触模型、基础平衡、高级控制复现 | 明确的独立 benchmark，不作为首版延期原因 |

**第一份有吸引力的公开演示目标：M0 + M1 + M2 + P0 + P1 + P2。** 此组合预计约 **175–345 小时**的开发量，属于估算而非工期承诺。已有算法代码能复用时可能缩短；若模型校准与沙盒环境有障碍则可能增加。

**第一份可靠 MVP 的最小剪裁**：若 P1 复杂，先让用户手动审核 AlgorithmSpec 并通过 Codex 在受控候选目录生成插件，再由项目自动完成测试/仿真/报告。这样已经具备实用价值；无需等待全自动 Agent。

---

## 10. 验收计划与可测指标

### 10.1 工程合同验收

- 同一清单版本、seed 和假后端产生确定的输出；非法参数获得稳定错误码。
- Core 对 ROS/MuJoCo/LLM SDK 没有直接依赖。
- 新增一个本地 IK 算法不需要更改 Core，能通过独立合同测试和 Arena。
- 机器人模型、joint name、TCP、actuator 映射均有可自动验证的完整性检查。
- CI 上能无头运行 `lint + typecheck + pytest + smoke experiment`。

### 10.2 运动学与控制测试

- FK 对比仿真中的相同 TCP（在统一零位、轴、单位和坐标系约定下），容差依据模型与浮点精度设定；
- Jacobian 通过中心有限差分检验，重点测试非零工具偏置、临近奇异位形、角度符号；
- IK 返回值需重新跑 FK 校验残差，并检查关节限位与 finite values；
- 固定基准只统计同样的任务、初值、时间预算和误差容差；
- 控制器记录 tracking RMS、peak torque、超限和跟踪停止状态；
- 无路径规划时只声明局部目标跟踪成功，不能以此推断无碰撞整段运动。

### 10.3 Paper pipeline 验收

- 能解析至少一篇已选定、公式清晰的目标论文；证据字段有页码；人工审核后 spec freeze；
- 自动生成候选插件遵守 IK 合同，通过静态/数学/基准测试或给出明确失败原因；
- 在**相同测试集**与 DLS 基线进行比较；包括正常、奇异、限位以及明显失败案例；
- 每个结果都有 `source_sha + spec_hash + code_commit + robot_hash + benchmark_id + seed`；
- 单次实验报告包括说明性复现等级 L0/L1/L2，生成并保存 replay 命令；
- 生成代码不会覆盖已验证算法，也不能未经用户批准接入实体硬件。

**对外效果指标建议**（团队内部目标，非当前成绩）：支持**一种论文类别**、至少**2 篇经过人工选型的 IK 论文**、各自产出完整证据链；记录首轮通过率、人工修改量、任务用时、最终修复次数；清晰统计失败样例。

---

## 11. 主要风险、决策与止损机制

| 风险 | 概率/影响 | 应对策略 | 触发止损条件 |
|---|---|---|---|
| 论文公式/伪代码不完整 | 高/高 | 审核中间规格、来源页码、缺失参数状态 | 缺必需公式或实验条件时转人工或更换论文 |
| 生成代码编译通过但算法错误 | 高/高 | 独立基线、数值测试、故障注入和 Arena | 合同/数学测试不通过，不进入已验证插件库 |
| 过度设计插件体系 | 中/高 | 最小 contract，第二种机器人后再泛化 | 仅出现接口却没有运行演示时停止新增抽象 |
| MuJoCo 与 MDH 模型不一致 | 高/高 | 固定 TCP、FK、关节方向和 tool offset 对照 | 核心运动学测试无法通过时暂停 AI 功能 |
| 不安全执行生成代码 | 中/极高 | 隔离、无网络、限额、审核、独立 workspace | 没有合格执行隔离时仅输出代码与审核结果 |
| 大模型成本和不稳定性 | 中/中 | 缓存 specs、生成次数预算、Provider 可替换 | 达到预算就终止并保留可人工继续的产物 |
| 过早扩展轮腿/ROS/GUI | 高/中 | 分阶段独立验收、可选依赖 | 机械臂与论文主线未验收时暂缓新方向 |
| 论文与模型许可 | 中/中 | 保存引用、许可、可再分发状态 | 无授权不得打包非许可论文全文/模型文件 |
| “真实复现”被夸大 | 中/高 | L0/L1/L2 分级和实验依据 | 只有 L1 证据就只对外说明迁移验证 |

---

## 12. Codex 执行原则与首轮任务

### 12.1 协作边界

- 一次只实现一张经过确认的任务卡；先读现有源码和文档，提出最小修改方案；
- 新功能均写对应合同、数学或 E2E 测试；实际运行测试后报告命令、成功/失败与未测试项；
- 设计决策（尤其 API 变更和信任边界）追加 `docs/adr/`；主要变更更新架构图与 README；
- AI 生成的内容标注来源与人工审核状态，算法基线单独冻结；
- 无明确授权不更改仓库结构、不发布、不接实机、不执行未经审核的外部代码；
- 先复用现有六轴算法与模型，但在没有源码和实测前将其记为 `to_be_verified`。

### 12.2 M0 的具体交付

1. `pyproject.toml`、源码布局、`robostudio list robots/plugins`、`robostudio validate robot`；
2. `RobotSpec / RobotState / RobotCommand / Pose / RunResult / Capability / SimulationBackend` 合同；
3. 一份有效和一份无效的机器人清单与 registry 测试；
4. deterministic fake backend，固定 seed 的 smoke test 和统一失败码；
5. 最小实验记录：`run_id / seed / scenario_hash / status / metrics`；
6. CI、README、ARCHITECTURE、ADR 0001；
7. **不在 M0 开发 PDF Agent、MuJoCo、机械臂 IK、ROS 或 GUI。**

### 12.3 可直接粘贴给 Codex 的第一条指令

> 请先完整阅读 `docs/PROJECT_PROPOSAL.md`、`AGENTS.md` 与 `tasks/M0_BOOTSTRAP.md`。目前仅执行 M0。先检查已有文件与可复用代码，提出最小分步实施计划，列出文件修改范围、接口合同与测试方案。按任务卡实现可运行的 Python package、typed contracts、manifest validation、registry、CLI、fake simulation backend、基础 run artifacts 和 CI。运行格式/类型/单元测试，逐项报告实际命令与结果。严禁提前实施 MuJoCo、论文解析、代码生成、复杂 IK、ROS 2 或 GUI。完成后提交 M1 的方案供人工审核。

---

## 13. 产品竞争力与招聘展示

企业真正应当看到：

1. **问题定义**：能把“这篇 IK 论文是否适用于这台机械臂？”变成可执行实验；
2. **机器人专业性**：坐标系、正逆运动学、限位、奇异性、时间预算与动力学理解；
3. **工程组织能力**：可替换模型/算法、明确接口、CI、隔离和数据留痕；
4. **技术判断力**：论文假设、AI 生成代码和统计结果之间能够区分证据和推断；
5. **交付能力**：别人按 README 能重复看到同一结论，面对失败能给出诊断。

**最好的 5 分钟演示**：将论文导入 → 展示来源页码和需审核的算法规格 → 生成受控候选插件 → 自动运行基准 → 展示相同难例下两种算法的轨迹、误差与失败原因 → 展示可复制的 replay 命令。第二个演示再展示不同机器人以相同框架运行任务。

**判断项目成功的标准**：实际有其他开发者愿意使用它来评价新论文，至少能复现你在公开报告中的 L0/L1 测试。仓库代码行数、支持机器人数量和 Agent 数量不作为主要目标。

---

## 14. 版本安排、历史项目的关系和下一步

- **v0.1**：多机器人仿真平台构想；
- **v0.2**：突出算法、工程验收与招聘价值；
- **v0.3（本版）**：新增论文算法复现与标准化实验链，明确有限范围、分级验证、自动代码执行安全和真实工作量。

**当前立刻执行的下一步**：

1. 将本文件放到仓库 `docs/PROJECT_PROPOSAL.md` 并创建 `AGENTS.md`/`tasks/M0_BOOTSTRAP.md`；
2. 使用 Codex 只完成 M0 工程底座；
3. 核对现有机械臂 MDH 参数、工具 TCP、DLS 源码和合法复用范围；
4. 完成 M1 与 M2 之后再选择第一篇 IK 论文，依据公式完整度和许可创建 P0 任务卡；
5. M0–P2 验收后发布第一份自带失败案例的可复现演示。

**尚待确认但不阻塞 M0**：操作系统、机器配置、Python 版本、是否已有可用 Git 仓库、选定第一篇论文、生成代码采用本地 Codex 还是 API、容器隔离能力和许可清单。

---

## 15. 参考资料（检索核实：2026-11-14）

1. [Google DeepMind MuJoCo Python Documentation](https://mujoco.readthedocs.io/en/stable/python.html) — 原生 Python 绑定、仿真数据与内存注意事项。
2. [ROS 2 Distributions（官方文档仓库）](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst) — Jazzy / Lyrical 等版本与支持生命周期。
3. [MoveIt 2 Motion Planning](https://moveit.picknik.ai/main/doc/concepts/motion_planning.html) — 规划器插件、约束、碰撞检查与轨迹。
4. [Paper2Code / PaperCoder（ICLR 2026）](https://github.com/going-doer/Paper2Code) — 论文到代码的多 Agent 研究与基准。
5. [OpenAI PaperBench（2025）](https://openai.com/index/paperbench/) — 科研论文自动复现的任务划分与评价方法。
6. [PaperBench 代码与评测工作流](https://github.com/openai/frontier-evals/blob/main/project/paperbench/README.md) — 执行与评估隔离的参考实现。

**说明**：外部参考用于架构选型与可行性论证，未证明 RoboStudio 自身已实现或经过性能验收。
