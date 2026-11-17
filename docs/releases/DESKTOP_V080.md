# v0.8.0 — 桌面 App 与仓库整理

交付日期：2026-11-17。范围是桌面化、目录整理与功能保持；没有更换算法或物理模型。

## 现在能做什么

双击 `RoboStudio.exe`，在独立 Windows 窗口中使用原工作站。无需打开浏览器、启动终端
或另装 Python。顶部保留机器人、任务、算法和场景库；菜单“工作区”保留论文规格及基础验证。
窗口内实际执行了接触抓取、施力、场景文件导入导出、DLS 等四种逆解对照与严格复算。

重复双击聚焦既有窗口。正常关闭和强制结束父进程均释放其仿真进程与端口。
桌面场景存储位于 `Data/WebView`，重启后保留；导出文件位于 `Data/Exports`，重名自动编号。
旧浏览器命名场景可通过 JSON 迁移，原实验目录可在“文件”菜单接入。
本机启动器已切换到桌面 App；网页开发入口仍可用。

## 文件与设计

- `desktop/windows`：C# / WinForms / WebView2 窗口、图标、清单、固定 SDK 版本与哈希。
- `src/robostudio/desktop_host.py`：独立服务就绪握手、父进程生存期与启动失败报告。
- `src/robostudio/simulation`：12 个现场模块迁入；原 `studio_*` 导入是同一模块对象的兼容别名。
- `src/robostudio/ui`：当前界面、历史界面与共享资源分组；内容与原版一致，HTTP 路由不变。
- `scripts/build`：桌面构建、离线 Python 运行环境打包、允许清单式源码导出。
- `scripts/validation`、`tests/browser`：验证工具集中；移除机器专属浏览器路径。
- `docs`：当前指南与架构说明；公开快照仅保留本版本交付，早期资料保存在原开发工作区。
- `examples/scenes`：四个可直接导入的工业 JSON。模型、核心合同与数值算法未改写。

新依赖仅微软官方 WebView2 SDK 1.0.4314.35，已获用户同意，下载包 SHA-256：
`90778deb1b77e36292828b4b5833c9255c27533f40e0ddd55891ae718944edc4`。
使用系统 Evergreen Runtime；包内带 Python 3.12.14、MuJoCo Core 4.0.1、NumPy 2.5.4。

## 实际验证

| 检查 | 命令或入口 | 结果 |
|---|---|---|
| Python 3.15 全回归 | `.venv/Scripts/python.exe -m pytest -q` | 358 passed，203.15 s，0 skipped |
| Python 3.12 最终全回归 | `PYTHONPATH=src`，`.venv-check/Scripts/python.exe -m pytest -q` | 358 passed，231.19 s，0 skipped |
| 最终迁移与进程合同 | `pytest tests/integration/test_desktop_host.py tests/integration/test_layout_migration.py -q` | 60 passed；两个 Python 版本均通过 |
| Lint / 格式 | `ruff check src tests scripts/build`；`ruff format --check src tests scripts/build` | 通过，82 文件 |
| 类型 | `mypy src/robostudio` | 51 源文件通过 |
| JS 数值 | `node --test tests/web/kinematics.test.mjs` | 5 passed，0 skipped |
| 场景浏览器回归 | `node tests/browser/scenes.cjs` | 10 检查项通过 |
| 施力浏览器回归 | `node tests/browser/force.cjs` | 5 检查项通过 |
| 算法浏览器回归 | `node tests/browser/algorithms.cjs` | 14 检查项通过 |
| 原生 WebView2 | `node tests/browser/desktop.cjs`，仅测试时启用 CDP | 8 检查项通过，0 页面错误 |
| 重启与导航 | `node tests/browser/desktop_persistence.cjs` | 2 检查项通过 |
| 窗口生命周期 | `scripts/validation/verify_desktop.ps1` | 正常退出、重启存储、重复启动、强制退出 4 项通过 |
| 构建 | `scripts/build/windows.ps1`；`pip wheel --no-deps --no-build-isolation .` | Windows exe、内置运行环境、Python wheel 成功 |
| 分发一致性 | 源码 / App 内置包 / wheel 逐文件比较 | 75 个运行文件一致，无旧 web 目录残留 |
| 干净源码包 | 解压源码 ZIP 后运行合同、迁移、进程测试，再构建 wheel | 110 passed，3.65 s；独立构建成功 |

四个工业场景用同一配置与种子，在冻结的 0.7.6 和重组后的代码各执行一次，再严格复算：

| 场景 | 完成放置 | 非预期碰撞步 | 迁移前后比较 |
|---|---:|---:|---|
| 圆柱上下料 | 1 | 0 | 物理帧、规划、指标与放置结果一致 |
| 来料分拣 | 3 | 0 | 同上 |
| 三层码垛 | 3 | 0 | 同上 |
| 隔板转运 | 1 | 0 | 同上 |

8 次放置均完成；两版各自严格复算为 IDENTICAL。这里比较真实物理输出，未把改动后的源码哈希
伪装成旧版哈希。App 内另完成一次接触抓取、保存与 IDENTICAL 复算。

最终 Python 源码哈希：`538df6d7fb53aa660a8772bd245f24beb4d0491ff03b2b6d978c8dd81184c782`。
本地原始证据位于 `artifacts/desktop-v080`，包含测试日志、物理对照、截图与分发清单；
实验数据和机器日志不进入源码发布包。早期检查暴露了 Python AST 输出格式与 Windows
文本编码差异，已修正测试兼容性；最终结果如上。桌面测试显式恢复原生下载策略，并等待异步保存完成。

## 边界与未完成事项

GitHub 最新快照另执行了以下检查（2026-11-17）：

| 检查 | 实际结果 |
|---|---|
| `python -m pytest -q`，Windows Python 3.15 | 361 passed，182.58 s，0 skipped |
| `ruff check src tests scripts/build` / `ruff format --check src tests scripts/build` | 通过，83 文件 |
| `mypy src/robostudio` | 51 文件通过 |
| `node --test tests/web/kinematics.test.mjs` | 5 passed，0 skipped |
| 干净源码解压后：`pytest tests/contracts tests/integration/test_layout_migration.py tests/integration/test_desktop_host.py -q` | 113 passed，3.61 s（最终清单） |
| 干净源码：`powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build/windows.ps1 -ShellOnly` | 编译成功；固定 SDK 已按哈希验证 |
| 干净源码：`pip wheel --no-deps --no-build-isolation .` | 构建成功；MIT 许可元数据与许可文件齐全 |
| 发布清单 / 文档链接 | 196 个源文件，52 个相对链接，无断链；历史交付和本机资料被排除 |

新增 3 个分发合同检查许可证、入门资料、历史与私有数据排除；先观察缺少文件与旧文档混入
导致的失败，再修改打包规则。界面迁移检查仅允许版本标识变更，其余字节继续对照冻结基线。
本次没有重新执行全部图形自动化验收；上文的桌面 / 浏览器结果属于同一实现的前次功能验收。
GitHub Actions 的实际状态以仓库运行页为准，本地通过不等于已通过远程矩阵。

Windows 10/11 x64 便携 App，尚无安装器、代码签名、自动升级、macOS/Linux 桌面包。
需要系统 .NET Framework 4.8 与 WebView2；不自动联网安装系统组件。独立窗口内部采用 WebView，
并未重写为原生三维引擎。固定本地端口被其他程序占用时显示错误，不接管其他服务。
启动测试曾在受限自动化桌面中导致图形进程退出，最终在正常 Windows 桌面会话完成原生验收。

关闭窗口不自动保存尚未保存的实验；持久场景与已保存记录保留。程序物理步按请求推进，非硬实时。
旧记录的严格复算仍需要对应源码版本。Paper-to-Simulation 仍保留人工审核与隔离边界，无自动执行生成代码。
2026-11-17 所有者确认以 THEr1stan 署名采用 MIT，并授权发布公开仓库与当前版 Release。
打包产物不包含本机 Data、视频和虚拟环境。GitHub 发布补充入门、场景、排错与贡献指南，
只更新界面版本标识为 0.8.0；算法和物理模型保持不变。上表为桌面化验收记录，
不能理解为每次文档修改都重新执行了全部图形验收。

下一项最小任务：在第二台干净的 Windows 机器验收 Release 下载、启动与首次抓取。
