# 开发、打包与仓库使用

## 分层

Python 包在 `src/robostudio`，当前现场逻辑集中于 `simulation`；`core` 不引入引擎或界面。
界面资源在 `ui/studio`，保留页面在 `ui/legacy`。HTTP 资源路径和 API 没有变化。
旧 `robostudio.studio_*` 模块是显式兼容别名，与新实现共享同一模块对象，避免出现两份状态。
新代码使用 `robostudio.simulation.api` 等路径。迁移合同冻结实现语法树、算法和模型字节。

Windows 桌面壳在 `desktop/windows`，独立于 Python 包。它启动自己拥有的
`robostudio.desktop_host` 子进程；标准输出只传就绪消息，标准输入 EOF 表示父进程结束。
Windows Job Object 会在窗口进程结束时清理其仿真子进程树。原网页服务不受影响。
默认本地端口为 18765，固定地址使命名场景在重启后保持；端口冲突时显示错误，不接管未知服务。

## Python 开发与测试

先安装 Python 3.12 x64 与 Git；JavaScript 数值测试需要 Node.js 22+。
普通 App 用户不需要这些工具。以下命令在 Windows PowerShell 的仓库根目录执行。

```powershell
git clone https://github.com/THEr1stan/RoboStudio.git
cd RoboStudio
```

```powershell
python -m venv .venv
.venv/Scripts/python.exe -m pip install -r requirements-dev.lock
.venv/Scripts/python.exe -m pip install --no-build-isolation -e ".[physics]"
.venv/Scripts/python.exe -m ruff check src tests
.venv/Scripts/python.exe -m ruff format --check src tests
.venv/Scripts/python.exe -m mypy src/robostudio
.venv/Scripts/python.exe -m pytest -q
node --test tests/web/kinematics.test.mjs
```

浏览器测试集中在 `tests/browser`；需要已有 Playwright 和 Edge / Chromium。
默认从 Node 模块环境查找 `playwright`；可用 `PLAYWRIGHT_MODULE` 指定已有安装，
`BROWSER_PATH` 指定浏览器，`STUDIO_URL` 指向独立的测试服务（默认 8766）。
测试设置不再包含开发者机器的绝对路径。不在实际使用中的现场运行场景测试。

```powershell
.venv/Scripts/python.exe -m robostudio workbench --port 8766 --output artifacts/browser-check
# 另一个终端，在项目根目录运行：
node tests/browser/scenes.cjs
node tests/browser/industrial.cjs
node tests/browser/force.cjs
node tests/browser/algorithms.cjs
```

固定物理案例：`scripts/validation/verify_industrial_scenes.py` 等。
历史记录中的 `scripts/replay_world.py` 和 `scripts/replay_lab.py` 路径继续保留。

## Windows 桌面构建

构建端需要 .NET Framework 4.8 的 C# 编译器，以及 **CPython 3.12 x64** 环境。
在该环境安装现有固定依赖 `mujoco-core==4.0.1`、`numpy==2.5.4`。
不用 Electron、Node 打包工具或 PyInstaller；不从宿主全局环境隐式查找 Python。
SDK 仅从锁文件中的微软 NuGet URL 获取并核对 SHA-256；运行时不联网获取组件。

```powershell
py -3.12 -m venv .venv-check
.venv-check/Scripts/python.exe -m pip install -r requirements-physics.lock
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build/windows.ps1 `
  -Python .venv-check/Scripts/python.exe -RestoreDependency
```

输出 `dist/RoboStudio-0.8.0-win-x64/RoboStudio.exe`。
完整打包需要新的输出目录，防止陈旧文件混入；仅修改桌面壳时可使用 `-ShellOnly`。
便携包包括 CPython 标准库、原项目源码、NumPy、MuJoCo 及许可文件；不包含开发工具与用户数据。
Python 版本、依赖版本写入 `runtime-manifest.json`。缺少系统 WebView2 时显示安装指引。
普通运行未启用调试端口；`--debug-port` 仅供本机桌面验收使用。

原生验收时先启动独立测试窗口：

```powershell
./dist/RoboStudio-0.8.0-win-x64/RoboStudio.exe --port 18766 --debug-port 9223 --no-demo --data-dir "$PWD/artifacts/desktop-v080/native-test-data"
# 等测试窗口显示工作站后，使用已有的 Playwright：
node tests/browser/desktop.cjs
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/validation/verify_desktop.ps1
```

这里的 Data 是专用测试资料；窗口验收会关闭和重开它，不能指向实际使用中的场景库。

数据默认写入 App 旁的 `Data`，因此请将整个目录放在可写位置。
导出文件自动进入 `Data/Exports`，重名追加编号。JSON 仍使用原来的 schema 与校验。
原实验文件保留在项目 `artifacts`；本机可在“文件”菜单接入该目录。
浏览器本地场景需 JSON 迁移；不直接复制或读取浏览器个人资料。
升级保留 `Data`；不得把它打进公开包。

## GitHub 准备

```powershell
.venv/Scripts/python.exe scripts/build/source_bundle.py
```

得到 `dist/RoboStudio-0.8.0-source.zip` 与清单。只导出明确允许的源文件：
`src`、`desktop`、`tests`、`scripts`、`docs`、`tasks`、`examples`、CI 和根配置文件。
不会包含 `.venv*`、`artifacts`、`build`、`dist`、`Data`、缓存、运行日志、视频或私有配置。
源码包可解压到新的 Git 仓库，按 README 安装并运行。二进制 App 包适合放 Releases，
不要提交到源码历史。原代码采用 MIT，第三方许可仍分别保留。
导出会跳过 docs/history、tasks/HISTORY.md 与旧版交付文档；公开快照保留当前需要的
兼容入口和冻结测试。只有当前版源码与 Windows 便携包进入 Release。

完整便携 ZIP 可用 `python scripts/build/portable_bundle.py` 生成；脚本排除 `Data`、日志与缓存。
上传前请对照生成的 manifest，确认 LICENSE、第三方声明、运行环境与当前文档齐全。
CI 会执行 Python/JS 数值与集成验证、Windows 壳编译；桌面交互测试需要图形会话，不能用
CI 绿色状态代替桌面手工 / 自动化验收。当前实测平台与结果见 [交付记录](releases/DESKTOP_V080.md)。

## 版本兼容

目录迁移会改变源码哈希，新记录使用 0.8.0 的身份。旧记录可浏览与导出，但严格复算不应
声称跨源码版本相同；如需逐字段严格复算旧记录，使用对应旧版环境。
迁移验收另以冻结的 0.7.6 比较物理帧、指标、规划与放置结果，独立于版本和源码路径元数据。
