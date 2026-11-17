# 常见问题

## 下载后找不到 exe

请在 [Releases](https://github.com/THEr1stan/RoboStudio/releases/latest) 的 Assets 下载
`RoboStudio-0.8.0-win-x64.zip`。`Source code` 和 `RoboStudio-0.8.0-source.zip` 是开发者源码包。
完整解压后启动 exe，不要只移动 exe：旁边的 runtime、DLL、文档均属于程序。

## 缺少 WebView2 / .NET，或系统提示未签名

从 [微软 WebView2 下载页](https://developer.microsoft.com/en-us/microsoft-edge/webview2/)
安装 Evergreen Runtime，或从 [微软 .NET Framework 4.8 页](https://dotnet.microsoft.com/en-us/download/dotnet-framework/net48)
安装运行环境。安装后重新打开 App。本项目不自动安装系统组件。

当前便携程序没有商业代码签名，Windows 可能提示来源未知。先确认来自本仓库 Release，
再用 PowerShell 检查 `Get-FileHash .\RoboStudio-0.8.0-win-x64.zip -Algorithm SHA256`，
与同一 Release 的 `SHA256SUMS.txt` 对照。不需要关闭杀毒软件或系统安全设置。

## 只有文字 / 没有机械臂画面

不要直接打开 `src/robostudio/ui/.../*.html`。普通使用请启动 `RoboStudio.exe`。
开发时先运行 [开发说明](DEVELOPMENT.md) 中的 workbench 服务，再打开它给出的本地地址。
若 App 本身也不显示画面，确认 WebView2 正常、显卡驱动支持 WebGL 2，并查看启动错误或日志。
远程桌面、虚拟机和软件渲染环境的图形能力与速度可能不同。

## 启动失败，提示端口 18765 被占用

先关闭重复的 RoboStudio 窗口。App 不会强行接管已有服务，不要结束来源不明的系统进程。
开发者可用 `RoboStudio.exe --port 18767 --data-dir "D:\RoboStudio-TestData"` 创建独立测试实例；
地址改变后本地场景存储也不同，日常使用建议保留默认端口。

## 施力后没有恢复 / 物体拖不动

按 Esc 清除当前拖动或外力，确认没有暂停、没有停在“自由操控”；必要时点 **交给机械臂**。
橙色轮廓表示实际几何阻挡，可保持按住并滚轮抬高后绕开。
施力过大导致滑落、目标不可达或观测被遮挡时，恢复可能失败；查看失败原因后调整布局。
普通松手会保留任务阶段，结构参数修改或重新载入场景则会重建任务。

## 文件导入被拒绝 / 保存的场景没出现在另一个窗口

场景仅接受 RoboStudio 场景配置，不能导入运行记录或 URDF。查看 [场景格式](SCENES.md)。
本机命名列表属于当前 App 的 Data/WebView；浏览器、不同端口或不同 Data 目录不会共享列表。
用场景 JSON 导出 / 导入迁移，不要复制浏览器的整个个人资料。

## 算法对照失败 / 严格复算不是 IDENTICAL

失败是结果的一部分。先看目标可达性、关节限位、碰撞、迭代预算、初始状态和感知来源。
严格复算要求源码、模型和环境匹配，跨版本拒绝一致性是预期行为。
展示画面相似不等于数据逐字段相同；不要把回放动画当成严格复算通过。

## 如何报告问题

到 [Issues](https://github.com/THEr1stan/RoboStudio/issues) 提供：版本、Windows 版本、机器人与场景、
具体操作顺序、预期结果、实际结果，以及可分享的场景 JSON / 错误信息。
如能导出运行记录，请检查其中路径和自定义内容后再附上。不要提交令牌、整个 Data 目录或个人资料。
