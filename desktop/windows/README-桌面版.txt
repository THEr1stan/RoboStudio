RoboStudio 0.8.0 · Windows x64 便携版

1. 将整个文件夹解压到可写目录（例如 D:\Apps\RoboStudio）。
2. 双击 RoboStudio.exe。无需 Python、终端或浏览器标签页。
3. 顶部选择机械臂、任务和算法；“场景库”提供可运行的工业工位。

请保留 runtime、DLL 和 exe 在同一文件夹，不要只复制 exe。
需要 Windows 10/11 x64、.NET Framework 4.8 和 WebView2 Evergreen Runtime。
多数 Windows 11 电脑已具备这些组件；本包不自动下载或安装系统组件。
WebView2 官方下载：https://developer.microsoft.com/microsoft-edge/webview2/

数据位于 App 旁的 Data 文件夹：
- Exports：场景 JSON、算法 JSON、CSV 等导出文件（重名自动编号）。
- Experiments：完整实验运行记录。
- WebView：App 独立的命名场景与窗口存储。
- desktop.log：启动诊断。
升级时保留整个 Data 文件夹。便携包只包含程序，不包含他人的数据。

旧浏览器命名场景：在原工作台导出 JSON，再到 App 的场景库导入。
旧实验：菜单“文件 → 选择实验记录文件夹”，选原项目 artifacts，重启生效。
本机原项目启动器会直接打开此 App；网页入口仍作为可选开发工具保留。

程序用自己的 127.0.0.1:18765 端口连接本地仿真；不会打开浏览器。
关闭窗口会结束本次仿真服务。它不会停止你已有的网页工作台服务。
如需保存运行过程，请在关闭前使用界面中的“保存运行记录”。

完整功能说明：docs/USER_GUIDE.md
第一次使用：docs/QUICK_START.md
自定义场景：docs/SCENES.md
问题排查：docs/TROUBLESHOOTING.md
项目结构与构建：docs/DEVELOPMENT.md
当前模拟范围和限制以说明为准。无真机控制，无自动论文代码执行。

原项目使用 MIT 许可证，见 LICENSE；第三方许可见 THIRD_PARTY_NOTICES.md。
项目主页：https://github.com/THEr1stan/RoboStudio
