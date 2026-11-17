using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;
using System.Windows.Forms;
using Microsoft.Web.WebView2.Core;
using Microsoft.Web.WebView2.WinForms;

[assembly: System.Reflection.AssemblyTitle("RoboStudio")]
[assembly: System.Reflection.AssemblyProduct("RoboStudio Desktop")]
[assembly: System.Reflection.AssemblyVersion("0.8.0.0")]

namespace RoboStudioDesktop
{
    internal sealed class Options
    {
        internal string Data = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Data");
        internal int Port = 18765;
        internal int DebugPort;
        internal bool NoDemo;
        internal static Options Parse(string[] args)
        {
            var result = new Options();
            for (int i = 0; i < args.Length; i++)
            {
                if (args[i] == "--no-demo") result.NoDemo = true;
                else if (args[i] == "--data-dir" && i + 1 < args.Length) result.Data = Path.GetFullPath(args[++i]);
                else if (args[i] == "--port" && i + 1 < args.Length) result.Port = int.Parse(args[++i]);
                else if (args[i] == "--debug-port" && i + 1 < args.Length) result.DebugPort = int.Parse(args[++i]);
                else throw new ArgumentException("Unknown or incomplete argument: " + args[i]);
            }
            if (result.Port < 1 || result.Port > 65535 || result.DebugPort < 0 || result.DebugPort > 65535)
                throw new ArgumentException("Port must be 1..65535.");
            return result;
        }
    }

    internal static class Program
    {
        [STAThread]
        private static void Main(string[] args)
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            try
            {
                var options = Options.Parse(args);
                Directory.CreateDirectory(options.Data);
                string key;
                using (var sha = SHA256.Create()) key = BitConverter.ToString(sha.ComputeHash(Encoding.UTF8.GetBytes(options.Data.ToUpperInvariant()))).Replace("-", "");
                bool created;
                using (var single = new Mutex(true, "Local\\RoboStudio-" + key, out created))
                {
                    if (!created)
                    {
                        IntPtr window = Native.FindWindow(null, "RoboStudio");
                        if (window != IntPtr.Zero) { Native.ShowWindow(window, 9); Native.SetForegroundWindow(window); }
                        return;
                    }
                    Application.Run(new StudioWindow(options));
                    single.ReleaseMutex();
                }
            }
            catch (Exception error)
            {
                MessageBox.Show("无法启动 RoboStudio。\n\n" + error.Message, "RoboStudio", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }

    internal sealed class StudioWindow : Form
    {
        private readonly Options options;
        private readonly WebView2 web = new WebView2();
        private readonly Label loading = new Label();
        private readonly JavaScriptSerializer json = new JavaScriptSerializer();
        private Process backend;
        private IntPtr job;
        private string origin;
        private string records;
        private bool closing;
        private readonly object logLock = new object();

        internal StudioWindow(Options value)
        {
            options = value;
            Text = "RoboStudio";
            Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath);
            StartPosition = FormStartPosition.CenterScreen;
            Size = new Size(1460, 960);
            WindowState = FormWindowState.Maximized;
            MinimumSize = new Size(1080, 720);
            BackColor = Color.FromArgb(245, 247, 250);
            var menu = new MenuStrip { BackColor = Color.White, Padding = new Padding(12, 4, 0, 4) };
            var file = new ToolStripMenuItem("文件");
            file.DropDownItems.Add("导出文件夹", null, delegate { OpenFolder(Path.Combine(options.Data, "Exports")); });
            file.DropDownItems.Add("实验记录文件夹", null, delegate { OpenFolder(records); });
            file.DropDownItems.Add("选择实验记录文件夹…", null, SelectRecords);
            file.DropDownItems.Add(new ToolStripSeparator());
            file.DropDownItems.Add("退出", null, delegate { Close(); });
            var views = new ToolStripMenuItem("工作区");
            views.DropDownItems.Add("机械臂工作站", null, delegate { Navigate("/"); });
            views.DropDownItems.Add("算法实验室", null, delegate { Navigate("/?page=algorithms&demo=0"); });
            views.DropDownItems.Add("论文规格与复现", null, delegate { Navigate("/workcell"); });
            views.DropDownItems.Add("关节与基础验证", null, delegate { Navigate("/advanced"); });
            var help = new ToolStripMenuItem("帮助");
            help.DropDownItems.Add("使用说明", null, delegate { ShowGuide(); });
            help.DropDownItems.Add("关于 RoboStudio", null, delegate {
                MessageBox.Show(this, "RoboStudio 0.8.0 · Windows Desktop\n\nPython + MuJoCo：算法与物理\nJavaScript + WebGL：交互与三维画面\nC# + WebView2：桌面窗口\n\n场景、数据与日志：\n" + options.Data + "\n\nSimulation-only · 无真机控制", "关于 RoboStudio", MessageBoxButtons.OK, MessageBoxIcon.Information);
            });
            menu.Items.AddRange(new ToolStripItem[] { file, views, help });
            MainMenuStrip = menu;
            web.Dock = DockStyle.Fill;
            web.Visible = false;
            loading.Dock = DockStyle.Fill;
            loading.TextAlign = ContentAlignment.MiddleCenter;
            loading.Font = new Font("Microsoft YaHei UI", 16);
            loading.Text = "RoboStudio\n\n正在准备仿真工作环境…";
            Controls.Add(web); Controls.Add(loading); Controls.Add(menu);
            Shown += async delegate { await StartAsync(); };
            FormClosing += OnClosing;
        }

        private void Log(string text)
        {
            if (String.IsNullOrEmpty(text)) return;
            lock (logLock) File.AppendAllText(Path.Combine(options.Data, "desktop.log"), DateTime.UtcNow.ToString("o") + " " + text + Environment.NewLine, Encoding.UTF8);
        }

        private async Task StartAsync()
        {
            try
            {
                CoreWebView2Environment.GetAvailableBrowserVersionString();
                records = Path.Combine(options.Data, "Experiments");
                string settings = Path.Combine(options.Data, "settings.json");
                if (File.Exists(settings))
                {
                    var saved = json.Deserialize<Dictionary<string, object>>(File.ReadAllText(settings, Encoding.UTF8));
                    if (saved.ContainsKey("recordsDirectory")) records = Path.GetFullPath((string)saved["recordsDirectory"]);
                }
                Directory.CreateDirectory(records);
                string python = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "runtime", "python.exe");
                if (!File.Exists(python)) throw new FileNotFoundException("运行环境不完整。请解压完整的 RoboStudio Windows 便携包，不要只复制 exe。", python);
                job = Native.CreateKillJob();
                backend = new Process();
                backend.StartInfo = new ProcessStartInfo(python, "-I -X utf8 -m robostudio.desktop_host --port " + options.Port + " --output \"" + records + "\"") {
                    UseShellExecute = false, CreateNoWindow = true,
                    RedirectStandardInput = true, RedirectStandardOutput = true, RedirectStandardError = true,
                    WorkingDirectory = AppDomain.CurrentDomain.BaseDirectory, StandardOutputEncoding = Encoding.UTF8, StandardErrorEncoding = Encoding.UTF8
                };
                backend.StartInfo.EnvironmentVariables["PYTHONIOENCODING"] = "utf-8";
                backend.ErrorDataReceived += delegate(object sender, DataReceivedEventArgs e) { Log(e.Data); };
                backend.EnableRaisingEvents = true;
                backend.Exited += delegate {
                    if (!closing && IsHandleCreated) BeginInvoke((Action)delegate {
                        if (closing) return;
                        web.Visible = false; loading.Visible = true;
                        loading.Text = "仿真服务已停止。\n请关闭并重新打开 RoboStudio。\n日志位于 Data / desktop.log。";
                    });
                };
                backend.Start();
                if (!Native.AssignProcessToJobObject(job, backend.Handle)) throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
                backend.BeginErrorReadLine();
                var readiness = backend.StandardOutput.ReadLineAsync();
                if (await Task.WhenAny(readiness, Task.Delay(20000)) != readiness) throw new TimeoutException("仿真服务启动超时。请查看 Data / desktop.log。");
                string line = await readiness;
                if (line == null) throw new InvalidOperationException("仿真服务未启动。端口 " + options.Port + " 可能被占用；请查看 Data / desktop.log。");
                var ready = json.Deserialize<Dictionary<string, object>>(line);
                origin = "http://127.0.0.1:" + options.Port;
                if ((string)ready["url"] != origin) throw new InvalidDataException("Unexpected backend address.");
                File.WriteAllText(Path.Combine(options.Data, "session.json"), json.Serialize(new { appPid = Process.GetCurrentProcess().Id, backendPid = ready["pid"], url = origin, recordsDirectory = records }), Encoding.UTF8);

                var environmentOptions = new CoreWebView2EnvironmentOptions();
                environmentOptions.AreBrowserExtensionsEnabled = false;
                if (options.DebugPort != 0) environmentOptions.AdditionalBrowserArguments = "--remote-debugging-address=127.0.0.1 --remote-debugging-port=" + options.DebugPort;
                var environment = await CoreWebView2Environment.CreateAsync(null, Path.Combine(options.Data, "WebView"), environmentOptions);
                if (closing) return;
                await web.EnsureCoreWebView2Async(environment);
                web.CoreWebView2.Settings.AreDevToolsEnabled = options.DebugPort != 0;
                web.CoreWebView2.Settings.AreDefaultContextMenusEnabled = false;
                web.CoreWebView2.Settings.IsStatusBarEnabled = false;
                web.CoreWebView2.Settings.IsPasswordAutosaveEnabled = false;
                web.CoreWebView2.Settings.IsGeneralAutofillEnabled = false;
                web.CoreWebView2.Settings.IsWebMessageEnabled = false;
                web.CoreWebView2.Settings.AreHostObjectsAllowed = false;
                web.CoreWebView2.NavigationStarting += delegate(object sender, CoreWebView2NavigationStartingEventArgs e) {
                    // Export links use Blob URLs owned by the current workbench origin.
                    if (e.Uri.StartsWith("blob:" + origin + "/", StringComparison.Ordinal)) return;
                    Uri target;
                    if (!Uri.TryCreate(e.Uri, UriKind.Absolute, out target) || target.GetLeftPart(UriPartial.Authority) != origin) { e.Cancel = true; Log("Blocked external navigation: " + e.Uri); }
                };
                web.CoreWebView2.NewWindowRequested += delegate(object sender, CoreWebView2NewWindowRequestedEventArgs e) {
                    e.Handled = true;
                    Uri target;
                    if (Uri.TryCreate(e.Uri, UriKind.Absolute, out target) && target.GetLeftPart(UriPartial.Authority) == origin) web.CoreWebView2.Navigate(e.Uri);
                };
                web.CoreWebView2.PermissionRequested += delegate(object sender, CoreWebView2PermissionRequestedEventArgs e) { e.State = CoreWebView2PermissionState.Deny; };
                web.CoreWebView2.DownloadStarting += DownloadStarting;
                web.CoreWebView2.ProcessFailed += delegate(object sender, CoreWebView2ProcessFailedEventArgs e) {
                    Log("WebView process failure: " + e.ProcessFailedKind);
                    if (!closing && (e.ProcessFailedKind == CoreWebView2ProcessFailedKind.BrowserProcessExited || e.ProcessFailedKind == CoreWebView2ProcessFailedKind.RenderProcessExited)) {
                        web.Visible = false; loading.Visible = true;
                        loading.Text = "三维界面进程已停止。\n请关闭并重新打开 RoboStudio。\n已保存的场景与实验文件会保留。";
                    }
                };
                web.CoreWebView2.NavigationCompleted += delegate(object sender, CoreWebView2NavigationCompletedEventArgs e) {
                    if (e.IsSuccess) { loading.Visible = false; web.Visible = true; web.Focus(); }
                    else { loading.Text = "界面载入失败：" + e.WebErrorStatus + "\n请关闭后重新打开。"; Log(loading.Text); }
                };
                Navigate(options.NoDemo ? "/?demo=0" : "/");
            }
            catch (Exception error)
            {
                Log(error.ToString());
                StopBackend();
                if (closing) return;
                loading.Visible = true; web.Visible = false;
                loading.Text = "无法启动 RoboStudio\n\n" + error.Message;
                if (error is WebView2RuntimeNotFoundException) loading.Text += "\n\n请安装 Microsoft Edge WebView2 Evergreen Runtime（x64）。\n下载地址见随附的 README-桌面版.txt。";
            }
        }

        private void DownloadStarting(object sender, CoreWebView2DownloadStartingEventArgs e)
        {
            Log("Download starting: " + Path.GetFileName(e.ResultFilePath));
            string name = Path.GetFileName(e.ResultFilePath);
            if (String.IsNullOrEmpty(name)) name = "RoboStudio-export.json";
            string folder = Path.Combine(options.Data, "Exports");
            Directory.CreateDirectory(folder);
            string destination = Path.Combine(folder, name);
            for (int number = 1; File.Exists(destination); number++) destination = Path.Combine(folder, Path.GetFileNameWithoutExtension(name) + " (" + number + ")" + Path.GetExtension(name));
            e.ResultFilePath = destination;
            // Keep the native WebView download panel, including progress and Show in folder.
            e.Handled = false;
        }

        private void Navigate(string path)
        {
            if (!closing && web.CoreWebView2 != null && origin != null) web.CoreWebView2.Navigate(origin + path);
        }

        private void OpenFolder(string path)
        {
            if (String.IsNullOrEmpty(path)) return;
            Directory.CreateDirectory(path);
            Process.Start(new ProcessStartInfo("explorer.exe", "\"" + path + "\"") { UseShellExecute = true });
        }

        private void SelectRecords(object sender, EventArgs e)
        {
            using (var dialog = new FolderBrowserDialog { Description = "选择实验记录目录（不会移动或删除文件；下次启动生效）", SelectedPath = records })
            {
                if (dialog.ShowDialog(this) != DialogResult.OK) return;
                File.WriteAllText(Path.Combine(options.Data, "settings.json"), json.Serialize(new { recordsDirectory = dialog.SelectedPath }), Encoding.UTF8);
                MessageBox.Show(this, "已保存。下次启动将使用所选目录，现有实验文件会保留。", "实验记录目录");
            }
        }

        private void ShowGuide()
        {
            MessageBox.Show(this, "直接拖动积木、末端或连杆，与真实物理任务交互。\n顶部选择机器人、任务与算法；场景库可载入工业工位。\n\n导出文件保存在 Data / Exports，菜单“文件”可直接打开。\n“本机场景”保存在本 App 内，退出后仍会保留。\n旧浏览器场景：先导出 JSON，再在 App 场景库中导入。\n旧实验目录可在“文件 → 选择实验记录文件夹”中接入。\n\n关闭窗口会结束本次仿真；需要保留时，请先在界面保存运行记录。\n\n完整功能与边界见随附 docs / USER_GUIDE.md。", "使用说明", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }

        private void StopBackend()
        {
            try { if (backend != null && !backend.HasExited) backend.StandardInput.Close(); } catch (InvalidOperationException) { }
            // Closing this job also handles force-close/crash and all owned descendants.
            if (job != IntPtr.Zero) { Native.CloseHandle(job); job = IntPtr.Zero; }
            if (backend != null) { backend.Dispose(); backend = null; }
        }

        private void OnClosing(object sender, FormClosingEventArgs e)
        {
            closing = true;
            Log("Closing window");
            StopBackend();
            Log("Backend stopped");
            web.Dispose();
            Log("WebView disposed");
            string session = Path.Combine(options.Data, "session.json");
            if (File.Exists(session)) File.Delete(session);
        }
    }

    internal static class Native
    {
        [DllImport("user32.dll", CharSet = CharSet.Unicode)] internal static extern IntPtr FindWindow(string className, string windowName);
        [DllImport("user32.dll")] internal static extern bool SetForegroundWindow(IntPtr window);
        [DllImport("user32.dll")] internal static extern bool ShowWindow(IntPtr window, int command);
        [DllImport("kernel32.dll", SetLastError = true)] private static extern IntPtr CreateJobObject(IntPtr attributes, string name);
        [DllImport("kernel32.dll", SetLastError = true)] private static extern bool SetInformationJobObject(IntPtr job, int informationClass, IntPtr information, uint length);
        [DllImport("kernel32.dll", SetLastError = true)] internal static extern bool AssignProcessToJobObject(IntPtr job, IntPtr process);
        [DllImport("kernel32.dll")] internal static extern bool CloseHandle(IntPtr handle);
        [StructLayout(LayoutKind.Sequential)] private struct BasicLimits {
            internal long PerProcessUserTimeLimit, PerJobUserTimeLimit;
            internal uint LimitFlags;
            internal UIntPtr MinimumWorkingSetSize, MaximumWorkingSetSize;
            internal uint ActiveProcessLimit;
            internal UIntPtr Affinity;
            internal uint PriorityClass, SchedulingClass;
        }
        [StructLayout(LayoutKind.Sequential)] private struct IoCounters { internal ulong ReadOperationCount, WriteOperationCount, OtherOperationCount, ReadTransferCount, WriteTransferCount, OtherTransferCount; }
        [StructLayout(LayoutKind.Sequential)] private struct ExtendedLimits {
            internal BasicLimits Basic;
            internal IoCounters Io;
            internal UIntPtr ProcessMemoryLimit, JobMemoryLimit, PeakProcessMemoryUsed, PeakJobMemoryUsed;
        }
        internal static IntPtr CreateKillJob()
        {
            IntPtr job = CreateJobObject(IntPtr.Zero, null);
            if (job == IntPtr.Zero) throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
            var limits = new ExtendedLimits(); limits.Basic.LimitFlags = 0x2000;
            int size = Marshal.SizeOf(limits);
            IntPtr memory = Marshal.AllocHGlobal(size);
            try {
                Marshal.StructureToPtr(limits, memory, false);
                if (!SetInformationJobObject(job, 9, memory, (uint)size)) { int error = Marshal.GetLastWin32Error(); CloseHandle(job); throw new System.ComponentModel.Win32Exception(error); }
            } finally { Marshal.FreeHGlobal(memory); }
            return job;
        }
    }
}
