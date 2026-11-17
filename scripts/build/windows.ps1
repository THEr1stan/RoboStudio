param(
    [string]$Python = '.venv-check/Scripts/python.exe',
    [string]$Output = 'dist/RoboStudio-0.8.0-win-x64',
    [switch]$ShellOnly,
    [switch]$RestoreDependency
)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
Set-Location -LiteralPath $repo
$target = [IO.Path]::GetFullPath((Join-Path $repo $Output))
$distRoot = [IO.Path]::GetFullPath((Join-Path $repo 'dist')) + [IO.Path]::DirectorySeparatorChar
if (-not $target.StartsWith($distRoot, [StringComparison]::OrdinalIgnoreCase)) { throw 'Output must stay inside the repository dist directory.' }
$dependency = (Get-Content -Raw desktop/windows/dependencies.json | ConvertFrom-Json).webview2
$cache = Join-Path $repo ('build/dependencies/webview2-' + $dependency.version)
$archive = $cache + '.zip'
if (-not (Test-Path -LiteralPath $archive)) {
    if (-not $RestoreDependency) { throw 'Approved SDK missing. Re-run with -RestoreDependency to download the pinned Microsoft SDK.' }
    New-Item -ItemType Directory -Force (Split-Path $archive) | Out-Null
    Invoke-WebRequest -Uri $dependency.url -OutFile $archive
}
if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $dependency.sha256) { throw 'WebView2 package checksum mismatch.' }
Expand-Archive -LiteralPath $archive -DestinationPath $cache -Force
New-Item -ItemType Directory -Force $target | Out-Null
$compiler = Join-Path $env:WINDIR 'Microsoft.NET/Framework64/v4.0.30319/csc.exe'
if (-not (Test-Path -LiteralPath $compiler)) { throw '.NET Framework 4.8 / C# compiler is required on the Windows build machine.' }
$arguments = @('/nologo', '/target:winexe', '/platform:x64', '/optimize+', '/warnaserror', '/utf8output',
    ('/out:' + (Join-Path $target 'RoboStudio.exe')),
    ('/win32manifest:' + (Join-Path $repo 'desktop\windows\app.manifest')), ('/win32icon:' + (Join-Path $repo 'desktop\windows\Assets\app.ico')),
    '/reference:System.dll', '/reference:System.Core.dll', '/reference:System.Drawing.dll',
    '/reference:System.Windows.Forms.dll', '/reference:System.Web.Extensions.dll',
    ('/reference:' + (Join-Path $cache 'lib/net462/Microsoft.Web.WebView2.Core.dll')),
    ('/reference:' + (Join-Path $cache 'lib/net462/Microsoft.Web.WebView2.WinForms.dll')),
    (Join-Path $repo 'desktop\windows\Program.cs'))
& $compiler @arguments
if ($LASTEXITCODE -ne 0) { throw 'Desktop shell compilation failed.' }
Copy-Item -LiteralPath (Join-Path $cache 'lib/net462/Microsoft.Web.WebView2.Core.dll') -Destination $target
Copy-Item -LiteralPath (Join-Path $cache 'lib/net462/Microsoft.Web.WebView2.WinForms.dll') -Destination $target
Copy-Item -LiteralPath (Join-Path $cache 'runtimes/win-x64/native/WebView2Loader.dll') -Destination $target
New-Item -ItemType Directory -Force (Join-Path $target 'licenses') | Out-Null
Copy-Item -LiteralPath (Join-Path $cache 'LICENSE.txt') -Destination (Join-Path $target 'licenses/WebView2-LICENSE.txt')
Copy-Item -LiteralPath (Join-Path $cache 'NOTICE.txt') -Destination (Join-Path $target 'licenses/WebView2-NOTICE.txt')
Copy-Item -LiteralPath 'desktop/windows/README-桌面版.txt' -Destination $target
Copy-Item -LiteralPath 'THIRD_PARTY_NOTICES.md' -Destination $target
Copy-Item -LiteralPath 'LICENSE' -Destination $target
if (-not $ShellOnly) {
    & $Python scripts/build/package_runtime.py --destination $target
    if ($LASTEXITCODE -ne 0) { throw 'Python runtime packaging failed.' }
    Copy-Item -LiteralPath docs -Destination (Join-Path $target 'docs') -Recurse
    Copy-Item -LiteralPath examples -Destination (Join-Path $target 'examples') -Recurse
}
Write-Output ('Built: ' + (Join-Path $target 'RoboStudio.exe'))
