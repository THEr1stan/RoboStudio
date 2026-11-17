param([string]$App = 'dist/RoboStudio-0.8.0-win-x64/RoboStudio.exe')
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
Set-Location -LiteralPath $root
$appPath = [IO.Path]::GetFullPath((Join-Path $root $App))
$dataPath = Join-Path $root 'artifacts/desktop-v080/native-test-data'
$sessionFile = Join-Path $dataPath 'session.json'
$checks = [Collections.Generic.List[string]]::new()
function Test-Port([int]$port) {
    $client = [Net.Sockets.TcpClient]::new()
    try { $client.Connect('127.0.0.1', $port); return $true } catch { return $false } finally { $client.Dispose() }
}
function Read-Session { return Get-Content -LiteralPath $sessionFile -Raw | ConvertFrom-Json }
function Start-TestApp {
    $launched = Start-Process -FilePath $appPath -ArgumentList ('--data-dir "' + $dataPath + '" --port 18766 --debug-port 9223 --no-demo') -PassThru
    for ($i=0; $i -lt 80; $i++) {
        try {
            $pages=Invoke-RestMethod 'http://127.0.0.1:9223/json/list' -TimeoutSec 1
            if ($pages.Count -gt 0) {
                $readySession = Read-Session
                if ($readySession.appPid -ne $launched.Id -and -not $launched.WaitForExit(5000)) { throw 'Duplicate launcher failed to exit.' }
                return
            }
        } catch { }
        Start-Sleep -Milliseconds 100
    }
    throw 'Desktop readiness timed out.'
}
function Close-TestApp {
    $session = Read-Session
    $ownedApp = Get-Process -Id $session.appPid
    if ($ownedApp.Path -ne $appPath) { throw 'Refusing to close an unrelated process.' }
    if (-not $ownedApp.CloseMainWindow()) { throw 'Native window could not be closed.' }
    if (-not $ownedApp.WaitForExit(10000)) { throw 'Native window exit timed out.' }
    if (Test-Port 18766) { throw 'Owned listener leaked after closing.' }
    if (Get-Process -Id $session.backendPid -ErrorAction SilentlyContinue) { throw 'Backend process leaked.' }
}
Start-TestApp
Close-TestApp
$checks.Add('normal native window close releases backend PID and loopback port')
Start-TestApp
& node tests/browser/desktop_persistence.cjs
if ($LASTEXITCODE -ne 0) { throw 'Desktop scene persistence failed.' }
$checks.Add('named desktop scenes survive process restart')
$before = Read-Session
Start-TestApp
$after = Read-Session
if ($before.appPid -ne $after.appPid -or $before.backendPid -ne $after.backendPid) { throw 'Duplicate launch replaced the owned session.' }
$checks.Add('repeat launch focuses existing instance and preserves its backend')
$ownedApp = Get-Process -Id $after.appPid
if ($ownedApp.Path -ne $appPath) { throw 'Refusing to stop an unrelated process.' }
Stop-Process -Id $ownedApp.Id -Force
for ($i=0; $i -lt 40 -and (Test-Port 18766); $i++) { Start-Sleep -Milliseconds 100 }
if (Test-Port 18766) { throw 'Job object failed to release port after parent termination.' }
if (Get-Process -Id $after.backendPid -ErrorAction SilentlyContinue) { throw 'Job object leaked backend after parent termination.' }
$checks.Add('forced parent termination also releases the owned backend through Windows job ownership')
@{ok=$true;checks=$checks} | ConvertTo-Json -Depth 3 | Set-Content -Encoding UTF8 artifacts/desktop-v080/native-lifecycle.json
$checks
