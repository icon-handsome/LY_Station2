# ============================================
# IPC Station2 现场排查和部署脚本
# ============================================

Write-Host "=== 1. 检查源码修改时间 ===" -ForegroundColor Green
$sourceFile = "D:\work\IPC_Station2\modules\hmi_server\src\hmi_tcp_server.cpp"
if (Test-Path $sourceFile) {
    $sourceTime = (Get-Item $sourceFile).LastWriteTime
    Write-Host "源码最后修改时间: $sourceTime" -ForegroundColor Yellow
} else {
    Write-Host "❌ 源码文件不存在！" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=== 2. 检查新增日志代码是否存在 ===" -ForegroundColor Green
$logLineExists = Select-String -Path $sourceFile -Pattern "接收 cmd.set_head_type msgId=" -Quiet
if ($logLineExists) {
    Write-Host "✅ 源码中包含新增的日志代码" -ForegroundColor Green
    # 显示具体行
    Select-String -Path $sourceFile -Pattern "接收 cmd.set_head_type msgId=" | Select-Object LineNumber, Line
} else {
    Write-Host "❌ 源码中没有找到新增的日志代码！" -ForegroundColor Red
}

Write-Host ""
Write-Host "=== 3. 查找编译产物 ===" -ForegroundColor Green
$buildDir = "D:\work\IPC_Station2\build"
$exeFile = Get-ChildItem -Path $buildDir -Recurse -Filter "scan_tracking.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
if ($exeFile) {
    Write-Host "找到可执行文件: $($exeFile.FullName)" -ForegroundColor Yellow
    Write-Host "编译时间: $($exeFile.LastWriteTime)" -ForegroundColor Yellow
    
    # 检查编译时间是否晚于源码修改时间
    if ($exeFile.LastWriteTime -gt $sourceTime) {
        Write-Host "✅ 编译产物时间晚于源码修改时间" -ForegroundColor Green
    } else {
        Write-Host "❌ 编译产物时间早于源码修改时间，需要重新编译！" -ForegroundColor Red
    }
} else {
    Write-Host "❌ 未找到可执行文件！" -ForegroundColor Red
}

Write-Host ""
Write-Host "=== 4. 检查正在运行的进程 ===" -ForegroundColor Green
$process = Get-Process -Name "scan_tracking" -ErrorAction SilentlyContinue
if ($process) {
    Write-Host "找到运行中的进程:" -ForegroundColor Yellow
    Write-Host "  PID: $($process.Id)" -ForegroundColor Yellow
    Write-Host "  启动时间: $($process.StartTime)" -ForegroundColor Yellow
    Write-Host "  路径: $($process.Path)" -ForegroundColor Yellow
    
    # 检查进程启动时间
    if ($exeFile -and $process.StartTime -lt $exeFile.LastWriteTime) {
        Write-Host "❌ 进程启动时间早于编译时间，需要重启进程！" -ForegroundColor Red
    }
} else {
    Write-Host "⚠ 没有找到运行中的 scan_tracking 进程" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "=== 5. 检查日志文件 ===" -ForegroundColor Green
$logFile = "D:\work\IPC_Station2\build\scan_tracking.log"
if (Test-Path $logFile) {
    Write-Host "日志文件: $logFile" -ForegroundColor Yellow
    Write-Host "最后修改: $((Get-Item $logFile).LastWriteTime)" -ForegroundColor Yellow
    
    # 检查最近的HMI连接日志
    Write-Host ""
    Write-Host "最近的HMI相关日志（最后20行）:" -ForegroundColor Cyan
    Get-Content $logFile -Tail 20 | Select-String -Pattern "hmi|HMI|cmd.set_head_type"
} else {
    Write-Host "⚠ 日志文件不存在: $logFile" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "=== 诊断总结 ===" -ForegroundColor Green
Write-Host "1. 如果源码有新日志但编译产物时间较旧 → 需要重新编译"
Write-Host "2. 如果编译产物是新的但进程启动时间较旧 → 需要重启进程"
Write-Host "3. 如果以上都正常但日志中仍没有新日志 → 可能日志级别被过滤"
Write-Host ""
Write-Host "按任意键继续..." -ForegroundColor Yellow
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
