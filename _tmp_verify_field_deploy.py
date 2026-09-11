#!/usr/bin/env python3
import sys
import paramiko

HOST = "192.168.104.211"
USER = "Administrator"
PASSWORD = "123456"


def main() -> int:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=PASSWORD, timeout=15, allow_agent=True, look_for_keys=True)

    def run(cmd: str, timeout: float = 60) -> int:
        print("CMD:", cmd, flush=True)
        _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
        out = stdout.read()
        err = stderr.read()
        code = stdout.channel.recv_exit_status()
        sys.stdout.buffer.write(out)
        sys.stdout.buffer.write(err)
        sys.stdout.buffer.flush()
        print(f"exit={code}", flush=True)
        return code

    run(
        "powershell -NoProfile -Command "
        "\"Get-Item "
        "'D:\\work\\IPC_Station2\\scan-tracking.exe',"
        "'D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\thickness-measure-v3_1-worker.exe',"
        "'D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\ThicknessMeasurement.dll' "
        "| ForEach-Object { $_.Name + '=' + $_.Length }\""
    )

    run(
        "powershell -NoProfile -Command "
        "\"$b=[IO.File]::ReadAllBytes('D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\ThicknessMeasurement.dll'); "
        "$s=[Text.Encoding]::ASCII.GetString($b); "
        "if($s -match 'MeasureOnePairForGroup'){'EXPORT_OK'} else {'EXPORT_MISSING'}\""
    )

    run(
        "powershell -NoProfile -Command "
        "\"$tmp=Join-Path $env:TEMP tm_deploy_probe; "
        "New-Item -ItemType Directory -Force -Path $tmp | Out-Null; "
        "Set-Content (Join-Path $tmp request.txt) 'mode=pair'; "
        "$p=Start-Process -FilePath 'D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\thickness-measure-v3_1-worker.exe' "
        "-ArgumentList $tmp -WorkingDirectory 'D:\\work\\IPC_Station2\\workers\\thickness_v3_1' "
        "-Wait -PassThru -NoNewWindow; "
        "Write-Host ('WORKER_EXIT=' + $p.ExitCode)\"",
        timeout=30,
    )

    # Compare with local expected sizes
    print("EXPECTED scan-tracking.exe=1959936 worker=50688 dll=678912", flush=True)
    c.close()
    print("VERIFY_DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
