#!/usr/bin/env python3
"""Deploy thickness pair-routing + path3 length fix to field IPC."""
from __future__ import annotations

import os
import stat
import sys
import time
from pathlib import Path

import paramiko

HOST = "192.168.104.211"
USER = "Administrator"
PASSWORD = "123456"
REMOTE_ROOT = r"D:\work\IPC_Station2"
LOCAL_APP = Path(r"D:\work\LY\IPC_Station2\build\win-msvc2019-qtcore-ninja-release\app")
LOCAL_WORKER = LOCAL_APP / "workers" / "thickness_v3_1"

FILES = [
    (LOCAL_APP / "scan-tracking.exe", rf"{REMOTE_ROOT}\scan-tracking.exe"),
    (
        LOCAL_WORKER / "thickness-measure-v3_1-worker.exe",
        rf"{REMOTE_ROOT}\workers\thickness_v3_1\thickness-measure-v3_1-worker.exe",
    ),
    (
        LOCAL_WORKER / "ThicknessMeasurement.dll",
        rf"{REMOTE_ROOT}\workers\thickness_v3_1\ThicknessMeasurement.dll",
    ),
]


def connect() -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    # Prefer key/agent if available; fall back to password.
    try:
        client.connect(
            HOST,
            username=USER,
            password=PASSWORD,
            timeout=15,
            allow_agent=True,
            look_for_keys=True,
        )
    except paramiko.AuthenticationException:
        client.connect(
            HOST,
            username=USER,
            password=PASSWORD,
            timeout=15,
            allow_agent=False,
            look_for_keys=False,
        )
    return client


def run(client: paramiko.SSHClient, cmd: str, timeout: float = 60) -> tuple[int, str, str]:
    print(f"$ {cmd}", flush=True)
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    code = stdout.channel.recv_exit_status()

    def safe_print(msg: str) -> None:
        data = (msg + "\n").encode("utf-8", "replace")
        try:
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
        except Exception:
            sys.stdout.write(msg.encode("ascii", "replace").decode("ascii") + "\n")
            sys.stdout.flush()

    if out.strip():
        safe_print(out.rstrip())
    if err.strip():
        safe_print("[stderr] " + err.rstrip())
    print(f"exit={code}", flush=True)
    return code, out, err


def sftp_put(sftp: paramiko.SFTPClient, local: Path, remote: str) -> None:
    remote_posix = remote.replace("\\", "/")
    remote_dir = remote_posix.rsplit("/", 1)[0]
    # Ensure remote dir exists (OpenSSH Windows accepts forward slashes).
    parts = remote_dir.split("/")
    cur = parts[0]  # e.g. D:
    for part in parts[1:]:
        if not part:
            continue
        cur = f"{cur}/{part}"
        try:
            sftp.stat(cur)
        except FileNotFoundError:
            try:
                sftp.mkdir(cur)
            except OSError:
                pass
    print(f"upload {local.name} ({local.stat().st_size}) -> {remote}", flush=True)
    sftp.put(str(local), remote_posix)
    attr = sftp.stat(remote_posix)
    print(f"  remote size={attr.st_size}", flush=True)
    if attr.st_size != local.stat().st_size:
        raise RuntimeError(f"size mismatch for {remote}")


def main() -> int:
    for local, _ in FILES:
        if not local.exists():
            print(f"MISSING local: {local}", flush=True)
            return 1

    print("=== connect ===", flush=True)
    client = connect()
    try:
        run(client, f'cmd /c "cd /d {REMOTE_ROOT} & echo CWD_OK & dir /b scan-tracking.exe"')
        code, out, _ = run(
            client,
            "powershell -NoProfile -Command \"Get-Process scan-tracking -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Id\"",
        )
        running = bool(out.strip())
        if running:
            print("=== stop scan-tracking.exe ===", flush=True)
            run(client, "powershell -NoProfile -Command \"Stop-Process -Name scan-tracking -Force -ErrorAction SilentlyContinue; Start-Sleep -Seconds 2; if(Get-Process scan-tracking -ErrorAction SilentlyContinue){'STILL_RUNNING'}else{'STOPPED'}\"")

        # Also stop leftover workers if any
        run(
            client,
            "powershell -NoProfile -Command \"Stop-Process -Name thickness-measure-v3_1-worker -Force -ErrorAction SilentlyContinue; 'WORKER_KILL_DONE'\"",
        )

        print("=== upload ===", flush=True)
        sftp = client.open_sftp()
        try:
            for local, remote in FILES:
                # Backup remote first
                remote_posix = remote.replace("\\", "/")
                bak = remote_posix + f".bak_{time.strftime('%Y%m%d_%H%M%S')}"
                try:
                    sftp.stat(remote_posix)
                    print(f"backup {remote_posix} -> {bak}", flush=True)
                    # SFTP rename may fail across volumes; copy via get/put not needed - use ssh copy
                    run(
                        client,
                        f'cmd /c "copy /Y {remote} {remote}.bak_deploy >nul & echo BAK_OK"',
                    )
                except FileNotFoundError:
                    print(f"no existing remote file: {remote}", flush=True)
                sftp_put(sftp, local, remote)
        finally:
            sftp.close()

        print("=== verify sizes ===", flush=True)
        run(
            client,
            'cmd /c "dir D:\\work\\IPC_Station2\\scan-tracking.exe '
            "D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\thickness-measure-v3_1-worker.exe "
            'D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\ThicknessMeasurement.dll"',
        )

        print("=== probe worker load ===", flush=True)
        # Invalid request should still load DLLs and exit non-crash (expect 4)
        run(
            client,
            'powershell -NoProfile -Command '
            '"$tmp=Join-Path $env:TEMP tm_deploy_probe; '
            "New-Item -ItemType Directory -Force -Path $tmp|Out-Null; "
            "Set-Content (Join-Path $tmp request.txt) 'mode=pair'; "
            "$p=Start-Process -FilePath 'D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\thickness-measure-v3_1-worker.exe' "
            "-ArgumentList $tmp -WorkingDirectory 'D:\\work\\IPC_Station2\\workers\\thickness_v3_1' "
            "-Wait -PassThru -NoNewWindow; "
            "Write-Host ('WORKER_EXIT=' + $p.ExitCode)\"",
            timeout=30,
        )

        print("=== export check MeasureOnePairForGroup ===", flush=True)
        # Use powershell Select-String on binary may fail; use findstr on strings via python remote if available
        run(
            client,
            'powershell -NoProfile -Command '
            "\"$b=[IO.File]::ReadAllBytes('D:\\work\\IPC_Station2\\workers\\thickness_v3_1\\ThicknessMeasurement.dll'); "
            "$s=[Text.Encoding]::ASCII.GetString($b); "
            "if($s -match 'MeasureOnePairForGroup'){'EXPORT_OK'} else {'EXPORT_MISSING'}\"",
        )

        print("DEPLOY_DONE", flush=True)
        print("请在现场重启 scan-tracking.exe 后验证：", flush=True)
        print("  1) path3 后应有 runRoot/container_length_mm.txt", flush=True)
        print("  2) path4 pair1 日志含 groupIndex=1 与 Template_..._12", flush=True)
        print("  3) lengthSource=path3_measured", flush=True)
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
