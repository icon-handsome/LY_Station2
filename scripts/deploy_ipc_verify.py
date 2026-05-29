#!/usr/bin/env python3
"""Deploy D:\\work\\LY\\deploy to IPC and run smoke verification."""
import io
import os
import sys

import paramiko

HOST, USER, PASS = "192.168.110.176", "administrator", "123456"
LOCAL = r"D:\work\LY\deploy"
REMOTE = "D:/work/LY/deploy"
REMOTE_WIN = r"D:\work\LY\deploy"

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")


def mkdir_p(sftp, path: str) -> None:
    parts = path.replace("\\", "/").split("/")
    cur = ""
    for part in parts:
        if not part:
            continue
        if part.endswith(":"):
            cur = part + "/"
            continue
        cur = f"{cur.rstrip('/')}/{part}" if cur else part
        try:
            sftp.stat(cur)
        except OSError:
            try:
                sftp.mkdir(cur)
            except OSError:
                pass


def run(client, cmd: str, timeout: int = 300):
    print(">>>", cmd)
    _, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    code = stdout.channel.recv_exit_status()
    if out.strip():
        print(out.rstrip())
    if err.strip():
        print("STDERR:", err.rstrip())
    print("EXIT:", code)
    return code


def main() -> int:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    print("Connecting to", HOST)
    client.connect(HOST, username=USER, password=PASS, timeout=30, allow_agent=False, look_for_keys=False)
    sftp = client.open_sftp()
    mkdir_p(sftp, REMOTE)

    try:
        items = sftp.listdir(REMOTE)
    except OSError as exc:
        print("listdir failed:", exc)
        items = []

    has_runtime = "MvCameraControl.dll" in items and "Qt5Core.dll" in items
    print("remote has_runtime=", has_runtime, "existing_files=", len(items))

    upload_list: list[tuple[str, str]] = []
    if has_runtime:
        for name in [
            "scan-tracking.exe",
            "scan_tracking_cxp_smoke.exe",
            "config.ini",
            "scan_paths_config.json",
            "现场测试说明.txt",
        ]:
            local_path = os.path.join(LOCAL, name)
            if os.path.isfile(local_path):
                upload_list.append((local_path, f"{REMOTE}/{name}"))
    else:
        print("Full deploy sync...")
        for root, _, files in os.walk(LOCAL):
            rel = os.path.relpath(root, LOCAL).replace("\\", "/")
            remote_dir = REMOTE if rel == "." else f"{REMOTE}/{rel}"
            mkdir_p(sftp, remote_dir)
            for filename in files:
                if filename.endswith(".zip"):
                    continue
                upload_list.append((os.path.join(root, filename), f"{remote_dir}/{filename}"))

    print("Uploading", len(upload_list), "files...")
    for index, (local_path, remote_path) in enumerate(upload_list, 1):
        if index % 50 == 0 or index == len(upload_list):
            print(f"  {index}/{len(upload_list)} {os.path.basename(local_path)}")
        sftp.put(local_path, remote_path)
    sftp.close()

    smoke_ps = (
        f'powershell -NoProfile -Command '
        f'"$d=\'{REMOTE_WIN}\'; '
        f'Remove-Item $d\\CxpSmokeTest\\* -Force -ErrorAction SilentlyContinue; '
        f'$p=Start-Process -FilePath $d\\scan_tracking_cxp_smoke.exe -WorkingDirectory $d -Wait -PassThru '
        f'-RedirectStandardOutput $d\\smoke_out.txt -RedirectStandardError $d\\smoke_err.txt; '
        f'Write-Host SMOKE_EXIT=$($p.ExitCode); '
        f'Get-Content $d\\smoke_out.txt -Encoding UTF8; '
        f'Write-Host ---BMP---; '
        f'Get-ChildItem $d\\CxpSmokeTest -Filter *.bmp | Select-Object Name,Length"'
    )
    smoke_code = run(client, smoke_ps, timeout=180)

    main_ps = (
        f'powershell -NoProfile -Command '
        f'"$d=\'{REMOTE_WIN}\'; '
        f'$p=Start-Process -FilePath $d\\scan-tracking.exe -WorkingDirectory $d -PassThru; '
        f'Start-Sleep 8; '
        f'if (-not $p.HasExited) {{ Stop-Process -Id $p.Id -Force }}; '
        f'Write-Host MAIN_STARTED_OK; '
        f'if (Test-Path $d\\logs) {{ '
        f'Get-ChildItem $d\\logs -Filter *.log | Sort-Object LastWriteTime -Descending | '
        f'Select-Object -First 1 | ForEach-Object {{ Get-Content $_.FullName -Tail 40 -Encoding UTF8 }} }}"'
    )
    run(client, main_ps, timeout=60)

    client.close()
    print("DONE smoke_exit=", smoke_code)
    return smoke_code


if __name__ == "__main__":
    raise SystemExit(main())
