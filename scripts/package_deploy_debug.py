#!/usr/bin/env python3
"""Package Debug build into D:/work/LY/deploy_debug."""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(r"D:\work\LY\IPC-192.168.110.173_track-main")
BUILD_APP = ROOT / "build" / "win-msvc2019-qtcore-ninja-debug" / "app"
BUILD_SMOKE = ROOT / "build" / "win-msvc2019-qtcore-ninja-debug" / "modules" / "vision"
DEPLOY = Path(r"D:\work\LY\deploy_debug")
DEPLOY_POSIX = "D:/work/LY/deploy_debug"

VS_DEBUG_CRT = Path(
    r"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Redist\MSVC"
    r"\14.29.30133\debug_nonredist\x64\Microsoft.VC142.DebugCRT"
)
UCRT_DEBUG = Path(
    r"C:\Program Files (x86)\Windows Kits\10\Redist\10.0.26100.0\ucrt\DLLs\x64"
)
SYSTEM32 = Path(r"C:\Windows\System32")

DEBUG_CRT_NAMES = [
    "vcruntime140d.dll",
    "vcruntime140_1d.dll",
    "msvcp140d.dll",
    "ucrtbased.dll",
    "CONCRT140D.dll",
    "MSVCP140_1D.dll",
    "MSVCP140D_ATOMIC_WAIT.dll",
    "MSVCP140_2D.dll",
    "VCOMP140D.dll",
]


def robocopy(src: Path, dst: Path) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    cmd = ["robocopy", str(src), str(dst), "/E", "/NFL", "/NDL", "/NJH", "/NJS", "/nc", "/ns", "/np"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode >= 8:
        raise RuntimeError(f"robocopy failed ({result.returncode}): {result.stderr}")


def find_debug_crt(name: str) -> Path | None:
    for folder in (VS_DEBUG_CRT, UCRT_DEBUG, SYSTEM32):
        candidate = folder / name
        if candidate.is_file():
            return candidate
    return None


def write_config_ini() -> None:
    src = ROOT / "config.ini"
    text = src.read_text(encoding="utf-8")
    replacements = {
        "D:/work/LY/IPC-192.168.110.173_track-main": DEPLOY_POSIX,
        "D:/work/LY/deploy": DEPLOY_POSIX,
        "D:/CxpSmokeTest": f"{DEPLOY_POSIX}/CxpSmokeTest",
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    if "hikSdkRoot=" in text and DEPLOY_POSIX not in text.split("hikSdkRoot=", 1)[1].splitlines()[0]:
        pass
    (DEPLOY / "config.ini").write_text(text, encoding="utf-8", newline="\n")


def main() -> int:
    if not BUILD_APP.joinpath("scan-tracking.exe").is_file():
        print("ERROR: Debug scan-tracking.exe not found. Build first.", file=sys.stderr)
        return 1

    if DEPLOY.exists():
        shutil.rmtree(DEPLOY)
    DEPLOY.mkdir(parents=True)

    print("Copying runtime from", BUILD_APP)
    robocopy(BUILD_APP, DEPLOY)

    smoke = BUILD_SMOKE / "scan_tracking_cxp_smoke.exe"
    if smoke.is_file():
        shutil.copy2(smoke, DEPLOY / smoke.name)
        print("Copied", smoke.name)

    shutil.copy2(ROOT / "scan_paths_config.json", DEPLOY / "scan_paths_config.json")

    for rel in (
        "third_party/LBN/data/template-3D-ALL-Shift-Cut-Cut.txt",
        "third_party/LB/Data/template-3D-ALL-Shift-Cut-Cut.txt",
    ):
        src = ROOT / rel
        dst = DEPLOY / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        if src.is_file():
            shutil.copy2(src, dst)

    mvs_common = ROOT / "third_party" / "MVS" / "CommonParameters.ini"
    if mvs_common.is_file():
        shutil.copy2(mvs_common, DEPLOY / "CommonParameters.ini")

    write_config_ini()

    copied_crt = 0
    for name in DEBUG_CRT_NAMES:
        src = find_debug_crt(name)
        if src is None:
            print("WARN: missing debug CRT", name)
            continue
        shutil.copy2(src, DEPLOY / name)
        copied_crt += 1
    print(f"Copied {copied_crt} MSVC debug CRT DLLs")

    bat = DEPLOY / "start_scan_tracking.bat"
    bat.write_text(
        "@echo off\r\n"
        "chcp 65001 >nul\r\n"
        'cd /d "%~dp0"\r\n'
        "echo Debug build - scan-tracking.exe\r\n"
        "scan-tracking.exe\r\n"
        "pause\r\n",
        encoding="ascii",
    )

    note = DEPLOY / "现场测试说明.txt"
    release_note = ROOT.parent / "deploy" / "现场测试说明.txt"
    if release_note.is_file():
        base = release_note.read_text(encoding="utf-8")
        base = base.replace("deploy_scan-tracking-cxp-release.zip", "deploy_debug（本目录）")
        base = base.replace("D:\\work\\LY\\deploy", r"D:\work\LY\deploy_debug")
        base = base.replace("D:/work/LY/deploy", DEPLOY_POSIX)
        base = "【Debug 构建】\n" + base
        note.write_text(base, encoding="utf-8", newline="\n")

    exe_count = len(list(DEPLOY.glob("*.exe")))
    dll_count = len(list(DEPLOY.glob("*.dll")))
    print(f"Done: {DEPLOY}")
    print(f"  exe={exe_count} dll={dll_count}")
    print(f"  scan-tracking.exe size={DEPLOY.joinpath('scan-tracking.exe').stat().st_size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
