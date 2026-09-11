#!/usr/bin/env python3
"""Smoke: worker group_index=1 on field911 pair2 clouds."""
from __future__ import annotations

import os
import shutil
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np

STAGE = Path(r"D:\work\LY\IPC_Station2\_tmp_path4_pair1_run")
RT = Path(
    r"D:\work\LY\IPC_Station2\build\win-msvc2019-qtcore-ninja-release\app\workers\thickness_v3_1"
)
NEW_EXE = Path(
    r"D:\work\LY\IPC_Station2\build\win-msvc2019-qtcore-ninja-release\modules\thickness_measure_v2\thickness-measure-v3_1-worker.exe"
)
NEW_DLL = Path(
    r"D:\work\LY\IPC_Station2\third_party\thickness_measure_v3_1\bin\Release\ThicknessMeasurement.dll"
)
INI = STAGE / "config" / "thickness_measurement_field911_pair2.ini"
INNER_PCD = STAGE / "Data" / "field_911" / "11_Path4_Arm_cloud_stitched_inner.pcd"
OUTER_PCD = STAGE / "Data" / "field_911" / "12_Path4_Arm_cloud_stitched_outter.pcd"


def load_pcd_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    text = raw.split(b"DATA binary")[0].decode("ascii", "ignore")
    fields, sizes, counts, n = [], [], [], 0
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "FIELDS":
            fields = p[1:]
        elif p[0] == "SIZE":
            sizes = list(map(int, p[1:]))
        elif p[0] == "COUNT":
            counts = list(map(int, p[1:]))
        elif p[0] == "POINTS":
            n = int(p[1])
    if not counts:
        counts = [1] * len(fields)
    stride = sum(s * c for s, c in zip(sizes, counts))
    body = raw[raw.find(b"DATA binary") + len(b"DATA binary") :].lstrip(b"\r\n")
    offs, off = [], 0
    for s, c in zip(sizes, counts):
        offs.append(off)
        off += s * c
    xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
    xyz = np.empty((n, 3), np.float32)
    for i in range(n):
        base = i * stride
        xyz[i, 0] = struct.unpack_from("<f", body, base + offs[xi])[0]
        xyz[i, 1] = struct.unpack_from("<f", body, base + offs[yi])[0]
        xyz[i, 2] = struct.unpack_from("<f", body, base + offs[zi])[0]
    return xyz


def main() -> int:
    RT.mkdir(parents=True, exist_ok=True)
    shutil.copy2(NEW_DLL, RT / "ThicknessMeasurement.dll")
    shutil.copy2(NEW_EXE, RT / "thickness-measure-v3_1-worker.exe")
    print("staged worker+dll into", RT, flush=True)

    inner = load_pcd_xyz(INNER_PCD)
    outer = load_pcd_xyz(OUTER_PCD)
    print(f"pts inner={inner.shape[0]} outer={outer.shape[0]}", flush=True)

    work = Path(tempfile.mkdtemp(prefix="tmv31_pair2_"))
    print("workdir", work, flush=True)
    inner.astype("<f4").tofile(work / "pair_0_inner.bin")
    outer.astype("<f4").tofile(work / "pair_0_outer.bin")
    (work / "request.txt").write_text(
        "\n".join(
            [
                "mode=pair",
                f"config={INI}",
                "pair_count=1",
                "group_index=1",
                "pair_index=1",
                "inner_0=pair_0_inner.bin",
                f"inner_0_count={inner.shape[0]}",
                "outer_0=pair_0_outer.bin",
                f"outer_0_count={outer.shape[0]}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    env = os.environ.copy()
    env["PATH"] = str(RT) + os.pathsep + env.get("PATH", "")
    t0 = time.time()
    proc = subprocess.run(
        [str(RT / "thickness-measure-v3_1-worker.exe"), str(work)],
        cwd=str(RT),
        env=env,
        capture_output=True,
    )
    dt = time.time() - t0
    print(f"EXIT={proc.returncode} elapsed={dt:.2f}s", flush=True)
    raw = proc.stdout + proc.stderr
    text = None
    for enc in ("utf-8", "gbk"):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            pass
    if text is None:
        text = raw.decode("utf-8", "replace")
    # print key lines
    for ln in text.splitlines():
        if any(
            k in ln
            for k in (
                "group_index",
                "template",
                "fitness",
                "ICP",
                "A=",
                "onnx",
                "[1/5",
                "[3/5",
                "[4/5",
                "未收敛",
                "失败",
            )
        ):
            print(ln, flush=True)
    result = work / "result.txt"
    print("--- result.txt ---", flush=True)
    print(result.read_text(encoding="utf-8", errors="replace") if result.exists() else "MISSING", flush=True)
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
