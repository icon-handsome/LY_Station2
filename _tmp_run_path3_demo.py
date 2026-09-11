#!/usr/bin/env python3
"""Resolve PE imports, stage DLLs, point config at field cloud, run demo."""
from __future__ import annotations

import os
import shutil
import struct
import subprocess
import sys
import time
from pathlib import Path


def pe_imports(path: Path):
    data = path.read_bytes()
    if data[:2] != b"MZ":
        return []
    e_lfanew = struct.unpack_from("<I", data, 0x3C)[0]
    if data[e_lfanew : e_lfanew + 4] != b"PE\0\0":
        return []
    coff = e_lfanew + 4
    num_sections = struct.unpack_from("<H", data, coff + 2)[0]
    opt_size = struct.unpack_from("<H", data, coff + 16)[0]
    opt = coff + 20
    magic = struct.unpack_from("<H", data, opt)[0]
    dd_off = opt + (112 if magic == 0x20B else 96)
    import_rva = struct.unpack_from("<I", data, dd_off + 8)[0]
    if import_rva == 0:
        return []
    sections = []
    sec_off = opt + opt_size
    for i in range(num_sections):
        off = sec_off + i * 40
        vsize, va, rawsize, rawptr = struct.unpack_from("<IIII", data, off + 8)
        sections.append((va, vsize, rawptr, rawsize))

    def rva_to_off(rva: int) -> int:
        for va, vsize, rawptr, rawsize in sections:
            if va <= rva < va + max(vsize, rawsize):
                return rawptr + (rva - va)
        raise RuntimeError(f"RVA not mapped: {rva:#x} in {path}")

    names = []
    desc = rva_to_off(import_rva)
    while True:
        oft, _ts, _fwd, name_rva, ft = struct.unpack_from("<IIIII", data, desc)
        if oft == 0 and name_rva == 0 and ft == 0:
            break
        name_off = rva_to_off(name_rva)
        end = data.find(b"\0", name_off)
        names.append(data[name_off:end].decode("ascii", "ignore"))
        desc += 20
    return names


SYSTEMISH = {
    "kernel32.dll",
    "user32.dll",
    "gdi32.dll",
    "advapi32.dll",
    "shell32.dll",
    "ole32.dll",
    "oleaut32.dll",
    "ntdll.dll",
    "ws2_32.dll",
    "wsock32.dll",
    "msvcrt.dll",
    "msvcr120.dll",
    "msvcp120.dll",
    "vcruntime140.dll",
    "msvcp140.dll",
    "ucrtbase.dll",
    "combase.dll",
    "rpcrt4.dll",
    "sechost.dll",
    "shlwapi.dll",
    "imm32.dll",
    "version.dll",
    "winmm.dll",
    "setupapi.dll",
    "cfgmgr32.dll",
    "bcrypt.dll",
    "crypt32.dll",
    "iphlpapi.dll",
    "dnsapi.dll",
}


def find_in_search(name: str, search_dirs: list[Path]) -> Path | None:
    lower = name.lower()
    for d in search_dirs:
        for cand in d.glob("*"):
            if cand.is_file() and cand.name.lower() == lower:
                return cand
    return None


def stage_deps(roots: list[Path], dest: Path, seeds: list[Path]):
    dest.mkdir(parents=True, exist_ok=True)
    queue = list(seeds)
    seen = set()
    missing = []
    while queue:
        cur = queue.pop(0)
        key = cur.resolve()
        if key in seen:
            continue
        seen.add(key)
        if cur.parent.resolve() != dest.resolve():
            target = dest / cur.name
            if not target.exists() or target.stat().st_size != cur.stat().st_size:
                shutil.copy2(cur, target)
                print(f"copied {cur.name}", flush=True)
            cur = target
        try:
            imports = pe_imports(cur)
        except Exception as e:
            print(f"warn: imports failed for {cur}: {e}", flush=True)
            continue
        for imp in imports:
            if imp.lower() in SYSTEMISH:
                continue
            if (dest / imp).exists() or (dest / imp.lower()).exists():
                # still recurse into staged copy
                staged = dest / imp if (dest / imp).exists() else dest / imp.lower()
                # Windows FS case-insensitive; find actual
                for p in dest.iterdir():
                    if p.name.lower() == imp.lower():
                        queue.append(p)
                        break
                continue
            found = find_in_search(imp, search_dirs=[dest] + roots)
            if found is None:
                missing.append(imp)
                print(f"MISSING {imp}", flush=True)
            else:
                queue.append(found)
    return sorted(set(missing))


def patch_config(config_path: Path, input_cloud: str):
    bak = config_path.with_suffix(".ini.bak_before_field")
    if not bak.exists():
        shutil.copy2(config_path, bak)
    text = config_path.read_text(encoding="utf-8-sig")
    lines = []
    for line in text.splitlines():
        if line.strip().lower().startswith("inputcloud"):
            lines.append(f"inputCloud = {input_cloud}")
        else:
            lines.append(line)
    config_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    root = Path(r"D:\test\Container_Total_Length_DLL")
    release = root / "x64" / "Release"
    exe = release / "ContainerTotalLengthDemo.exe"
    dll = release / "ContainerTotalLength.dll"
    search = [
        Path(r"D:\work\IPC_Station2"),
        Path(r"D:\work\IPC_Station2\deploy_backup\container_total_length_20260819_0937"),
        Path(r"D:\softwareInstall\Mech-Eye SDK-2.6.0"),
        Path(r"D:\work\Mech-Eye SDK-2.5.4"),
        release,
    ]
    print("=== staging deps ===", flush=True)
    missing = stage_deps(search, release, [exe, dll])
    if missing:
        print("Still missing:", missing, flush=True)

    field_cloud = "./Data/Field_Path3_Arm_All_20260911.ply"
    print("=== patch config ===", flush=True)
    patch_config(root / "config.ini", field_cloud)
    print((root / "config.ini").read_text(encoding="utf-8-sig")[:400], flush=True)

    log = root / "demo_field_run.log"
    print("=== run demo ===", flush=True)
    t0 = time.time()
    env = os.environ.copy()
    env["PATH"] = str(release) + os.pathsep + env.get("PATH", "")
    with open(log, "w", encoding="utf-8", errors="replace") as f:
        proc = subprocess.run(
            [str(exe)],
            cwd=str(root),
            env=env,
            stdout=f,
            stderr=subprocess.STDOUT,
        )
    dt = time.time() - t0
    print(f"EXIT_CODE={proc.returncode} elapsed_sec={dt:.2f}", flush=True)
    print(log.read_text(encoding="utf-8", errors="replace"), flush=True)
    (root / "demo_field_run.exit").write_text(
        f"EXIT_CODE={proc.returncode}\nelapsed_sec={dt:.2f}\n", encoding="utf-8"
    )
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
