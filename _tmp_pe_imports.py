#!/usr/bin/env python3
"""List DLL imports from a PE file (no external deps)."""
from __future__ import annotations

import argparse
import struct
import sys


def read_imports(path: str):
    with open(path, "rb") as f:
        data = f.read()
    if data[:2] != b"MZ":
        raise RuntimeError("not PE")
    e_lfanew = struct.unpack_from("<I", data, 0x3C)[0]
    if data[e_lfanew : e_lfanew + 4] != b"PE\0\0":
        raise RuntimeError("bad PE")
    coff = e_lfanew + 4
    num_sections = struct.unpack_from("<H", data, coff + 2)[0]
    opt_size = struct.unpack_from("<H", data, coff + 16)[0]
    opt = coff + 20
    magic = struct.unpack_from("<H", data, opt)[0]
    if magic == 0x20B:  # PE32+
        dd_off = opt + 112
    elif magic == 0x10B:  # PE32
        dd_off = opt + 96
    else:
        raise RuntimeError(f"unknown optional magic {magic:#x}")
    import_rva, import_size = struct.unpack_from("<II", data, dd_off + 8)
    sections = []
    sec_off = opt + opt_size
    for i in range(num_sections):
        off = sec_off + i * 40
        name = data[off : off + 8].split(b"\0", 1)[0].decode("ascii", "ignore")
        vsize, va, rawsize, rawptr = struct.unpack_from("<IIII", data, off + 8)
        sections.append((name, va, vsize, rawptr, rawsize))

    def rva_to_off(rva: int) -> int:
        for _, va, vsize, rawptr, rawsize in sections:
            if va <= rva < va + max(vsize, rawsize):
                return rawptr + (rva - va)
        raise RuntimeError(f"RVA not found: {rva:#x}")

    names = []
    desc = rva_to_off(import_rva)
    while True:
        (
            oft,
            ts,
            fwd,
            name_rva,
            ft,
        ) = struct.unpack_from("<IIIII", data, desc)
        if oft == 0 and name_rva == 0 and ft == 0:
            break
        name_off = rva_to_off(name_rva)
        name = data[name_off : data.find(b"\0", name_off)].decode("ascii", "ignore")
        names.append(name)
        desc += 20
    return names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pe")
    args = ap.parse_args()
    for n in read_imports(args.pe):
        print(n)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("ERROR:", e, file=sys.stderr)
        sys.exit(1)
