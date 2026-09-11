#!/usr/bin/env python3
"""Merge Path3 arm stitched PLY clouds into one XYZ binary PLY for DLL demo."""
from __future__ import annotations

import argparse
import os
import struct
import sys


def parse_header(f):
    header_lines = []
    while True:
        line = f.readline()
        if not line:
            raise RuntimeError("unexpected EOF in PLY header")
        header_lines.append(line)
        if line.strip() == b"end_header":
            break
    text = b"".join(header_lines).decode("ascii", errors="replace")
    fmt = None
    n_vertex = 0
    props = []
    for raw in text.splitlines():
        parts = raw.strip().split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element" and parts[1] == "vertex":
            n_vertex = int(parts[2])
        elif parts[0] == "property":
            props.append((parts[1], parts[2]))
    if fmt != "binary_little_endian":
        raise RuntimeError(f"unsupported format: {fmt}")
    size_map = {
        "float": 4,
        "float32": 4,
        "double": 8,
        "float64": 8,
        "uchar": 1,
        "uint8": 1,
        "char": 1,
        "int8": 1,
        "ushort": 2,
        "uint16": 2,
        "short": 2,
        "int16": 2,
        "uint": 4,
        "uint32": 4,
        "int": 4,
        "int32": 4,
    }
    stride = 0
    xyz_offsets = {}
    for typ, name in props:
        if typ not in size_map:
            raise RuntimeError(f"unsupported property type: {typ} {name}")
        if name in ("x", "y", "z"):
            xyz_offsets[name] = stride
        stride += size_map[typ]
    if set(xyz_offsets) != {"x", "y", "z"}:
        raise RuntimeError(f"missing xyz in props: {props}")
    return n_vertex, stride, xyz_offsets


def merge(paths, out_path, stride_points: int = 1):
    counts = []
    for p in paths:
        with open(p, "rb") as f:
            n, stride, _ = parse_header(f)
            counts.append((p, n, stride))
            print(f"  {os.path.basename(p)}: {n} pts, stride={stride}", flush=True)

    total = sum(((n + stride_points - 1) // stride_points) for _, n, _ in counts)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {total}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")

    written = 0
    with open(out_path, "wb") as out:
        out.write(header)
        for path, n, stride in counts:
            with open(path, "rb") as f:
                parse_header(f)
                chunk = 65536
                i = 0
                while i < n:
                    take = min(chunk, n - i)
                    blob = f.read(take * stride)
                    if len(blob) != take * stride:
                        raise RuntimeError(f"short read in {path}")
                    for j in range(take):
                        if ((i + j) % stride_points) != 0:
                            continue
                        off = j * stride
                        out.write(blob[off : off + 12])  # xyz float32 little-endian
                        written += 1
                    i += take
            print(f"  merged {os.path.basename(path)}", flush=True)

    print(f"Wrote {written} points -> {out_path}", flush=True)
    if written != total:
        raise RuntimeError(f"count mismatch written={written} expected={total}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="path_3 directory")
    ap.add_argument("--out", required=True)
    ap.add_argument("--segments", type=int, default=8)
    ap.add_argument("--stride", type=int, default=1, help="keep every Nth point")
    args = ap.parse_args()
    paths = []
    for i in range(1, args.segments + 1):
        p = os.path.join(args.root, "arm", str(i), f"Path3_Arm_cloud_stitched_{i}.ply")
        if not os.path.isfile(p):
            print(f"missing: {p}", file=sys.stderr)
            return 2
        paths.append(p)
    print(f"Merging {len(paths)} clouds, stride={args.stride}", flush=True)
    merge(paths, args.out, stride_points=max(1, args.stride))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
