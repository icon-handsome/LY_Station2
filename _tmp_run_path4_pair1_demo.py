#!/usr/bin/env python3
"""Convert field path4 arm PLY (pair 1=frames 1/2) to PCD, patch ini, run V3.1 demo."""
from __future__ import annotations

import os
import struct
import subprocess
import time
from pathlib import Path

import numpy as np

DEMO_ROOT = Path(r"D:\work\LY\第二工位测量源码\path4\ThicknessMeasurementDll_V3.1\ThicknessMeasurementDll")
ARM_ROOT = Path(r"D:\work\LY\测试数据\911\path_4\arm")
OUT_DIR = DEMO_ROOT / "Data" / "field_911"
RELEASE = DEMO_ROOT / "bin" / "x64" / "Release"
EXE = RELEASE / "ThicknessMeasurementDemo.exe"
TEMPLATE = DEMO_ROOT / "Data" / "Template_Path4_Arm_cloud_stitched_1_sample.pcd"
INI_OUT = DEMO_ROOT / "config" / "thickness_measurement_field911_pair1.ini"
LOG = DEMO_ROOT / "demo_field911_pair1.log"


def load_ply_xyz(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    marker = b"end_header\n"
    idx = raw.find(marker)
    if idx < 0:
        marker = b"end_header\r\n"
        idx = raw.find(marker)
    if idx < 0:
        raise RuntimeError(f"no end_header in {path}")
    header = raw[:idx].decode("ascii", "ignore")
    body = memoryview(raw)[idx + len(marker) :]
    n = None
    props = []
    fmt = None
    for line in header.splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element" and parts[1] == "vertex":
            n = int(parts[2])
        elif parts[0] == "property":
            props.append((parts[1], parts[2]))
    if n is None or fmt is None:
        raise RuntimeError(f"bad ply header: {path}")
    if fmt != "binary_little_endian":
        raise RuntimeError(f"unsupported ply format {fmt}: {path}")

    # Expect float x y z then uchar rgb (common Mech-Eye export).
    type_map = {
        "float": ("f", 4),
        "float32": ("f", 4),
        "double": ("d", 8),
        "uchar": ("B", 1),
        "uint8": ("B", 1),
        "char": ("b", 1),
        "ushort": ("H", 2),
        "short": ("h", 2),
        "int": ("i", 4),
        "uint": ("I", 4),
    }
    struct_fmt = "<"
    stride = 0
    x_off = y_off = z_off = None
    for ptype, pname in props:
        code, size = type_map[ptype]
        if pname == "x":
            x_off = stride
        elif pname == "y":
            y_off = stride
        elif pname == "z":
            z_off = stride
        struct_fmt += code
        stride += size
    if None in (x_off, y_off, z_off):
        raise RuntimeError(f"xyz missing in {path}")

    # Vectorized unpack via numpy frombuffer for xyz+rgb case (15 bytes).
    if stride == 15 and [p[0] for p in props[:3]] == ["float", "float", "float"]:
        dt = np.dtype(
            [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "u1", 3)]
        )
        arr = np.frombuffer(body, dtype=dt, count=n)
        xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float32, copy=False)
    else:
        xyz = np.empty((n, 3), dtype=np.float32)
        for i in range(n):
            base = i * stride
            xyz[i, 0] = struct.unpack_from("<f", body, base + x_off)[0]
            xyz[i, 1] = struct.unpack_from("<f", body, base + y_off)[0]
            xyz[i, 2] = struct.unpack_from("<f", body, base + z_off)[0]

    mask = np.isfinite(xyz).all(axis=1)
    # also drop exact zeros
    mask &= ~((np.abs(xyz) < 1e-6).all(axis=1))
    out = xyz[mask]
    print(f"  {path.name}: total={n} finite={out.shape[0]}", flush=True)
    return out


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
    body = raw[raw.find(b"DATA binary") + len(b"DATA binary") :]
    body = body.lstrip(b"\r\n")
    offs, off = [], 0
    for s, c in zip(sizes, counts):
        offs.append(off)
        off += s * c
    xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
    xyz = np.empty((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * stride
        xyz[i, 0] = struct.unpack_from("<f", body, base + offs[xi])[0]
        xyz[i, 1] = struct.unpack_from("<f", body, base + offs[yi])[0]
        xyz[i, 2] = struct.unpack_from("<f", body, base + offs[zi])[0]
    mask = np.isfinite(xyz).all(axis=1)
    mask &= ~((np.abs(xyz) < 1e-6).all(axis=1))
    return xyz[mask]


def write_pcd_xyz(path: Path, xyz: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    n = xyz.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(np.ascontiguousarray(xyz, dtype=np.float32).tobytes())
    print(f"  wrote {path} pts={n} size={path.stat().st_size}", flush=True)


def centroid(xyz: np.ndarray) -> np.ndarray:
    return xyz.mean(axis=0)


def write_ini(inner_pcd: Path, outer_pcd: Path) -> None:
    # Relative paths from config/ directory.
    def rel(p: Path) -> str:
        return os.path.relpath(p, INI_OUT.parent).replace("\\", "/")

    text = f"""# UTF-8 field911 Pair1-only offline test (inner=frame1, outer=frame2)

[Input]
outer_template_cloud_path={rel(TEMPLATE)}
inner_scan_input_path={rel(inner_pcd)}
outer_scan_input_path={rel(outer_pcd)}

[Output]
save_data=false

[DynamicSections]
enabled=true
axis_point=(-912.4816,-357.8219,3348.3134)
axis_direction=(0.6960,-0.4671,0.5453)
reference_point=(-936.650024,305.256409,2722.577393)
section_offset_1_mm=30.0
section_offset_2_mm=60.0
section_half_width_mm=60.0
section_thickness_mm=1.5
center_point_1=(-495.215424,-4.684036,3066.045166)
center_point_2=(-309.695404,-120.532257,3215.445068)

[Preprocess]
enable_outlier_removal=false
mean_k=10
stddev_mul_thresh=5.0
enable_voxel_downsample=true
leaf_size_mm=1.5

[InnerOuterICP]
max_iterations=15
max_correspondence_distance_mm=1.5
transformation_epsilon=0.001
euclidean_fitness_epsilon=0.001

[OuterTemplateICP]
max_iterations=100
max_correspondence_distance_mm=100.0
transformation_epsilon=0.001
euclidean_fitness_epsilon=0.001

[ONNX]
enabled=true
model_path={rel(DEMO_ROOT / 'Data' / 'pointnet_weld_seam_V7.3_good.onnx')}
input_raw_name=input_raw
input_smooth_name=input_smooth
output_name=output_segmentation
target_point_count=512
median_filter_window=7
seam_class_id=1
num_classes=3
min_seam_points=3

[FallbackFeatures]
feature_count=2
point1=(-483.128296,31.429688,3142.536377)
point2=(-467.313965,-39.799164,3066.436523)
"""
    INI_OUT.write_text(text, encoding="utf-8")
    print(f"wrote ini {INI_OUT}", flush=True)


def main() -> int:
    print("=== convert PLY -> PCD (pair1: inner=1 outer=2) ===", flush=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    inner = load_ply_xyz(ARM_ROOT / "1" / "Path4_Arm_cloud_stitched_1.ply")
    outer = load_ply_xyz(ARM_ROOT / "2" / "Path4_Arm_cloud_stitched_2.ply")
    tmpl = load_pcd_xyz(TEMPLATE)
    tc = centroid(tmpl)
    oc = centroid(outer)
    delta = tc - oc
    print(
        f"prealign template_centroid={tc} outer_centroid={oc} delta={delta} |delta|={np.linalg.norm(delta):.2f}",
        flush=True,
    )
    # Same translation on inner+outer keeps relative pose for InnerOuterICP.
    inner_a = inner + delta.astype(np.float32)
    outer_a = outer + delta.astype(np.float32)
    print(
        f"after prealign inner_c={centroid(inner_a)} outer_c={centroid(outer_a)}",
        flush=True,
    )

    inner_pcd = OUT_DIR / "1_Path4_Arm_cloud_stitched_inner.pcd"
    outer_pcd = OUT_DIR / "2_Path4_Arm_cloud_stitched_outter.pcd"
    write_pcd_xyz(inner_pcd, inner_a)
    write_pcd_xyz(outer_pcd, outer_a)
    write_ini(inner_pcd, outer_pcd)

    if not EXE.exists():
        print(f"MISSING exe: {EXE}", flush=True)
        return 1

    print("=== run ThicknessMeasurementDemo (Pair1 only) ===", flush=True)
    env = os.environ.copy()
    env["PATH"] = str(RELEASE) + os.pathsep + env.get("PATH", "")
    # cwd = demo root so relative output paths land there; pass absolute ini.
    t0 = time.time()
    with open(LOG, "w", encoding="utf-8", errors="replace") as f:
        proc = subprocess.run(
            [str(EXE), str(INI_OUT)],
            cwd=str(DEMO_ROOT),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=f,
            stderr=subprocess.STDOUT,
        )
    dt = time.time() - t0
    print(f"EXIT_CODE={proc.returncode} elapsed_sec={dt:.2f}", flush=True)
    print(f"log: {LOG}", flush=True)
    text = LOG.read_text(encoding="utf-8", errors="replace")
    print(text, flush=True)
    (DEMO_ROOT / "demo_field911_pair1.exit").write_text(
        f"EXIT_CODE={proc.returncode}\nelapsed_sec={dt:.2f}\n", encoding="utf-8"
    )
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
