#!/usr/bin/env python3
"""Run field911 Pair2 (11/12) via V3.1 demo group2 (point-to-plane)."""
from __future__ import annotations

import os
import re
import subprocess
import time
from pathlib import Path

STAGE = Path(r"D:\work\LY\IPC_Station2\_tmp_path4_pair1_run")
EXE = STAGE / "bin" / "ThicknessMeasurementDemo.exe"
INI = STAGE / "config" / "thickness_measurement_field911_pair2.ini"
LOG = STAGE / "demo_field911_pair2.log"

INI_TEXT = r"""# UTF-8 field911: group1=pair1, group2=pair2(11/12 point-to-plane)

[Input]
outer_template_cloud_path=../Data/Template_Path4_Arm_cloud_stitched_1_sample.pcd
inner_scan_input_path=../Data/field_911/1_Path4_Arm_cloud_stitched_inner.pcd
outer_scan_input_path=../Data/field_911/2_Path4_Arm_cloud_stitched_outter.pcd

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

[Input2]
outer_template_cloud_path=../Data/Template_Path4_Arm_cloud_stitched_12_sample.pcd
inner_scan_input_path=../Data/field_911/11_Path4_Arm_cloud_stitched_inner.pcd
outer_scan_input_path=../Data/field_911/12_Path4_Arm_cloud_stitched_outter.pcd

[DynamicSections2]
enabled=true
axis_point=(-125.3787, -283.2391, 3342.9138)
axis_direction=(0.0326, 0.6934, -0.7198)
reference_point=(-683.993103, 69.643288, 2955.781250)
section_offset_1_mm=30.0
section_offset_2_mm=60.0
section_half_width_mm=60.0
section_thickness_mm=1.5
center_point_1=(-726.182373,-155.593552,3198.762695)
center_point_2=(-729.718262,-224.059143,3266.466553)

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
model_path=../Data/pointnet_weld_seam_V7.3_good.onnx
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

[InnerOuterICP2]
max_iterations=15
max_correspondence_distance_mm=1.5
transformation_epsilon=0.000001
euclidean_fitness_epsilon=0.000001
normal_k=20
max_curvature=0.03
min_planarity=0.55
normal_smoothing_k=12
normal_smoothing_angle_deg=25.0
min_reliable_point_count=100

[FallbackFeatures2]
feature_count=2
point1=(-722.491943,-202.410751,3159.555420)
point2=(-726.359192,-158.731384,3202.736328)
"""


def main() -> int:
    INI.write_text(INI_TEXT, encoding="utf-8")
    env = os.environ.copy()
    env["PATH"] = str(STAGE / "bin") + os.pathsep + env.get("PATH", "")
    print("running group1(pair1)+group2(pair2)...", flush=True)
    t0 = time.time()
    with open(LOG, "w", encoding="utf-8", errors="replace") as f:
        proc = subprocess.run(
            [str(EXE), str(INI)],
            cwd=str(STAGE),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=f,
            stderr=subprocess.STDOUT,
        )
    dt = time.time() - t0
    print(f"EXIT_CODE={proc.returncode} elapsed_sec={dt:.2f}", flush=True)

    raw = LOG.read_bytes()
    text = None
    for enc in ("utf-8", "gbk", "utf-8-sig"):
        try:
            text = raw.decode(enc)
            print(f"decoded_as={enc}", flush=True)
            break
        except UnicodeDecodeError:
            pass
    if text is None:
        text = raw.decode("utf-8", "replace")

    fitness = re.findall(r"fitness=([0-9.]+)", text)
    ab = re.findall(r"A=([0-9.]+) mm.*?B=([0-9.]+) mm", text, flags=re.S)
    # Summary lines like: 第N组厚度: xx mm  (may be mojibake) — capture "mm, method"
    group_thickness = re.findall(
        r"([0-9]+\.[0-9]+) mm, (?:onnx_toe_sections|method|[^\n]{0,40})",
        text,
    )
    # More reliable: lines containing 'onnx_toe_sections'
    onnx_lines = [ln for ln in text.splitlines() if "onnx_toe_sections" in ln or "fitness=" in ln]
    print("fitness_values=", fitness, flush=True)
    print("A_B_pairs=", ab, flush=True)
    print("onnx_or_fitness_lines:", flush=True)
    for ln in onnx_lines:
        print(" ", ln, flush=True)

    # Split cycles on [1/5
    cycles = [p for p in re.split(r"(?=\[1/5)", text) if p.strip() and "[1/5" in p]
    print(f"measure_cycles={len(cycles)}", flush=True)
    for i, cyc in enumerate(cycles, 1):
        label = "Pair1(group1)" if i == 1 else "Pair2(group2)" if i == 2 else f"cycle{i}"
        print(f"\n==== {label} ====", flush=True)
        fits = re.findall(r"fitness=([0-9.]+)", cyc)
        abs_ = re.findall(r"A=([0-9.]+) mm.*?B=([0-9.]+) mm", cyc, flags=re.S)
        pts = re.findall(r"点数=([0-9]+)", cyc)
        if not pts:
            pts = re.findall(r"=([0-9]{6,})", cyc)  # fallback
        print(f"  fitness={fits}", flush=True)
        print(f"  section A/B={abs_}", flush=True)
        # print key ascii-ish lines
        for ln in cyc.splitlines():
            if any(
                k in ln
                for k in (
                    "fitness=",
                    "A=",
                    "onnx",
                    "[1/5",
                    "[2/5",
                    "[3/5",
                    "[4/5",
                    "[5/5",
                    "ICP",
                    "11_",
                    "12_",
                    "1_",
                    "2_",
                    "Template",
                )
            ):
                print(f"  {ln}", flush=True)

    print("\n---- FULL LOG ----", flush=True)
    print(text, flush=True)
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())
