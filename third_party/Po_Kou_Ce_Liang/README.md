# Nano Ultra Bevel Measurement

This VS2022/C++ project implements a configurable bevel measurement pipeline:

1. Uniform down sampling.
2. Pose correction.
3. Crop-box filtering.
4. Statistical isolated-point filtering.
5. Local tangent-section projection to a 512 x 512 grayscale image.
6. OpenCV 3.4.3 SVM bevel-type prediction.
7. Scan-to-template point-to-point ICP.
8. Actual bevel angle and length solving from template feature points mapped back to nearest scan points.

## Core Interface

Use this function from `include/BevelMeasurement.h`:

```cpp
bevel::BevelMeasurementResult solveBevelFromRawCloud(
    const bevel::CloudT::ConstPtr& rawCloud,
    const std::string& configPath = "config.txt");
```

The input is the raw PCL point cloud. The output contains:

- `ok`
- `bevelType`
- `angleDeg`
- `length`
- `icpFitness`
- `scanToTemplate`
- `message`

## Config

All runtime parameters are in `config.txt`, including:

- down-sampling radius
- pose translation and rotation
- crop-box bounds
- outlier-filter parameters
- tangent-section center, normal, thickness, scale, and image size
- SVM model path
- template PCD path pattern
- template feature path pattern
- ICP thresholds

The template path uses the predicted SVM label:

```txt
template.path_pattern = data/templates/type_{type}.pcd
template.plane_fit_feature_path_pattern = data/templates/type_{type}_features_plane_fit.txt
template.direct_feature_path_pattern = data/templates/type_{type}_features_direct.txt
```

For example, if SVM returns `0`, the project loads `data/templates/type_0.pcd` and then chooses `type_0_features_plane_fit.txt` or `type_0_features_direct.txt` according to `measurement.method`.

## Template Feature File

Each template feature file must contain these five named points:

```txt
angle_a x y z
angle_b x y z
angle_c x y z
length_a x y z
length_b x y z
```

`angle_a`, `angle_b`, and `angle_c` define the measured bevel angle at `angle_b`.
`length_a` and `length_b` define the measured bevel length.

These points must be expressed in the same coordinate system as the corresponding template PCD.

## VS2022 Setup

Open:

```txt
BevelMeasurementDemo.sln
```

This project uses the `v120` toolset because the local PCL 1.8.0 and OpenCV 3.4.3 libraries are `vc120/vc12` prebuilt binaries.

The OpenCV property sheet points to:

```txt
F:\deep_learning\opencv\opencv-3.4.3\opencv-3.4.3\build\install
```

The PCL property sheet is:

```txt
props\PCL180_x64.props
```

`PCL_ROOT` points to `F:\software install\PCL 1.8.0`.

## Demo Usage

After building:

```bat
BevelMeasurementDemo.exe raw_cloud.pcd config.txt
BevelMeasurementDemo.exe raw_cloud.txt config.txt
```

TXT point clouds are expected as whitespace-separated `x y z` rows.

## Notes

The ICP direction is scan-to-template:

```cpp
icp.setInputSource(scan);
icp.setInputTarget(templateCloud);
```

After ICP, template feature points are transformed back into scan coordinates, and nearest scan points are used to solve the actual angle and length.
