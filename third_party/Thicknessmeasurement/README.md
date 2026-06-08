# 厚度测量算法

这是基于 VS2013、C++、本机 PCL 1.8.0 的厚度测量工程。

## 工程内容

- `ThicknessMeasurement.sln`: VS2013 解决方案。
- `ThicknessMeasurement.vcxproj`: VS2013 C++ 控制台工程。
- `config/thickness_config.json`: 算法配置示例。
- `src/Config.*`: JSON 配置读取。
- `src/ThicknessMeasurement.*`: 点云预处理、内表面 ICP、外表面点云变换、最近点搜索、切面投影和厚度计算。

## 配置说明

`config/thickness_config.json` 中需要设置：

- `point_cloud.template_cloud_path`: 模板点云路径，支持 `.pcd` 和 `.ply`。
- `preprocess`: 孤立点去除和体素降采样参数。
- `icp`: ICP 匹配参数。
- `template_cylinder.axis_point`: 模板点云对应直边柱面轴线上的一点。
- `template_cylinder.axis_direction`: 模板点云对应直边柱面轴线方向。
- `template_feature_points`: 模板点云上绑定的两个特征点坐标。
- `output.result_path`: 输出结果 JSON 文件路径。

第一帧内表面扫描点云和第二帧外表面扫描点云路径不再写入 JSON，默认在 `src/main.cpp` 中设置：

```cpp
const std::string innerScanCloudPath = argc > 2 ? argv[2] : "D:/data/inner_scan.pcd";
const std::string outerScanCloudPath = argc > 3 ? argv[3] : "D:/data/outer_scan.pcd";
```

## 算法流程

1. 读取模板点云、第一帧内表面点云、第二帧外表面点云和配置文件。
2. 对三份点云执行统计孤立点去除和体素降采样。
3. 只使用第一帧内表面点云和模板点云执行 ICP，得到扫描坐标系到模板坐标系的位姿。
4. 将第二帧外表面点云按 ICP 位姿变换到模板坐标系。
5. 使用模板点云绑定的两个特征点，不对特征点应用 ICP 位姿。
6. 在变换后的外表面点云中分别搜索两个模板特征点的最近点。
7. 结合模板柱面轴线，计算过轴线和第一个模板特征点的切面。
8. 将两个最近点投影到切面内，计算投影点距离，作为厚度输出。

## 编译和运行

本机已检测到：

```text
PCL_ROOT=F:\software install\PCL 1.8.0
```

工程默认使用 `$(PCL_ROOT)`。如果在 VS2013 中找不到 PCL，请检查系统环境变量 `PCL_ROOT` 是否存在，或在项目属性中设置 `PclRoot`。

运行方式：

```bat
ThicknessMeasurement.exe config\thickness_config.json
```

也可以在运行时覆盖内外两帧扫描点云路径：

```bat
ThicknessMeasurement.exe config\thickness_config.json D:\data\inner_scan.pcd D:\data\outer_scan.pcd
```

如果不传参数，程序默认读取：

```text
config\thickness_config.json
```

输出为 JSON，包含 ICP 评分、厚度值、模板特征点、扫描最近点和切面投影点。
