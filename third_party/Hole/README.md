# 柱面和开孔测量

这是用于封头内径、内周长、圆度公差、直边斜度、直边高度、开孔距离和开孔接头角度测量的 C++/PCL 工程骨架。当前按 VS2013 + PCL 1.8.0 环境配置，PCL 路径默认为 `F:/software install/PCL 1.8.0`。

## 构建

已提供 VS2013 工程：

```powershell
MSBuild.exe HeadMeasureVS2013.sln /p:Configuration=Release /p:Platform=x64
```

本机已使用 `C:\Program Files (x86)\MSBuild\12.0\Bin\amd64\MSBuild.exe` 编译通过，输出位于：

```text
build_vs2013\Release\head_measure.exe
```

也可使用支持 VS2013 生成器的旧版 CMake：

```powershell
cd "D:\1 自研\15 兰铀算法\测量算法\柱面和开孔测量"
cmake -G "Visual Studio 12 2013 Win64" -S . -B build
cmake --build build --config Release
```

注意：当前机器安装的新版 CMake 不再显示 `Visual Studio 12 2013` 生成器。若命令行生成失败，请使用支持 VS2013 的旧版 CMake，或在 VS2013 中手工创建项目并引用 `PCL_ROOT/include`、`PCL_ROOT/lib`、`PCL_ROOT/3rdParty/Boost` 等路径。

## 运行

```powershell
.\build\Release\head_measure.exe .\config\sample_config.json
```

## 输出

程序会打印所有拟合误差：

- `fit_error name=global_template_icp`
- `fit_error name=top_plane`
- `fit_error name=straight_side_cylinder`
- `fit_error name=axis_slice_x`
- `fit_error name=opening_local_icp`

最终输出 `measure_result`，包含尺寸集合。

## 标准说明

圆度公差参考 GB/T 7235-2004《产品几何量技术规范(GPS) 评定圆度误差的方法 半径变化量测量》。当前默认按最小区域圆思路计算同一截面最大半径与最小半径之差。
