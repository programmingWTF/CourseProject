# Point Cloud Pipeline Control

点云处理流水线可视化控制台

基于 **PCL (Point Cloud Library)** + **ImGui** + **OpenGL** 的点云处理工具，支持交互式构建滤波-特征提取-可视化流水线。

## 功能

- 📂 支持 `.pcd` / `.ply` / `.bin` (KITTI / XYZ) 格式的点云文件
- 🔧 滤波器：`PassThrough` / `VoxelGrid` / `StatisticalOutlier`
- 📐 特征提取：法线估计 + 主曲率计算
- 🎨 多视口 3D 可视化（原始点云 + 各步骤结果）
- 🧩 拖拽式流水线编辑器（步骤增删改、排序、参数调节）
- 💾 流水线执行结果按步骤保存到 `output/` 目录
- 🌐 中文 ImGui 界面（微软雅黑）

## 依赖

| 依赖 | 版本要求 | 安装方式 |
|------|---------|---------|
| PCL | ≥ 1.8 | [vcpkg](https://github.com/microsoft/vcpkg) 或 [官网](https://pointclouds.org/) |
| GLFW 3 | - | vcpkg: `vcpkg install glfw3` |
| OpenGL | - | 系统自带 |
| CMake | ≥ 3.10 | [cmake.org](https://cmake.org/) |
| MSVC / Clang / GCC | C++17 | VS 2022 / Clang 14+ / GCC 11+ |

> 实际开发环境：Visual Studio 2022 + vcpkg (x64-windows) + MSVC

## 编译

```bash
# 1. 配置（指定 vcpkg toolchain）
cmake -B build -DCMAKE_TOOLCHAIN_FILE=D:/Software/vcpkg/scripts/buildsystems/vcpkg.cmake

# 2. 编译
cmake --build build --config Release
```

> 如果 vcpkg 路径不同，请修改 `CMAKE_TOOLCHAIN_FILE` 为你的 vcpkg 安装路径。  
> 也可以在 VS 中直接打开 CMakeLists.txt，IDE 会自动检测 vcpkg。

## 使用

```bash
cd build/Release
./Point-Cloud-Pipeline-Control.exe
```

1. **加载点云**：在搜索文件夹中输入数据目录（默认 `../data`），点击文件名选择，再点"加载当前选择文件"。也可点击"浏览..."按钮用系统对话框选择文件。
2. **编辑流水线**：拖拽步骤排序、修改参数（如滤波轴/范围、体素大小、曲率参数等）
3. **执行**：点击"执行流水线"按钮，后台异步执行（不影响 UI 操作）
4. **查看结果**：PCL Viewer 窗口会显示多视口对比；勾选"保存输出"可在源文件目录的 `output/` 文件夹下保存中间结果
5. **重置**：点击"重置"按钮恢复到原始点云

## 项目结构

```
CourseProject/
├── CMakeLists.txt          # CMake 构建配置
├── README.md
├── .clang-format           # 代码格式化规则
├── .clang-tidy             # 静态分析规则
├── include/                # 头文件
│   ├── FeatureBase.hpp     # 特征提取基类
│   ├── FeatureExtractors.hpp  # 法线/曲率提取器
│   ├── FilterBase.hpp      # 滤波器基类
│   ├── Filters.hpp         # PassThrough/VoxelGrid/StatisticalOutlier
│   ├── Logger.hpp          # 处理日志
│   ├── Pipeline.hpp        # 流水线引擎
│   └── PointCloudIO.hpp    # 点云 IO（读写）
├── src/                    # 源文件
│   ├── main.cpp            # GUI 主程序
│   ├── FeatureExtractors.cpp
│   ├── Filters.cpp
│   ├── Logger.cpp
│   ├── Pipeline.cpp
│   └── PointCloudIO.cpp
├── extern/imgui/           # Dear ImGui 源码
├── fonts/                  # 字体文件
└── data/                   # 示例点云数据
    ├── bun_zipper.ply
    ├── dragonStandRight_336.ply
    └── 0000000000.bin      # KITTI 格式
```

## 架构

```
[文件加载] → [ImGui 流水线编辑器] → [异步后台执行]
                 ↓
[Filter Pipeline (Strategy Pattern)]
  ├── PassThroughFilter
  ├── VoxelGridFilter
  └── StatisticalOutlierFilter
                 ↓
[Feature Extraction (Template Base)]
  ├── NormalExtractor
  └── CurvatureExtractor
                 ↓
[PCLVisualizer 多视口 3D 渲染]
```

## 已知限制

- `.bin` 文件自动检测 KITTI (16B/点) vs XYZ (12B/点) 格式，不支持其他变体
- 仅支持 Windows (ImGui + GLFW + Win32 原生对话框)
- 可视化窗口需要 OpenGL 3.0+ 支持

## License

MIT - 课程设计项目
