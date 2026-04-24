# Point-Cloud-Pipeline-Control 架构说明

> 交互式点云处理与可视化工具 · ImGui + GLFW + OpenGL + PCL

你可以在这个 GUI 里**搭积木**（滤波 → 提取特征 → 显示），程序实时执行并在多个独立视口里**对比效果**。

---

## 目录结构

```
Point-Cloud-Pipeline-Control/
│
├── include/                        ← 头文件（接口层）
│   ├── FeatureBase.hpp             # 特征提取模板基类
│   ├── FeatureExtractors.hpp       # 法线 / 曲率提取器
│   ├── FilterBase.hpp              # 滤波抽象基类
│   ├── Filters.hpp                 # 三种滤波实现
│   ├── Logger.hpp                  # 日志模块
│   ├── Pipeline.hpp                # 流水线编排
│   └── PointCloudIO.hpp            # 点云文件 IO
│
├── src/                            ← 源文件（实现层）
│   ├── main.cpp            (908行) # 主程序：GUI + 流水线执行器
│   ├── Filters.cpp          (59行) # 滤波实现
│   ├── FeatureExtractors.cpp(52行) # 特征提取实现
│   ├── Pipeline.cpp         (33行) # 流水线编排实现
│   ├── PointCloudIO.cpp     (98行) # 文件读写(.pcd/.ply/.bin)
│   └── Logger.cpp           (43行) # 日志实现
│
├── extern/imgui/                   # 内置 Dear ImGui
├── data/                           # 测试点云
│   ├── 0000000000.bin              # KITTI 二进制点云
│   ├── bun_zipper.ply              # 🐰 Stanford Bunny
│   ├── dragonStandRight_336.ply    # 🐉 Stanford Dragon
│   └── output/                     # 流水线输出目录
│
├── logs/processing_gui.log         # 运行日志
├── CMakeLists.txt                  # 构建配置
├── README.md                       # 使用说明
└── ARCHITECTURE.md                 # ← 本文档
```

---

## 架构分层

### 三层结构总览

**第 1 层 — GUI 层（main.cpp）**

| 子层                | 功能                                                |
| ------------------- | --------------------------------------------------- |
| ImGui 窗口          | 文件浏览 · 流水线编辑 · 参数调整 · 执行 & 保存      |
| PCL Visualizer 线程 | 多视口并行显示：视口 0 原始点云 + 视口 1..N 各步骤  |
| 流水线执行引擎      | 按序执行滤波 → 提取特征 → 截取快照 → 可选保存到文件 |

⬇ _调用_ ⬇

**第 2 层 — 算法层**

| 组件                    | 职责                                                                  |
| ----------------------- | --------------------------------------------------------------------- |
| `Pipeline`              | 编排滤波步骤 + 触发特征提取（法线 / 曲率），核心方法 `execute(cloud)` |
| `FilterBase` + 子类     | 三种滤波算法（PassThrough / VoxelGrid / StatisticalOutlier）          |
| `FeatureBase<T>` + 子类 | 法线估计 · 曲率计算                                                   |

⬇ _调用_ ⬇

**第 3 层 — 基础设施层**

| 组件            | 职责                                   |
| --------------- | -------------------------------------- |
| `PointCloudIO`  | 文件读写（.pcd / .ply / .bin）         |
| `ProcessingLog` | 操作日志（时间戳 · 操作名 · 点数变化） |

---

### 1. Filter 体系 — 「怎么处理点云」

| 类                         | 参数                        | 作用                     |
| -------------------------- | --------------------------- | ------------------------ |
| `FilterBase`（抽象基类）   | `getName()`、`apply(cloud)` | 所有滤波的接口           |
| `PassThroughFilter`        | 坐标轴(x/y/z)、上下限       | **按坐标轴范围裁剪**点云 |
| `VoxelGridFilter`          | 体素边长 `leaf_size`        | **体素网格降采样**       |
| `StatisticalOutlierFilter` | 近邻数、标准差倍数          | **统计法剔除离群点**     |

> 三种滤波都通过 `apply(cloud)` **修改传入的 `shared_ptr`**，指向新生成的点云。

### 2. Feature 体系 — 「提取了什么特征」

#### 两个提取器

| 类                   | 输入→输出                          | 说明                              |
| -------------------- | ---------------------------------- | --------------------------------- |
| `NormalExtractor`    | `PointXYZ` → `Normal`              | 基于 kd-tree 计算每个点的法线方向 |
| `CurvatureExtractor` | `PointXYZ` → `PrincipalCurvatures` | 基于法线进一步计算主曲率          |

> 特征提取**不修改输入点云**，返回新的特征点云。
>
> **关系说明**：`NormalExtractor` 与 `CurvatureExtractor` 在架构上是平级能力；曲率计算会复用法线信息，但并不改变两者在模块设计中的并列关系。

#### 哪些步骤用到了 FeatureBase？

| Pipeline 步骤                                | 使用的 Feature 类                    | 执行时机                                          |
| -------------------------------------------- | ------------------------------------ | ------------------------------------------------- |
| ShowNormals                                  | `NormalExtractor`                    | 先配置提取器，再由 `pipeline->execute()` 统一执行 |
| ComputeCurvature                             | `CurvatureExtractor`（内部复用法线） | 在 pipeline->execute() 中自动执行                 |
| PassThrough / VoxelGrid / StatisticalOutlier | **不使用**                           | —                                                 |
| ShowCloud                                    | **不使用**                           | —                                                 |

### 3. Pipeline — 流水线编排

| 方法                        | 作用                                     |
| --------------------------- | ---------------------------------------- |
| `addStage(filter)`          | 追加一个滤波步骤（保留顺序，支持多个）   |
| `setNormalExtractor(ne)`    | 设置法线提取器（在滤波之后执行）         |
| `setCurvatureExtractor(ce)` | 设置曲率提取器（在滤波之后执行）         |
| `getNormals()`              | 获取上次 `execute()` 的法线结果          |
| `getCurvatures()`           | 获取上次 `execute()` 的曲率结果          |
| `execute(cloud)`            | **核心方法**：按序执行 filter / 特征提取 |

> 特征提取在架构上是并列能力，执行入口统一为 `Pipeline::execute()`。
>
> 1. `ShowNormals` 负责配置 `NormalExtractor`，由 `execute()` 触发计算；
> 2. `ComputeCurvature` 负责配置 `CurvatureExtractor`，由 `execute()` 触发计算。

`execute()` 内部流程：

```
① 遍历 stages_ → 依次执行每个 filter
② 如有 normal_extractor_ → 对滤波结果计算法线
③ 如有 curvature_extractor_ → 对滤波结果计算曲率
④ 每步滤波记录日志（名称+前后点数）
```

> 说明：法线与曲率在特征模块中是平级关系，且现在共享同一个执行入口 `execute()`。

### 4. PointCloudIO — 文件读写

```
load(path)    → pcl::PointCloud<PointXYZ>::Ptr
save<T>(path) → bool  (模板，支持 PointXYZ / PointNormal)

支持的格式：
  · .pcd — PCL 原生格式
  · .ply — Stanford Polygon 格式
  · .bin — KITTI 数据集格式（每 4 float: xyz + intensity，忽略 intensity）
```

### 5. 可视化

> _显示点云的过程完全由 `main.cpp` 完成，`FeatureBase<T>` 只负责计算，不负责渲染。_

#### 显示流程（ShowNormals 为例）

```
  ShowNormals 步骤
    │
    ├─ pipeline->setNormalExtractor(...)
    ├─ capture_display_snapshot() → flush_pending_pipeline()
    ├─ pipeline->execute(cloud)  → 统一执行（滤波 + 法线/曲率特征）
    ├─ pipeline->getNormals()  → 取法线结果
    │
    └─ { snapshot.cloud, snapshot.normals } → VisualizerData

  可视化线程（另一个线程）
    │
    ├─ build_viewer_scene(viewer, original, snapshots...)
    │
    ├─ viewer->addPointCloud(cloud, color, viewport)    ← 加 XYZ
    │
    └─ viewer->addPointCloudNormals(cloud, normals, level, scale, viewport)
         （PCL Visualizer 负责把渲染法线）
```

#### 多视口对比

每次执行流水线后，`build_viewer_scene` 会根据 `snapshots` 数量**动态划分视口**：

| 视口      | 内容       | 颜色方案         |
| --------- | ---------- | ---------------- |
| 0（固定） | 原始点云   | 白色，带坐标轴   |
| 1         | 显示步骤 A | 用户自定义颜色   |
| 2         | 显示步骤 B | 可自定义不同颜色 |
| ...       | ...        | ...              |

每个视口**独立配置**：背景色、点颜色、点大小、坐标轴、法线显隐。

---

### 6. Logger — 日志

```
ProcessingLog("logs/processing_gui.log")

每次操作记录示例：
[2026-04-24 15:30:00] Operation: VoxelGridFilter | Pts before: 35947 | Pts after: 8923 | Diff: -27024
```

---

## main.cpp 内部结构（908 行）

### 数据结构

```
DisplayConfig           → 可视化参数（颜色、点大小、坐标轴、背景色、法线开关/密度/长度）
PipelineStepConfig      → 单个步骤的完整配置（类型 + 各自参数 + 保存选项 + DisplayConfig）
VisualizerData          → 线程安全容器（mutex 保护）：原始点云 + 各步骤快照 + should_update 标志
StepType                → 步骤类型枚举：3 滤波 + 法线显示 + 曲率计算 + 普通显示
```

### 核心工具函数

```cpp
get_output_folder(filepath)                    → "data/output/"
generate_output_filename(folder, step, name)   → "data/output/cloud_01.pcd"

// 合并 XYZ + 法线 + 曲率 → PointNormal 云
build_featured_point_cloud(xyz, normals, curvatures, prev_featured_cloud) → featured_cloud
// ⚠️ prev_featured_cloud 只在点索引一致时有效
//   滤波步骤改变了点索引后必须废弃

setup_imgui_font(size)         → true/false（加载中文字体）
render_display_config_ui(cfg)  → 显示参数 GUI 控件
build_viewer_scene(viewer, ...)→ 构建 PCL 多视口可视化场景
```

### main() 核心流程

```
启动
 │
 ├─ 初始化 GLFW / ImGui / PCL Visualizer 线程
 │
 ├─ 进入主循环（每帧）
 │   │
 │   ├─ 渲染 GUI
 │   │   ├─ 文件选择区
 │   │   ├─ 流水线编辑区（增/删/调序步骤）
 │   │   └─ 执行按钮
 │   │
 │   ├─ 用户点击「执行」时：
 │   │   │
 │   │   ├─ 遍历用户配置的每一步
 │   │   │   │
 │   │   │   ├─ 滤波步骤 → pipeline->addStage(filter)
 │   │   │   │    └─ 勾选保存? → flush 执行 → save → 新建 pipeline
 │   │   │   │
 │   │   │   ├─ 法线步骤（ShowNormals）→ pipeline->setNormalExtractor()
 │   │   │   │    └─ capture_display_snapshot() 内 flush → execute() 统一提取
 │   │   │   │    └─ 勾选保存? → save(含法线) → 新建 pipeline
 │   │   │   │
 │   │   │   ├─ 曲率步骤（ComputeCurvature）→ pipeline->setCurvatureExtractor()
 │   │   │   │    └─ 勾选保存? → flush 执行 → save(含曲率) → 新建 pipeline
 │   │   │   │
 │   │   │   └─ 显示步骤 → capture_display_snapshot()
 │   │   │        = flush → 截快照 → save(可选) → 新建 pipeline
 │   │   │
 │   │   └─ 通知 Visualizer 线程更新画面
 │   │
 │   └─ OpenGL 渲染 + swap buffers
 │
 ├─ 等待 Visualizer 线程结束
 │
 └─ 清理退出
```

---

## 关键数据流（三种典型场景）

### 场景 1：纯处理 + 显示（不保存）

```
用户配置：PassThrough(z,0,2) → VoxelGrid(0.01) → ShowCloud

执行过程：
  PassThrough: addStage(PT)     → pipeline: [PT]
  VoxelGrid:   addStage(VG)     → pipeline: [PT, VG]
  ShowCloud:   capture_display_snapshot()
                 → flush: pipeline->execute(cloud)
                   PT: 10万点 → 8万点
                   VG:  8万点 → 1万点
                 → 截取快照 {title, cloud(1万点), normals(nullptr)}
                 → 通知 Visualizer 刷新

视口结果：
  视口0: 原始点云 (白色, 10万点)
  视口1: ShowCloud 步骤 (绿色, 1万点)
```

### 场景 2：保存 + 特征累积

```
用户配置：ShowNormals(保存) → ShowCloud(保存)

执行过程：
  ShowNormals:
    setNormalExtractor() → pipeline: {normal ready}
    保存 → flush: execute 计算法线
         → save(cloud + normals)   ← 保存为 PointNormal
         → previous_feature_cloud = saved cloud (含法线特征)
         → 新建 pipeline

  ShowCloud:
    capture_display_snapshot()
      → flush (pipeline 为空，无操作)
      → 截取快照
      → 保存: save(cloud + 复用 prev_featured_cloud)
         法线数据保留到了最终输出
```

### 场景 3：⚠️ 滤波后特征失效

```
用户配置：ShowNormals(保存) → ComputeCurvature(保存) → VoxelGrid(保存) → ShowCloud(保存)

执行过程：
  ShowNormals:
    setNormalExtractor()
    保存 → flush → save(cloud + normals)
         → previous_feature_cloud = saved cloud (含法线数据)
         → 新建 pipeline

  ComputeCurvature:
    setCurvatureExtractor()
    保存 → flush → save(cloud + curvatures, 复用 prev_featured_cloud)
         → previous_feature_cloud = saved cloud (同时含法线 + 曲率数据)
         → 新建 pipeline

  VoxelGrid:
    addStage(VG) → pipeline: [VG]
                → filters_pending_in_pipeline = true ← 标记"有滤波"
    保存 → flush: VG 执行，点索引改变！
         → filters_pending_in_pipeline = true
         → previous_feature_cloud_valid = false  废弃！（索引不匹配）
         → save(cloud) ← 只保存 XYZ，不合并旧特征
         → 新建 pipeline

  ShowCloud:
    capture_display_snapshot() → save(cloud, 无 prev_featured)
                                 只保存 XYZ
```

> **核心规则**：`previous_feature_cloud` 只在点索引一致时有效。每当滤波步骤执行后，点索引改变，必须废弃旧的特征数据。

---

## 🧵 线程模型

| 线程                | 职责                        | 循环                            |
| ------------------- | --------------------------- | ------------------------------- |
| **主线程**          | ImGui 事件处理 + 流水线执行 | 每帧(~16ms)                     |
| **Visualizer 线程** | 管理 PCL 可视化窗口         | `spinOnce(10ms)` + `sleep(1ms)` |

**数据同步**：唯一共享状态 `VisualizerData` 用 `std::mutex` 保护。

```
主线程                         Visualizer 线程
  │                               │
  ├─ 写数据（lock）                ├─ while（running）:
  │  cloud_original = ...         │    lock
  │  stage_snapshots = ...        │    检查 should_update
  │  should_update = true         │    如果是 → rebuild_scene()
  │  unlock                       │    unlock
  │                               │    viewer->spinOnce(10ms)
  │                               │    sleep(1ms)
```

---

## 设计原则

1. **模块化** — 滤波、特征、IO、日志各司其职，加新滤波只需继承 `FilterBase`，加新特征只需继承 `FeatureBase<T>`
2. **线程安全** — 唯一的共享状态 `VisualizerData` 用 `mutex` 保护
3. **流水线隔离（仅限中途保存时）** — 当某步骤勾选「保存输出」时，会先执行已积压的步骤 → 保存 → **新建 Pipeline 实例**。这是因为 `execute()` 已消费了当前 pipeline 的执行上下文，后续步骤需要干净状态（全新的 `stages_` + `normal_extractor_` + `curvature_extractor_`）。不保存时全程只用一个 Pipeline，一气呵成执行到底。
4. **可扩展** — `FeatureBase` 模板已保留扩展到 `PointXYZRGB`、`PointXYZI` 的空间；`PointCloudIO::save<T>` 可处理任意点类型
5. **对比友好** — 每步显示独立视口，原始点云固定左上角视口，随时对照
