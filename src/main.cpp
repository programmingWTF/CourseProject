#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <thread>

#include "Filters.hpp"
#include "Logger.hpp"
#include "Pipeline.hpp"
#include "PointCloudIO.hpp"

int main(int argc, char** argv) {
    std::string input_file = "data/input.pcd";
    if (argc > 1) {
        input_file = argv[1];
    }

    // 1. 读取点云
    auto cloud = PointCloudIO::load(input_file);
    if (!cloud || cloud->empty()) {
        std::cerr << "[Error] Initialization failed: Please make sure the input file exists at " << input_file << "\n";
        return -1;
    }

    // 保留原始点云以用于对比可视化
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

    // 2. 初始化日志与流水线
    auto logger = std::make_shared<ProcessingLog>("logs/processing.log");
    PointCloudPipeline pipeline(logger);

    // 3. 按顺序装载滤波器 (可根据需求配置具体参数)
    // a. 裁剪 Z 轴范围 (扩大范围防止切掉太多点，或者注释掉这一行来验证)
    pipeline.addStage(std::make_shared<PassThroughFilter>("z", -100.0f, 100.0f));

    // b. 体素下采样，减小数据量 (1cm的叶子大小)
    pipeline.addStage(std::make_shared<VoxelGridFilter>(0.01f));

    // c. 统计滤波去噪
    pipeline.addStage(std::make_shared<StatisticalOutlierFilter>(50, 1.0f));

    // 4. 执行滤波流水线
    std::cout << "Starting pipeline execution..." << "\n";
    pipeline.execute(cloud);
    std::cout << "Pipeline execution finished. Final size: " << cloud->size() << "\n";

    // 5. 保存结果
    PointCloudIO::save("data/output.pcd", cloud);

    // 6. 可视化
    std::cout << "Opening visualizer..." << "\n";
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Cloud Viewer (Left: Original | Right: Filtered)"));

    int v1 = 0;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, v1);
    viewer->addText("Original", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(original_cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(original_cloud, color1, "original_cloud", v1);

    int v2 = 0;
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("Filtered", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud, 0, 255, 0);  // 绿色表示处理后
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color2, "filtered_cloud", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_cloud");
    // 将坐标轴调细，参数 0.1 表示比例，根据需要可调
    viewer->addCoordinateSystem(0.1);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
