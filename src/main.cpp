#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <atomic>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "FeatureExtractors.hpp"
#include "Filters.hpp"
#include "Logger.hpp"
#include "Pipeline.hpp"
#include "PointCloudIO.hpp"

namespace fs = std::filesystem;

// ================================================================
// 线程安全的可视化数据容器
// ================================================================
struct VisualizerData {
    std::mutex mutex;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    bool should_update = false;
    bool show_normals = false;
    int normal_display_level = 10;

    VisualizerData() {
        cloud_original.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
        normals.reset(new pcl::PointCloud<pcl::Normal>());
    }
};

// ================================================================
// PCL 可视化线程函数
// ================================================================
void visualizer_thread_func(std::shared_ptr<VisualizerData> vis_data, std::atomic<bool>& keep_running) {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("PCL Viewer (Left: Original | Middle: Processed | Right: Normals)"));

    viewer->setBackgroundColor(0.05, 0.05, 0.05);
    viewer->addCoordinateSystem(0.1);

    int v1 = 0, v2 = 0, v3 = 0;
    viewer->createViewPort(0.0, 0.0, 0.3333, 1.0, v1);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, v1);
    viewer->addText("Original", 10, 10, "v1 text", v1);

    viewer->createViewPort(0.3333, 0.0, 0.6666, 1.0, v2);
    viewer->setBackgroundColor(0.08, 0.08, 0.08, v2);
    viewer->addText("Filtered", 10, 10, "v2 text", v2);

    viewer->createViewPort(0.6666, 0.0, 1.0, 1.0, v3);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v3);
    viewer->addText("Filtered & Features", 10, 10, "v3 text", v3);

    bool visualizer_initialized = false;

    while (keep_running.load() && !viewer->wasStopped()) {
        {
            std::lock_guard<std::mutex> lock(vis_data->mutex);

            if (!visualizer_initialized && vis_data->cloud_original && !vis_data->cloud_original->empty()) {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_orig(vis_data->cloud_original,
                                                                                           255, 255, 255);
                viewer->addPointCloud<pcl::PointXYZ>(vis_data->cloud_original, color_orig, "cloud_orig", v1);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_orig",
                                                         v1);
                visualizer_initialized = true;
            }

            if (vis_data->should_update) {
                viewer->removeAllPointClouds(v2);
                viewer->removeAllPointClouds(v3);
                viewer->removeAllShapes(v3);

                if (vis_data->cloud_filtered && !vis_data->cloud_filtered->empty()) {
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_filt(vis_data->cloud_filtered,
                                                                                               0, 255, 0);
                    viewer->addPointCloud<pcl::PointXYZ>(vis_data->cloud_filtered, color_filt, "cloud_filt_v2", v2);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                             "cloud_filt_v2", v2);

                    viewer->addPointCloud<pcl::PointXYZ>(vis_data->cloud_filtered, color_filt, "cloud_filt_v3", v3);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                                             "cloud_filt_v3", v3);

                    if (vis_data->show_normals && vis_data->normals && !vis_data->normals->empty()) {
                        viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
                            vis_data->cloud_filtered, vis_data->normals, vis_data->normal_display_level, 0.05,
                            "normals_v3", v3);
                    }
                }
                vis_data->should_update = false;
            }
        }

        viewer->spinOnce(10);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    viewer->close();
}

// ================================================================
// 错误回调
// ================================================================
static void glfw_error_callback(int error, const char* description) {
    std::cerr << "Glfw Error " << error << ": " << description << std::endl;
}

// ================================================================
// 主函数：ImGui 线程
// ================================================================
int main(int argc, char** argv) {
    // 初始化 GLFW
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(450, 900, "Point Cloud Pipeline Control", NULL, NULL);
    if (window == NULL) return 1;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 初始化 ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // 创建线程安全的可视化数据容器
    auto vis_data = std::make_shared<VisualizerData>();
    std::atomic<bool> keep_visualizer_running(true);

    // 启动后台可视化线程
    std::thread visualizer_thread(visualizer_thread_func, vis_data, std::ref(keep_visualizer_running));

    // 主线程（ImGui）的状态变量
    std::string current_file = "";
    bool cloud_loaded = false;

    float pass_z_min = -100.0f, pass_z_max = 100.0f;
    float leaf_size = 0.01f;
    int sor_k = 50;
    float sor_std_dev = 1.0f;
    bool enable_normals = true, enable_curvature = false;
    int normal_ksearch = 10, curvature_ksearch = 10;

    auto logger = std::make_shared<ProcessingLog>("logs/processing_gui.log");

    // ================================================================
    // ImGui 主循环（主线程）
    // ================================================================
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin("Pipeline Control Panel", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

        // ========== 文件浏览器 ==========
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "[1] Load Point Cloud");
        ImGui::Text("Available .pcd, .ply, and .bin files in ./data/:");
        ImGui::BeginChild("FileBrowser", ImVec2(0, 120), true);

        std::string data_path = "data";
        if (fs::exists(data_path) && fs::is_directory(data_path)) {
            for (const auto& entry : fs::directory_iterator(data_path)) {
                if (entry.path().extension() == ".pcd" || entry.path().extension() == ".ply" ||
                    entry.path().extension() == ".bin") {
                    std::string filename = entry.path().filename().string();
                    if (ImGui::Selectable(filename.c_str(), current_file == filename)) {
                        current_file = filename;
                        auto cloud = PointCloudIO::load(entry.path().string());
                        if (cloud && !cloud->empty()) {
                            cloud_loaded = true;
                            {
                                std::lock_guard<std::mutex> lock(vis_data->mutex);
                                vis_data->cloud_original = cloud;
                                vis_data->should_update = true;
                            }
                            std::cout << "[INFO] Loaded " << filename << " (" << cloud->size() << " points)"
                                      << std::endl;
                        }
                    }
                }
            }
        } else {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "ERROR: ./data/ directory not found!");
        }
        ImGui::EndChild();

        ImGui::Separator();
        ImGui::Spacing();

        // ========== 滤波器配置 ==========
        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[2] Filter Configuration");
        ImGui::DragFloatRange2("PassThrough Z Min/Max", &pass_z_min, &pass_z_max, 0.5f, -200.0f, 200.0f);
        ImGui::SliderFloat("VoxelGrid Leaf Size", &leaf_size, 0.001f, 0.5f, "%.3f m");
        ImGui::SliderInt("StatOutlier Mean K", &sor_k, 1, 200);
        ImGui::SliderFloat("StatOutlier Std Dev", &sor_std_dev, 0.1f, 5.0f);

        ImGui::Separator();
        ImGui::Spacing();

        // ========== 特征提取配置 ==========
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.4f, 1.0f), "[3] Feature Extraction");
        ImGui::Checkbox("Calculate & Display Normals", &enable_normals);
        if (enable_normals) {
            ImGui::SliderInt("Normal KSearch", &normal_ksearch, 3, 100);
            ImGui::SliderInt("Normal Display Level", (int*)&vis_data->normal_display_level, 1, 50);
        }
        ImGui::Checkbox("Calculate Curvature", &enable_curvature);
        if (enable_curvature) {
            ImGui::SliderInt("Curvature KSearch", &curvature_ksearch, 3, 100);
        }

        ImGui::Separator();
        ImGui::Spacing();

        // ========== "应用并刷新"按钮 ==========
        if (ImGui::Button("Apply & Refresh Pipeline", ImVec2(-1, 50)) && cloud_loaded) {
            {
                std::lock_guard<std::mutex> lock(vis_data->mutex);

                // 复制原始点云用于处理
                auto cloud_to_process = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*vis_data->cloud_original);

                // 装配流水线
                PointCloudPipeline pipeline(logger);
                pipeline.addStage(std::make_shared<PassThroughFilter>("z", pass_z_min, pass_z_max));
                pipeline.addStage(std::make_shared<VoxelGridFilter>(leaf_size));
                pipeline.addStage(std::make_shared<StatisticalOutlierFilter>(sor_k, sor_std_dev));

                if (enable_normals) {
                    pipeline.setNormalExtractor(std::make_shared<NormalExtractor>(normal_ksearch, 0.0));
                }
                if (enable_curvature) {
                    pipeline.setCurvatureExtractor(std::make_shared<CurvatureExtractor>(curvature_ksearch, 0.0));
                }

                // 执行处理
                pipeline.execute(cloud_to_process);

                // 将结果传递给可视化线程
                vis_data->cloud_filtered = cloud_to_process;
                if (enable_normals) {
                    vis_data->normals = pipeline.getNormals();
                    vis_data->show_normals = true;
                } else {
                    vis_data->show_normals = false;
                }
                vis_data->should_update = true;

                std::cout << "[INFO] Pipeline executed. Filtered size: " << cloud_to_process->size() << std::endl;
            }
        }

        if (!cloud_loaded) {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Load a point cloud first!");
        }

        ImGui::End();

        // 渲染 ImGui
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.12f, 0.12f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 关闭可视化线程
    keep_visualizer_running = false;
    visualizer_thread.join();

    // 清理 ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}