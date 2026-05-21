#include "PointCloudIO.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

namespace {

std::string get_file_extension(const std::string& filename) {
    return fs::path(filename).extension().string();
}

}  // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudIO::load(const std::string& filename) {
    if (filename.empty()) {
        std::cerr << "Error: empty filename\n";
        return nullptr;
    }

    if (!fs::exists(filename)) {
        std::cerr << "Error: file not found: " << filename << "\n";
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        std::string ext = get_file_extension(filename);
        if (ext == ".pcd") {
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
                throw std::runtime_error("Failed to read PCD file.");
            }
            std::cout << "Loaded " << cloud->size() << " data points from " << filename << "";
            return cloud;
        }

        if (ext == ".ply") {
            if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) {
                throw std::runtime_error("Failed to read PLY file.");
            }
            std::cout << "Loaded " << cloud->size() << " data points from " << filename << "";
            return cloud;
        }

        if (ext == ".bin") {
            std::ifstream input(filename, std::ios::binary);
            if (!input.is_open()) {
                throw std::runtime_error("Failed to open .bin file.");
            }

            // 支持两种常见的 .bin 点云格式：
            //   - KITTI 格式：每点 4 个 float (x, y, z, intensity)
            //   - XYZ-only 格式：每点 3 个 float (x, y, z)
            // 自动根据文件大小判断：文件可被 16 整除 → KITTI 格式；否则 → XYZ 格式
            input.seekg(0, std::ios::end);
            auto file_size = input.tellg();
            input.seekg(0, std::ios::beg);

            const bool is_kitti_format = (file_size % 16 == 0) && (file_size % 12 != 0);
            const size_t point_size = is_kitti_format ? 16 : 12;
            const size_t num_points = file_size / point_size;

            cloud->reserve(num_points);

            if (is_kitti_format) {
                float data[4];  // x, y, z, intensity
                for (size_t i = 0; i < num_points; ++i) {
                    if (!input.read(reinterpret_cast<char*>(data), sizeof(data))) break;
                    pcl::PointXYZ point;
                    point.x = data[0];
                    point.y = data[1];
                    point.z = data[2];
                    cloud->push_back(point);
                }
            } else {
                float data[3];  // x, y, z
                for (size_t i = 0; i < num_points; ++i) {
                    if (!input.read(reinterpret_cast<char*>(data), sizeof(data))) break;
                    pcl::PointXYZ point;
                    point.x = data[0];
                    point.y = data[1];
                    point.z = data[2];
                    cloud->push_back(point);
                }
            }
            input.close();
            std::cout << "Loaded " << cloud->size() << " data points from " << filename
                      << " (" << (is_kitti_format ? "KITTI" : "XYZ") << " format)";
            return cloud;
        }

        throw std::runtime_error("Unsupported file extension: " + ext);
    } catch (const std::exception& e) {
        std::cerr << "Error loading file " << filename << ": " << e.what() << "\n";
        return nullptr;
    }
}

template <typename PointT>
bool PointCloudIO::save(const std::string& filename, const pcl::PointCloud<PointT>& cloud) {
    if (cloud.empty()) {
        std::cerr << "Cannot save empty point cloud." << "\n";
        return false;
    }
    try {
        std::string ext = get_file_extension(filename);
        if (ext == ".bin") {
            ext = ".pcd";
        }
        if (ext == ".pcd") {
            pcl::io::savePCDFileASCII(filename, cloud);
            std::cout << "Saved " << cloud.size() << " points to " << filename << "\n";
            return true;
        }

        if (ext == ".ply") {
            pcl::io::savePLYFileASCII(filename, cloud);
            std::cout << "Saved " << cloud.size() << " points to " << filename << "\n";
            return true;
        }

        throw std::runtime_error("Unsupported file extension for saving: " + ext);
    } catch (const std::exception& e) {
        std::cerr << "Error saving file " << filename << ": " << e.what() << "\n";
        return false;
    }
}

// 显式实例化
template bool PointCloudIO::save<pcl::PointXYZ>(const std::string&, const pcl::PointCloud<pcl::PointXYZ>&);
template bool PointCloudIO::save<pcl::PointNormal>(const std::string&, const pcl::PointCloud<pcl::PointNormal>&);
