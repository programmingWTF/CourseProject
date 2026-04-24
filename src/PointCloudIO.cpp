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

            // 每次读取 4 个 float: x, y, z, intensity
            float data[4];
            while (input.read(reinterpret_cast<char*>(data), sizeof(data))) {
                pcl::PointXYZ point;
                point.x = data[0];
                point.y = data[1];
                point.z = data[2];
                // 暂时忽略 intensity，因为 cloud 类型是 PointXYZ
                cloud->push_back(point);
            }
            input.close();
            std::cout << "Loaded " << cloud->size() << " data points from " << filename << "";
            return cloud;
        }

        throw std::runtime_error("Unsupported file extension: " + ext);
    } catch (const std::exception& e) {
        std::cerr << "Error loading file " << filename << ": " << e.what() << "\n";
        return nullptr;
    }
}

bool PointCloudIO::save(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Cannot save empty or null point cloud." << "\n";
        return false;
    }
    try {
        std::string ext = get_file_extension(filename);
        if (ext == ".pcd") {
            pcl::io::savePCDFileASCII(filename, *cloud);
            std::cout << "Saved " << cloud->size() << " data points to " << filename << "\n";
            return true;
        }

        if (ext == ".ply") {
            pcl::io::savePLYFileASCII(filename, *cloud);
            std::cout << "Saved " << cloud->size() << " data points to " << filename << "\n";
            return true;
        }

        if (ext == ".bin") {
            std::ofstream output(filename, std::ios::binary);
            if (!output.is_open()) {
                throw std::runtime_error("Failed to open .bin file for writing.");
            }
            for (const auto& point : cloud->points) {
                float data[4] = {point.x, point.y, point.z, 0.0F};  // intensity 设为默认 0
                output.write(reinterpret_cast<const char*>(data), sizeof(data));
            }
            output.close();
            std::cout << "Saved " << cloud->size() << " data points to " << filename << "\n";
            return true;
        }
        throw std::runtime_error("Unsupported file extension for saving: " + ext);
    } catch (const std::exception& e) {
        std::cerr << "Error saving file " << filename << ": " << e.what() << "\n";
        return false;
    }
}
