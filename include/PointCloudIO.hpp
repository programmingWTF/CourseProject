#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

class PointCloudIO {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr load(const std::string& filename);
    static bool save(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static bool save(const std::string& filename, const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud);
};
