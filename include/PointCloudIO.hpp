#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

class PointCloudIO {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr load(const std::string& filename);

    // 模板 save：直接接受点云引用（而非 Ptr 类型），模板推导不受嵌套依赖类型限制
    template <typename PointT>
    static bool save(const std::string& filename, const pcl::PointCloud<PointT>& cloud);
};
