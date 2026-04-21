#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

class FilterBase {
public:
    virtual ~FilterBase() = default;

    // 返回滤波器名称，用于日志记录
    virtual std::string getName() const = 0;

    // 执行滤波处理
    virtual void apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) = 0;
};
