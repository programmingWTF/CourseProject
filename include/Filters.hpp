#pragma once
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "FilterBase.hpp"

class PassThroughFilter : public FilterBase {
public:
    PassThroughFilter(const std::string& field_name, float limit_min, float limit_max);
    std::string getName() const override;
    void apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override;

private:
    std::string field_name_;
    float limit_min_;
    float limit_max_;
};

class VoxelGridFilter : public FilterBase {
public:
    VoxelGridFilter(float leaf_size);
    std::string getName() const override;
    void apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override;

private:
    float leaf_size_;
};

class StatisticalOutlierFilter : public FilterBase {
public:
    StatisticalOutlierFilter(int mean_k, float std_dev_mul_thresh);
    std::string getName() const override;
    void apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override;

private:
    int mean_k_;
    float std_dev_mul_thresh_;
};
