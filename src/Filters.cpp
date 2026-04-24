#include "Filters.hpp"

// PassThroughFilter
PassThroughFilter::PassThroughFilter(const std::string& field_name, float limit_min, float limit_max)
    : field_name_(field_name), limit_min_(limit_min), limit_max_(limit_max) {}

std::string PassThroughFilter::getName() const {
    return "PassThroughFilter (" + field_name_ + ")";
}

void PassThroughFilter::apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field_name_);
    pass.setFilterLimits(limit_min_, limit_max_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*filtered_cloud);
    cloud = filtered_cloud;  // 更新指针
}

// VoxelGridFilter
VoxelGridFilter::VoxelGridFilter(float leaf_size) : leaf_size_(leaf_size) {}

std::string VoxelGridFilter::getName() const {
    return "VoxelGridFilter (leaf_size: " + std::to_string(leaf_size_) + ")";
}

void VoxelGridFilter::apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*filtered_cloud);
    cloud = filtered_cloud;
}

// StatisticalOutlierFilter
StatisticalOutlierFilter::StatisticalOutlierFilter(int mean_k, float std_dev_mul_thresh)
    : mean_k_(mean_k), std_dev_mul_thresh_(std_dev_mul_thresh) {}

std::string StatisticalOutlierFilter::getName() const {
    return "StatisticalOutlierFilter (meanK: " + std::to_string(mean_k_) + ")";
}

void StatisticalOutlierFilter::apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_dev_mul_thresh_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_cloud);
    cloud = filtered_cloud;
}
