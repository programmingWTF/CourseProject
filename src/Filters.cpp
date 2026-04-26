#include "Filters.hpp"

namespace {

/// 抽取 PCL 滤波器的公共模式：setInputCloud → filter → 替换指针
template <typename PclFilter>
void apply_pcl_filter(PclFilter& pcl_filter, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl_filter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_filter.filter(*filtered_cloud);
    cloud = filtered_cloud;
}

}  // namespace

// PassThroughFilter
PassThroughFilter::PassThroughFilter(const std::string& field_name, float limit_min, float limit_max)
    : field_name_(field_name), limit_min_(limit_min), limit_max_(limit_max) {}

std::string PassThroughFilter::getName() const {
    return "PassThroughFilter (" + field_name_ + ")";
}

void PassThroughFilter::apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName(field_name_);
    pass.setFilterLimits(limit_min_, limit_max_);
    apply_pcl_filter(pass, cloud);
}

// VoxelGridFilter
VoxelGridFilter::VoxelGridFilter(float leaf_size) : leaf_size_(leaf_size) {}

std::string VoxelGridFilter::getName() const {
    return "VoxelGridFilter (leaf_size: " + std::to_string(leaf_size_) + ")";
}

void VoxelGridFilter::apply(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    apply_pcl_filter(voxel, cloud);
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
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_dev_mul_thresh_);
    apply_pcl_filter(sor, cloud);
}
