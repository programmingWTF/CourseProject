#pragma once

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include "FeatureBase.hpp"

class NormalExtractor : public FeatureBase<pcl::PointXYZ, pcl::Normal> {
public:
    NormalExtractor(int k_neighbors = 10, double search_radius = 0.0);

    FeatureCloudOutPtr extract(const PointCloudInPtr& input) override;

private:
    int k_neighbors_;
    double search_radius_;
};

class CurvatureExtractor : public FeatureBase<pcl::PointXYZ, pcl::PrincipalCurvatures> {
public:
    CurvatureExtractor(int k_neighbors = 10, double search_radius = 0.0);

    FeatureCloudOutPtr extract(const PointCloudInPtr& input) override;

    /// 设置预先计算好的法线云。如果设置了，extract() 将复用这些法线而不再重新计算。
    /// 如果法线云与 input 点云索引不一致（如中间经过了滤波），调用方负责先清空。
    void setInputNormals(pcl::PointCloud<pcl::Normal>::ConstPtr normals);

private:
    int k_neighbors_;
    double search_radius_;
    pcl::PointCloud<pcl::Normal>::ConstPtr input_normals_;
};