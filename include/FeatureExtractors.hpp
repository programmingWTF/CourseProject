#ifndef FEATURE_EXTRACTORS_HPP
#define FEATURE_EXTRACTORS_HPP

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

private:
    int k_neighbors_;
    double search_radius_;
};

#endif  // FEATURE_EXTRACTORS_HPP