#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointInT, typename FeatureOutT>
class FeatureBase {
public:
    using InputPointType = PointInT;
    using OutputFeatureType = FeatureOutT;

    using PointCloudIn = pcl::PointCloud<PointInT>;
    using PointCloudInPtr = typename PointCloudIn::Ptr;

    using FeatureCloudOut = pcl::PointCloud<FeatureOutT>;
    using FeatureCloudOutPtr = typename FeatureCloudOut::Ptr;

    virtual ~FeatureBase() = default;

    virtual FeatureCloudOutPtr extract(const PointCloudInPtr& input) = 0;
};