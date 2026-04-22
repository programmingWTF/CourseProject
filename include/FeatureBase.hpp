#ifndef FEATURE_BASE_HPP
#define FEATURE_BASE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>


template <typename PointInT, typename FeatureOutT>
class FeatureBase {
public:
    using PointCloudIn = pcl::PointCloud<PointInT>;
    using PointCloudInPtr = typename PointCloudIn::Ptr;

    using FeatureCloudOut = pcl::PointCloud<FeatureOutT>;
    using FeatureCloudOutPtr = typename FeatureCloudOut::Ptr;

    virtual ~FeatureBase() = default;

    virtual FeatureCloudOutPtr extract(const PointCloudInPtr& input) = 0;
};

#endif  // FEATURE_BASE_HPP