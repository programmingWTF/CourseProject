#pragma once
#include <memory>
#include <string>
#include <vector>

#include "FeatureExtractors.hpp"
#include "FilterBase.hpp"
#include "Logger.hpp"

class PointCloudPipeline {
public:
    PointCloudPipeline(std::shared_ptr<ProcessingLog> logger);

    void addStage(std::shared_ptr<FilterBase> filter);
    void setNormalExtractor(std::shared_ptr<NormalExtractor> ne);
    void setCurvatureExtractor(std::shared_ptr<CurvatureExtractor> ce);

    pcl::PointCloud<pcl::Normal>::Ptr getNormals() const;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr getCurvatures() const;

    void execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
    std::vector<std::shared_ptr<FilterBase>> stages_;
    std::shared_ptr<ProcessingLog> logger_;

    std::shared_ptr<NormalExtractor> normal_extractor_;
    std::shared_ptr<CurvatureExtractor> curvature_extractor_;

    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures_;
};
