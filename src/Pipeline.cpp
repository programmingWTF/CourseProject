#include "Pipeline.hpp"

PointCloudPipeline::PointCloudPipeline(std::shared_ptr<ProcessingLog> logger) : logger_(logger) {}

void PointCloudPipeline::addStage(std::shared_ptr<FilterBase> filter) {
    stages_.push_back(filter);
}

void PointCloudPipeline::setNormalExtractor(std::shared_ptr<NormalExtractor> ne) {
    normal_extractor_ = ne;
}

void PointCloudPipeline::setCurvatureExtractor(std::shared_ptr<CurvatureExtractor> ce) {
    curvature_extractor_ = ce;
}

pcl::PointCloud<pcl::Normal>::Ptr PointCloudPipeline::getNormals() const {
    return normals_;
}

pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr PointCloudPipeline::getCurvatures() const {
    return curvatures_;
}

void PointCloudPipeline::execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;

    for (const auto& stage : stages_) {
        size_t origin_count = cloud->size();
        stage->apply(cloud);
        size_t current_count = cloud->size();

        if (logger_) {
            logger_->log(stage->getName(), origin_count, current_count);
        }
    }

    if (normal_extractor_) {
        normals_ = normal_extractor_->extract(cloud);
    }
    if (curvature_extractor_) {
        curvatures_ = curvature_extractor_->extract(cloud);
    }
}
