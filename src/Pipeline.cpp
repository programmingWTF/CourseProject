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
    assert(executed_ && "getNormals() called before execute() — results are not yet available");
    return normals_;
}

pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr PointCloudPipeline::getCurvatures() const {
    assert(executed_ && "getCurvatures() called before execute() — results are not yet available");
    return curvatures_;
}

void PointCloudPipeline::execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;

    normals_.reset();
    curvatures_.reset();
    executed_ = true;

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
        // 如果已经计算了法线，直接复用，避免 CurvatureExtractor 内部重复计算
        if (normals_ && !normals_->empty()) {
            curvature_extractor_->setInputNormals(normals_);
        }
        curvatures_ = curvature_extractor_->extract(cloud);
    }
}
