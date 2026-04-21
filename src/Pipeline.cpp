#include "Pipeline.hpp"

PointCloudPipeline::PointCloudPipeline(std::shared_ptr<ProcessingLog> logger) : logger_(logger) {}

void PointCloudPipeline::addStage(std::shared_ptr<FilterBase> filter) {
    stages_.push_back(filter);
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
}
