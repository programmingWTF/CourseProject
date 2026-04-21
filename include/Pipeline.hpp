#pragma once
#include <memory>
#include <vector>

#include "FilterBase.hpp"
#include "Logger.hpp"

class PointCloudPipeline {
public:
    PointCloudPipeline(std::shared_ptr<ProcessingLog> logger);

    void addStage(std::shared_ptr<FilterBase> filter);
    void execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
    std::vector<std::shared_ptr<FilterBase>> stages_;
    std::shared_ptr<ProcessingLog> logger_;
};
