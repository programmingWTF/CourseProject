#pragma once
#include <memory>
#include <string>
#include <vector>

#include "FeatureExtractors.hpp"
#include "FilterBase.hpp"
#include "Logger.hpp"

class PointCloudPipeline {
public:
    // 快照结构：记录每一步的处理结果
    struct StageSnapshot {
        std::string title;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
    };

    PointCloudPipeline(std::shared_ptr<ProcessingLog> logger);

    void addStage(std::shared_ptr<FilterBase> filter);
    void clearStages();

    void setNormalExtractor(std::shared_ptr<NormalExtractor> ne);
    void setCurvatureExtractor(std::shared_ptr<CurvatureExtractor> ce);

    pcl::PointCloud<pcl::Normal>::Ptr getNormals() const;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr getCurvatures() const;

    // 获取处理过程中的快照
    const std::vector<StageSnapshot>& getStageSnapshots() const;

    void execute(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
    std::vector<std::shared_ptr<FilterBase>> stages_;
    std::shared_ptr<ProcessingLog> logger_;

    std::shared_ptr<NormalExtractor> normal_extractor_;
    std::shared_ptr<CurvatureExtractor> curvature_extractor_;

    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures_;

    std::vector<StageSnapshot> stage_snapshots_;
};
