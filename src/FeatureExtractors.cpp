#include "FeatureExtractors.hpp"

#include <pcl/search/kdtree.h>

NormalExtractor::NormalExtractor(int k_neighbors, double search_radius)
    : k_neighbors_(k_neighbors), search_radius_(search_radius) {}

NormalExtractor::FeatureCloudOutPtr NormalExtractor::extract(const PointCloudInPtr& input) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    FeatureCloudOutPtr cloud_normals(new FeatureCloudOut());

    if (search_radius_ > 0.0) {
        ne.setRadiusSearch(search_radius_);
    } else {
        ne.setKSearch(k_neighbors_);
    }

    ne.compute(*cloud_normals);

    return cloud_normals;
}

CurvatureExtractor::CurvatureExtractor(int k_neighbors, double search_radius)
    : k_neighbors_(k_neighbors), search_radius_(search_radius) {}

CurvatureExtractor::FeatureCloudOutPtr CurvatureExtractor::extract(const PointCloudInPtr& input) {
    NormalExtractor ne(k_neighbors_, search_radius_);
    auto normals = ne.extract(input);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc_est;
    pc_est.setInputCloud(input);
    pc_est.setInputNormals(normals);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pc_est.setSearchMethod(tree);

    FeatureCloudOutPtr principal_curvatures(new FeatureCloudOut());

    if (search_radius_ > 0.0) {
        pc_est.setRadiusSearch(search_radius_);
    } else {
        pc_est.setKSearch(k_neighbors_);
    }

    pc_est.compute(*principal_curvatures);

    return principal_curvatures;
}