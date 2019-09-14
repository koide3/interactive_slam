#include <hdl_graph_slam/plane_detection_modal.hpp>

#include <glk/primitives/primitives.hpp>

#include <g2o/types/slam3d/vertex_se3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>

namespace hdl_graph_slam {

PlaneDetectionModal::PlaneDetectionModal(InteractiveGraphView& graph)
: show_window(false),
graph(graph),
center_point(0.0f, 0.0f, 0.0f),
min_cluster_size(50),
max_cluster_size(10000),
num_neighbors(10),
smoothness_threshold(20.0f),
curvature_threshold(0.5f)
{}

PlaneDetectionModal::~PlaneDetectionModal() {}

void PlaneDetectionModal::set_center_point(const Eigen::Vector3f& point) {
  region_growing_result.reset();
  center_point = point;
}

void PlaneDetectionModal::show() {
  show_window = true;
}

RegionGrowingResult::Ptr PlaneDetectionModal::region_growing() {
  RegionGrowingResult::Ptr result(new RegionGrowingResult());

  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_points(new pcl::PointCloud<pcl::PointXYZI>());

  for (const auto& keyframe : graph.keyframes) {
    std::vector<int> neighbor_indices = keyframe.second->neighbors(center_point, 1.0f);
    if(neighbor_indices.size() < 10) {
      continue;
    }

    result->candidates.push_back(keyframe.second);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*keyframe.second->cloud, *cloud, keyframe.second->node->estimate().cast<float>());

    if(accumulated_points->empty()) {
      pcl::PointNormal pt;
      accumulated_points->push_back(cloud->at(neighbor_indices[0]));
    }

    for(int i=0; i<cloud->size(); i++) {
      accumulated_points->push_back(cloud->at(i));
    }
  }

  pcl::PointCloud<pcl::Normal>::Ptr accumulated_normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setInputCloud(accumulated_points);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.25f);

  ne.compute(*accumulated_normals);

  pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
  reg.setMinClusterSize(min_cluster_size);
  reg.setMaxClusterSize(max_cluster_size);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>());
  kdtree->setInputCloud(accumulated_points);
  reg.setSearchMethod(kdtree);

  reg.setNumberOfNeighbours(num_neighbors);
  reg.setInputCloud(accumulated_points);
  reg.setInputNormals(accumulated_normals);
  reg.setSmoothnessThreshold(smoothness_threshold / 180.0f * M_PI);
  reg.setCurvatureThreshold(curvature_threshold);

  pcl::PointIndices::Ptr cluster(new pcl::PointIndices());
  reg.getSegmentFromPoint(0, *cluster);

  result->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  result->normals.reset(new pcl::PointCloud<pcl::Normal>());

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(accumulated_points);
  extract.setIndices(cluster);
  extract.setNegative(false);
  extract.filter(*result->cloud);

  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setInputCloud(accumulated_normals);
  extract_normals.setIndices(cluster);
  extract_normals.setNegative(false);
  extract_normals.filter(*result->normals);

  // result->plane_cloud = accumulated_points;
  result->cloud_buffer = std::make_shared<glk::PointCloudBuffer>(result->cloud);

  return result;
}

void PlaneDetectionModal::draw_ui() {
  if(!show_window) {
    region_growing_result.reset();
    return;
  }

  ImGui::Begin("plane detection", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::Text("Region growing");
  ImGui::DragInt("Min cluster size", &min_cluster_size, 1, 10, 10000);
  ImGui::DragInt("Max cluster size", &max_cluster_size, 100, 100, 1000000);
  ImGui::DragInt("Num neighbors", &num_neighbors, 1, 1, 1000);
  ImGui::DragFloat("Smoothness threshold[deg]", &smoothness_threshold, 0.1f, 0.1f, 100.0f);
  ImGui::DragFloat("Curvature threshold", &curvature_threshold, 0.1f, 0.1f, 100.0f);

  if(ImGui::Button("Perform")) {
    region_growing_result = region_growing();
  }

  ImGui::Text("RANSAC plane");
  if (ImGui::Button("Detect")) {
  }

  ImGui::End();
}

void PlaneDetectionModal::draw_gl(glk::GLSLShader& shader) {
  if(!show_window) {
    return;
  }

  shader.set_uniform("color_mode", 1);
  shader.set_uniform("point_scale", 200.0f);
  shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f));

  if(region_growing_result) {
    shader.set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
    region_growing_result->cloud_buffer->draw(shader);

    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    for(const auto& candidate: region_growing_result->candidates){
      Eigen::Matrix4f model_matrix = candidate->node->estimate().cast<float>().matrix();
      model_matrix.block<3, 3>(0, 0) *= 0.5f;
      shader.set_uniform("model_matrix", model_matrix);
      sphere.draw(shader);
    }
  }
}


}  // namespace hdl_graph_slam