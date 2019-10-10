#include <hdl_graph_slam/plane_detection_window.hpp>

#include <glk/colormap.hpp>
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

PlaneDetectionWindow::PlaneDetectionWindow(std::shared_ptr<InteractiveGraphView>& graph)
    : show_window(false),
      graph(graph),
      center_point(0.0f, 0.0f, 0.0f),
      initial_neighbor_search_radius(1.0f),
      normal_estimation_radius(1.0f),
      min_cluster_size(50),
      max_cluster_size(10000),
      num_neighbors(10),
      smoothness_threshold(10.0f),
      curvature_threshold(0.5f),
      ransac_distance_thresh(0.25f),
      min_plane_supports(100),
      robust_kernel(1),
      robust_kernel_delta(0.1f),
      region_growing_progress_modal("region growing progress") {}

PlaneDetectionWindow::~PlaneDetectionWindow() {}

void PlaneDetectionWindow::set_center_point(const Eigen::Vector3f& point) {
  region_growing_result = nullptr;
  plane_detection_result = nullptr;
  center_point = point;
}

void PlaneDetectionWindow::show() { show_window = true; }

void PlaneDetectionWindow::close() {
  show_window = false;
  region_growing_result = nullptr;
  plane_detection_result = nullptr;
}

RegionGrowingResult::Ptr PlaneDetectionWindow::region_growing(guik::ProgressInterface& progress) {
  RegionGrowingResult::Ptr result(new RegionGrowingResult());
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_points(new pcl::PointCloud<pcl::PointXYZI>());

  progress.set_maximum(5);
  progress.set_text("accumulating points");
  progress.increment();
  for (const auto& keyframe : graph->keyframes) {
    std::vector<int> neighbor_indices = keyframe.second->neighbors(center_point, initial_neighbor_search_radius);
    if (neighbor_indices.size() < 10) {
      continue;
    }

    result->candidates.push_back(keyframe.second);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*keyframe.second->cloud, *cloud, keyframe.second->node->estimate().cast<float>());

    if (accumulated_points->empty()) {
      pcl::PointNormal pt;
      accumulated_points->push_back(cloud->at(neighbor_indices[0]));
    }

    for (int i = 0; i < cloud->size(); i++) {
      accumulated_points->push_back(cloud->at(i));
    }
  }

  if(accumulated_points->size() < 50) {
    std::cerr << "too few points for region growing" << std::endl;
    return nullptr;
  }

  progress.set_text("normal estimation");
  progress.increment();
  pcl::PointCloud<pcl::Normal>::Ptr accumulated_normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  ne.setInputCloud(accumulated_points);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(normal_estimation_radius);

  ne.compute(*accumulated_normals);

  progress.set_text("region growing");
  progress.increment();
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

  progress.set_text("extract indices");
  progress.increment();
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

  progress.set_text("done");
  progress.increment();

  // result->plane_cloud = accumulated_points;
  return result;
}

PlaneDetectionResult::Ptr PlaneDetectionWindow::detect_plane(const RegionGrowingResult::Ptr& rg_result) {
  pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(rg_result->cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
  ransac.setDistanceThreshold(ransac_distance_thresh);
  ransac.computeModel();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  ransac.getInliers(inliers->indices);

  PlaneDetectionResult::Ptr result(new PlaneDetectionResult());
  ransac.getModelCoefficients(result->coeffs);

  for (const auto& candidate : rg_result->candidates) {
    Eigen::Vector4f coeffs = result->coeffs;
    Eigen::Vector4f local_coeffs;

    Eigen::Isometry3f trans = candidate->node->estimate().inverse().cast<float>();
    local_coeffs.head<3>() = trans.linear() * coeffs.head<3>();
    local_coeffs[3] = coeffs[3] - trans.translation().dot(local_coeffs.head<3>());

    auto inliers = detect_plane_with_coeffs(candidate->cloud, local_coeffs);
    if (inliers == nullptr || inliers->size() < min_plane_supports) {
      continue;
    }

    result->candidates.push_back(candidate);
    result->candidate_inliers.push_back(inliers);
    result->candidate_inlier_buffers.push_back(std::make_shared<glk::PointCloudBuffer>(inliers));
    result->candidate_local_coeffs.push_back(local_coeffs);
  }

  return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PlaneDetectionWindow::detect_plane_with_coeffs(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, Eigen::Vector4f& coeffs) {
  pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(cloud));
  pcl::PointIndices::Ptr init_indices(new pcl::PointIndices);
  model_p->selectWithinDistance(coeffs, ransac_distance_thresh, init_indices->indices);

  if (init_indices->indices.size() < 10) {
    return nullptr;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr init_inliers(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(init_indices);
  extract.setNegative(false);
  extract.filter(*init_inliers);

  model_p.reset(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(init_inliers));
  pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
  ransac.setDistanceThreshold(ransac_distance_thresh);
  ransac.computeModel();

  Eigen::VectorXf coeffs_;
  ransac.getModelCoefficients(coeffs_);

  if (coeffs_.head<3>().dot(coeffs.head<3>()) < 0.0f) {
    coeffs_ *= -1.0f;
  }
  coeffs = coeffs_;

  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  ransac.getInliers(indices->indices);

  pcl::PointCloud<pcl::PointXYZI>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZI>());
  extract.setInputCloud(init_inliers);
  extract.setIndices(indices);
  extract.filter(*inliers);

  return inliers;
}

void PlaneDetectionWindow::draw_ui() {
  if (!show_window) {
    region_growing_result.reset();
    return;
  }

  ImGui::Begin("plane detection", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::Text("Region growing");
  ImGui::DragFloat("Initial neighbor radius", &initial_neighbor_search_radius, 0.1f, 0.1f, 30.0f);
  ImGui::DragFloat("Normal estimation radius", &normal_estimation_radius, 0.1f, 0.1f, 5.0f);
  ImGui::DragInt("Min cluster size", &min_cluster_size, 1, 10, 10000);
  ImGui::DragInt("Max cluster size", &max_cluster_size, 100, 100, 1000000);
  ImGui::DragInt("Num neighbors", &num_neighbors, 1, 1, 1000);
  ImGui::DragFloat("Smoothness threshold[deg]", &smoothness_threshold, 0.1f, 0.1f, 100.0f);
  ImGui::DragFloat("Curvature threshold", &curvature_threshold, 0.1f, 0.1f, 100.0f);

  if (ImGui::Button("Perform")) {
    region_growing_result = nullptr;
    plane_detection_result = nullptr;
    region_growing_progress_modal.open<RegionGrowingResult::Ptr>("region growing", [this](guik::ProgressInterface& progress) { return region_growing(progress); });
  }
  if (region_growing_progress_modal.run("region growing")) {
    region_growing_result = region_growing_progress_modal.result<RegionGrowingResult::Ptr>();
    if(region_growing_result) {
      region_growing_result->cloud_buffer = std::make_shared<glk::PointCloudBuffer>(region_growing_result->cloud);
    }
  }

  if (region_growing_result) {
    ImGui::SameLine();
    ImGui::Text("points:%d", region_growing_result->cloud->size());
  }

  ImGui::Text("RANSAC plane");
  ImGui::DragFloat("Distance thresh", &ransac_distance_thresh, 0.01f, 0.01f, 10.0f);
  ImGui::DragInt("Min plane supports", &min_plane_supports, 10, 10, 10000);

  if (ImGui::Button("Detect")) {
    if (region_growing_result) {
      plane_detection_result = nullptr;
      plane_detection_result = detect_plane(region_growing_result);
    }
  }

  if (plane_detection_result) {
    ImGui::SameLine();
    Eigen::VectorXf coeffs = plane_detection_result->coeffs;
    ImGui::Text("coeffs:%.3f %.3f %.3f %.3f", coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
  }

  ImGui::Text("Robust kernel");
  const char* kernels[] = {"NONE", "Huber"};
  ImGui::Combo("Kernel type", &robust_kernel, kernels, IM_ARRAYSIZE(kernels));
  ImGui::DragFloat("Kernel delta", &robust_kernel_delta, 0.01f, 0.01f, 10.0f);

  if (ImGui::Button("Add edge")) {
    if (plane_detection_result) {
      auto plane_vertex = graph->add_plane(plane_detection_result->coeffs.cast<double>());

      for (int i = 0; i < plane_detection_result->candidates.size(); i++) {
        const auto& candidate = plane_detection_result->candidates[i];
        const Eigen::Vector4f& coeffs = plane_detection_result->candidate_local_coeffs[i];

        Eigen::Matrix3d information = Eigen::Matrix3d::Identity();
        graph->add_edge(candidate, plane_vertex, coeffs.cast<double>(), information, kernels[robust_kernel], robust_kernel_delta);
      }

      graph->optimize();
      show_window = false;
    }
  }

  ImGui::End();
}

void PlaneDetectionWindow::draw_gl(glk::GLSLShader& shader) {
  if (!show_window) {
    region_growing_result = nullptr;
    plane_detection_result = nullptr;
    return;
  }

  shader.set_uniform("color_mode", 1);
  shader.set_uniform("point_scale", 4.0f);

  if (plane_detection_result) {
    shader.set_uniform("material_color", Eigen::Vector4f(0.0f, 0.5f, 1.0f, 1.0f));
    shader.set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
    // plane_detection_result->cloud_buffer->draw(shader);

    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    for (int i = 0; i < plane_detection_result->candidates.size(); i++) {
      const auto& candidate = plane_detection_result->candidates[i];
      Eigen::Matrix4f model_matrix = candidate->node->estimate().cast<float>().matrix();
      model_matrix.block<3, 3>(0, 0) *= 0.5f;

      shader.set_uniform("material_color", Eigen::Vector4f(glk::colormap_categoricalf(glk::TURBO, i, 16)));
      shader.set_uniform("model_matrix", model_matrix);
      sphere.draw(shader);

      shader.set_uniform("model_matrix", candidate->node->estimate().cast<float>().matrix());
      plane_detection_result->candidate_inlier_buffers[i]->draw(shader);
    }
    return;
  }

  if (region_growing_result) {
    shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f));
    shader.set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());
    region_growing_result->cloud_buffer->draw(shader);

    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    for (const auto& candidate : region_growing_result->candidates) {
      Eigen::Matrix4f model_matrix = candidate->node->estimate().cast<float>().matrix();
      model_matrix.block<3, 3>(0, 0) *= 0.5f;
      shader.set_uniform("model_matrix", model_matrix);
      sphere.draw(shader);
    }
  }
}

}  // namespace hdl_graph_slam