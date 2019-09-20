#include <hdl_graph_slam/interactive_graph.hpp>

#include <chrono>
#include <boost/filesystem.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_plane_parallel.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>

namespace hdl_graph_slam {

InteractiveGraph::InteractiveGraph() : GraphSLAM("lm_var"), iterations(0), chi2_before(0.0), chi2_after(0.0), elapsed_time_msec(0.0) {
  inf_calclator.reset(new InformationMatrixCalculator());
  inf_calclator->load(params);
  edge_id_gen = 0;
}

InteractiveGraph::~InteractiveGraph() {}

bool InteractiveGraph::load_map_data(const std::string& directory, guik::ProgressInterface& progress) {
  progress.set_title("Opening " + directory);
  progress.set_text("loading graph");
  if (!load(directory + "/graph.g2o")) {
    return false;
  }

  edge_id_gen = 0;
  for (auto& edge : graph->edges()) {
    edge->setId(edge_id_gen++);
  }

  progress.increment();
  progress.set_text("loading keyframes");
  if (!load_keyframes(directory, progress)) {
    return false;
  }

  return true;
}

bool InteractiveGraph::merge_map_data(InteractiveGraph& graph_, const InteractiveKeyFrame::Ptr& key1, const InteractiveKeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose) {
  long max_vertex_id = 0;
  long max_edge_id = 0;

  for(const auto& vertex: graph->vertices()) {
    max_vertex_id = std::max<long>(max_vertex_id, vertex.first);
  }
  for(const auto& edge: graph->edges()) {
    max_edge_id = std::max<long>(max_edge_id, edge->id());
  }

  std::unordered_map<long, g2o::HyperGraph::Vertex*> new_vertices_map;

  int n = 0;
  for(const auto& vertex : graph_.graph->vertices()) {
    long new_vertex_id = ++max_vertex_id;

    auto ele = vertex.second->clone();
    std::cout << int(ele != nullptr) << std::endl;
    // auto cloned = dynamic_cast<g2o::HyperGraph::Vertex*>(ele);
    auto cloned = vertex.second;
    new_vertices_map[vertex.first] = cloned;
    cloned->setId(new_vertex_id);
    graph->addVertex(cloned);
  }

  for(const auto& edge : graph_.graph->edges()) {
    long new_edge_id = ++max_edge_id;
    // g2o::HyperGraph::Edge* cloned = dynamic_cast<g2o::HyperGraph::Edge*>(edge->clone());
    edge->setId(new_edge_id);
    graph->addEdge(edge);
  }

  for(const auto& keyframe : graph_.keyframes) {
    keyframe.second->node = dynamic_cast<g2o::VertexSE3*>(new_vertices_map[keyframe.second->id()]);
    assert(keyframe.second->node);
    keyframes[keyframe.second->id()] = keyframe.second;
  }

  std::cout << "edge between " << key1->id() << " and " << key2->id() << std::endl;
  add_se3_edge(key1->node, key2->node, relative_pose, Eigen::MatrixXd::Identity(6, 6));

  return true;
}

bool InteractiveGraph::load_keyframes(const std::string& directory, guik::ProgressInterface& progress) {
  progress.set_maximum(graph->vertices().size());
  for (int i = 0;; i++) {
    std::string keyframe_dir = (boost::format("%s/%06d") % directory % i).str();
    if (!boost::filesystem::is_directory(keyframe_dir)) {
      break;
    }

    InteractiveKeyFrame::Ptr keyframe = std::make_shared<InteractiveKeyFrame>(keyframe_dir, graph.get());
    if (!keyframe->node) {
      std::cerr << "error : failed to load keyframe!!" << std::endl;
      std::cerr << "      : " << keyframe_dir << std::endl;
      return false;
    }

    keyframes[keyframe->id()] = keyframe;
    progress.increment();
  }

  return true;
}

g2o::EdgeSE3* InteractiveGraph::add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose, const std::string& robust_kernel, double robust_kernel_delta) {
  Eigen::MatrixXd inf = inf_calclator->calc_information_matrix(key1->cloud, key2->cloud, relative_pose);
  g2o::EdgeSE3* edge = add_se3_edge(key1->node, key2->node, relative_pose, inf);
  edge->setId(edge_id_gen++);

  if (robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }

  return edge;
}

g2o::VertexPlane* InteractiveGraph::add_plane(const Eigen::Vector4d& coeffs) { return add_plane_node(coeffs); }
g2o::EdgeSE3Plane* InteractiveGraph::add_edge(const KeyFrame::Ptr& v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& coeffs, const Eigen::MatrixXd& information, const std::string& robust_kernel, double robust_kernel_delta) {
  g2o::EdgeSE3Plane* edge = add_se3_plane_edge(v_se3->node, v_plane, coeffs, information);
  edge->setId(edge_id_gen++);

  if (robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }

  return edge;
}

void InteractiveGraph::add_edge_parallel(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale) {
  auto edge = add_plane_parallel_edge(v1, v2, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * information_scale);
  edge->setId(edge_id_gen++);
}

void InteractiveGraph::add_edge_perpendicular(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale) {
  auto edge = add_plane_perpendicular_edge(v1, v2, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * information_scale);
  edge->setId(edge_id_gen++);
}

void InteractiveGraph::optimize() {
  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
  auto t1 = std::chrono::high_resolution_clock::now();

  chi2_before = graph->chi2();
  iterations = GraphSLAM::optimize(params.param<int>("g2o_solver_num_iterations", 64));
  chi2_after = graph->chi2();

  auto t2 = std::chrono::high_resolution_clock::now();
  elapsed_time_msec = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
}

void InteractiveGraph::dump(const std::string& directory, guik::ProgressInterface& progress) {
  progress.set_maximum(keyframes.size());
  progress.set_text("saving graph");
  progress.increment();

  save(directory + "/graph.g2o");

  progress.set_text("saving keyframes");

  int keyframe_id = 0;
  for (const auto& keyframe : keyframes) {
    progress.increment();

    std::stringstream sst;
    sst << boost::format("%s/%06d") % directory % (keyframe_id++);
    keyframe.second->save(sst.str());
  }
}

bool InteractiveGraph::save_pointcloud(const std::string& filename, guik::ProgressInterface& progress) {
  progress.set_maximum(keyframes.size() + 1);
  progress.set_text("accumulate points");

  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto& keyframe : keyframes) {
    progress.increment();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*keyframe.second->cloud, *transformed, keyframe.second->node->estimate().cast<float>());

    std::copy(transformed->begin(), transformed->end(), std::back_inserter(accumulated->points));
  }

  accumulated->is_dense = false;
  accumulated->width = accumulated->size();
  accumulated->height = 1;

  progress.set_text("saving pcd");
  progress.increment();

  return pcl::io::savePCDFileBinary(filename, *accumulated);
}
}  // namespace hdl_graph_slam