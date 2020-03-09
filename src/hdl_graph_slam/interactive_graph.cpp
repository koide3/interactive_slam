#include <hdl_graph_slam/interactive_graph.hpp>

#include <chrono>
#include <boost/filesystem.hpp>

#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_parallel.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>

namespace hdl_graph_slam {

InteractiveGraph::InteractiveGraph() : GraphSLAM("lm_var_cholmod"), iterations(0), chi2_before(0.0), chi2_after(0.0), elapsed_time_msec(0.0) {
  inf_calclator.reset(new InformationMatrixCalculator());
  inf_calclator->load(params);
  edge_id_gen = 0;

  anchor_node = nullptr;
  anchor_edge = nullptr;
  floor_node = nullptr;
}

InteractiveGraph::~InteractiveGraph() {
  if(optimization_thread.joinable()) {
    optimization_thread.join();
  }
}

bool InteractiveGraph::load_map_data(const std::string& directory, guik::ProgressInterface& progress) {
  // load graph file
  progress.set_title("Opening " + directory);
  progress.set_text("loading graph");
  if(!load(directory + "/graph.g2o")) {
    return false;
  }

  // re-assign edge ids
  // note: newly created edges may have inconsistent IDs (who cares!)
  edge_id_gen = 0;
  for(auto& edge : graph->edges()) {
    edge->setId(edge_id_gen++);
  }

  // load keyframes
  progress.increment();
  progress.set_text("loading keyframes");
  if(!load_keyframes(directory, progress)) {
    return false;
  }

  // load anchor and floor nodes
  if(!load_special_nodes(directory, progress)) {
    return false;
  }

  return true;
}

bool InteractiveGraph::merge_map_data(InteractiveGraph& graph_, const InteractiveKeyFrame::Ptr& key1, const InteractiveKeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose) {
  long max_vertex_id = 0;
  long max_edge_id = 0;

  for(const auto& vertex : graph->vertices()) {
    max_vertex_id = std::max<long>(max_vertex_id, vertex.first);
  }
  for(const auto& edge : graph->edges()) {
    max_edge_id = std::max<long>(max_edge_id, edge->id());
  }

  g2o::Factory* factory = g2o::Factory::instance();
  std::unordered_map<long, g2o::HyperGraph::Vertex*> new_vertices_map;    // old vertex Id -> new vertex instance map

  // remove the anchor node in the graph to be merged
  if(graph_.anchor_node) {
    graph_.graph->removeEdge(graph_.anchor_edge);
    graph_.graph->detachVertex(graph_.anchor_node);
  }

  // clone vertices
  for(const auto& vertex : graph_.graph->vertices()) {
    long new_vertex_id = ++max_vertex_id;
    auto v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex.second);

    // copy params via g2o::Factory
    std::stringstream sst;
    if(!v->write(sst)) {
      std::cerr << "error: failed to write vertex data" << std::endl;
      return false;
    }

    auto new_v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(factory->construct(factory->tag(v)));
    if(!new_v->read(sst)) {
      std::cerr << "error: failed to read vertex data" << std::endl;
      return false;
    }
    new_v->setFixed(v->fixed());
    new_v->setId(new_vertex_id);
    graph->addVertex(new_v);

    // for remapping
    new_vertices_map[v->id()] = new_v;
  }

  // clone edges
  for(const auto& edge : graph_.graph->edges()) {
    long new_edge_id = ++max_edge_id;
    auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);

    // copy params via g2o::Factory
    std::stringstream sst;
    if(!e->write(sst)) {
      std::cerr << "error: failed to write edge data" << std::endl;
      return false;
    }

    auto new_e = dynamic_cast<g2o::OptimizableGraph::Edge*>(factory->construct(factory->tag(e)));
    if(!new_e->read(sst)) {
      std::cerr << "error: failed to read edge data" << std::endl;
      return false;
    }
    new_e->setId(new_edge_id);

    // remap vertices with new ones
    for(int i = 0; i < new_e->vertices().size(); i++) {
      new_e->vertices()[i] = new_vertices_map[e->vertices()[i]->id()];
    }

    // copy robust kernel
    if(e->robustKernel()) {
      g2o::RobustKernel* kernel = nullptr;

      if(dynamic_cast<g2o::RobustKernelHuber*>(e->robustKernel())) {
        kernel = new g2o::RobustKernelHuber();
      }

      if(kernel == nullptr) {
        std::cerr << "warning: unknown kernel type!!" << std::endl;
      } else {
        kernel->setDelta(e->robustKernel()->delta());
        new_e->setRobustKernel(kernel);
      }
    }

    edge->setId(new_edge_id);
    graph->addEdge(new_e);
  }

  // copy keyframes
  for(const auto& keyframe : graph_.keyframes) {
    keyframe.second->node = dynamic_cast<g2o::VertexSE3*>(new_vertices_map[keyframe.second->id()]);
    std::cout << "keyframe_id:" << keyframe.second->node->id() << std::endl;
    assert(keyframe.second->node);
    keyframes[keyframe.second->id()] = keyframe.second;
  }

  return true;
}

bool InteractiveGraph::load_special_nodes(const std::string& directory, guik::ProgressInterface& progress) {
  // load special nodes from file
  std::ifstream ifs(directory + "/special_nodes.csv");
  if(ifs) {
    while(!ifs.eof()) {
      std::string line;
      std::getline(ifs, line);

      if(line.empty()) {
        continue;
      }

      std::stringstream sst(line);
      std::string tag;
      sst >> tag;

      // load anchor node
      if(tag == "anchor_node") {
        long anchor_node_id = -1;
        sst >> anchor_node_id;

        if(anchor_node_id < 0) {
          continue;
        }

        anchor_node = dynamic_cast<g2o::VertexSE3*>(graph->vertex(anchor_node_id));
        if(anchor_node == nullptr) {
          std::cerr << "failed to cast anchor node to VertexSE3!!" << std::endl;
          return false;
        }
        if(anchor_node->edges().empty()) {
          std::cerr << "anchor node is not connected with any edges!!" << std::endl;
          return false;
        }

        anchor_edge = dynamic_cast<g2o::EdgeSE3*>(*anchor_node->edges().begin());
        if(anchor_edge == nullptr) {
          std::cerr << "failed to cast anchor edge to EdgeSE3!!" << std::endl;
          return false;
        }

      }
      // load floor node
      else if(tag == "floor_node") {
        long floor_node_id = -1;
        sst >> floor_node_id;

        if(floor_node_id < 0) {
          continue;
        }

        floor_node = dynamic_cast<g2o::VertexPlane*>(graph->vertex(floor_node_id));
        if(floor_node == nullptr) {
          std::cerr << "failed to cast floor node to VertexPlane!!" << std::endl;
          return false;
        }
      }
    }
  }

  // create anchor node if it is not loaded yet
  if(anchor_node == nullptr) {
    std::cout << "create new anchor" << std::endl;
    using ID_Keyframe = std::pair<long, InteractiveKeyFrame::Ptr>;
    auto first_keyframe = std::min_element(keyframes.begin(), keyframes.end(), [=](const ID_Keyframe& lhs, const ID_Keyframe& rhs) { return lhs.first < rhs.first; });

    if(first_keyframe == keyframes.end()) {
      std::cerr << "corrupted graph file!!" << std::endl;
      return false;
    }

    anchor_node = add_se3_node(first_keyframe->second->node->estimate());
    anchor_edge = add_se3_edge(anchor_node, first_keyframe->second->node, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 6) * 0.1);
  }

  std::cout << "anchor_node:" << anchor_node->id() << std::endl;
  std::cout << "anchor_edge:" << anchor_edge->vertices()[0]->id() << " - " << anchor_edge->vertices()[1]->id() << std::endl;

  anchor_node->setFixed(true);

  return true;
}

bool InteractiveGraph::load_keyframes(const std::string& directory, guik::ProgressInterface& progress) {
  progress.set_maximum(graph->vertices().size());
  for(int i = 0;; i++) {
    std::string keyframe_dir = (boost::format("%s/%06d") % directory % i).str();
    if(!boost::filesystem::is_directory(keyframe_dir)) {
      break;
    }

    InteractiveKeyFrame::Ptr keyframe = std::make_shared<InteractiveKeyFrame>(keyframe_dir, graph.get());
    if(!keyframe->node) {
      std::cerr << "error : failed to load keyframe!!" << std::endl;
      std::cerr << "      : " << keyframe_dir << std::endl;
    } else {
      keyframes[keyframe->id()] = keyframe;
      progress.increment();
    }
  }

  return true;
}

long InteractiveGraph::anchor_node_id() const {
  return anchor_node ? anchor_node->id() : -1;
}

g2o::EdgeSE3* InteractiveGraph::add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose, const std::string& robust_kernel, double robust_kernel_delta) {
  Eigen::MatrixXd inf = inf_calclator->calc_information_matrix(key1->cloud, key2->cloud, relative_pose);
  g2o::EdgeSE3* edge = add_se3_edge(key1->node, key2->node, relative_pose, inf);
  edge->setId(edge_id_gen++);

  if(robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }

  return edge;
}

g2o::VertexPlane* InteractiveGraph::add_plane(const Eigen::Vector4d& coeffs) {
  return add_plane_node(coeffs);
}

g2o::EdgeSE3Plane* InteractiveGraph::add_edge(const KeyFrame::Ptr& v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& coeffs, const Eigen::MatrixXd& information, const std::string& robust_kernel, double robust_kernel_delta) {
  g2o::EdgeSE3Plane* edge = add_se3_plane_edge(v_se3->node, v_plane, coeffs, information);
  edge->setId(edge_id_gen++);

  if(robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }

  return edge;
}

void InteractiveGraph::apply_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& robust_kernel, double robust_kernel_delta) {
  add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
}

void InteractiveGraph::add_edge_identity(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale, const std::string& robust_kernel, double robust_kernel_delta) {
  auto edge = add_plane_identity_edge(v1, v2, Eigen::Vector4d::Zero(), Eigen::Matrix4d::Identity() * information_scale);
  if(robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }
  edge->setId(edge_id_gen++);
}

void InteractiveGraph::add_edge_parallel(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale, const std::string& robust_kernel, double robust_kernel_delta) {
  auto edge = add_plane_parallel_edge(v1, v2, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * information_scale);
  if(robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }
  edge->setId(edge_id_gen++);
}

void InteractiveGraph::add_edge_perpendicular(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale, const std::string& robust_kernel, double robust_kernel_delta) {
  auto edge = add_plane_perpendicular_edge(v1, v2, Eigen::Vector3d::Zero(), Eigen::MatrixXd::Identity(1, 1) * information_scale);
  if(robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }
  edge->setId(edge_id_gen++);
}

bool InteractiveGraph::add_edge_prior_normal(long plane_vertex_id, const Eigen::Vector3d& normal, double information_scale, const std::string& robust_kernel, double robust_kernel_delta) {
  auto vertex = graph->vertex(plane_vertex_id);
  if(vertex == nullptr) {
    return false;
  }

  g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(vertex);
  if(vertex_plane == nullptr) {
    return false;
  }

  Eigen::Matrix3d inf = Eigen::Matrix3d::Identity() * information_scale;
  auto edge = this->add_plane_normal_prior_edge(vertex_plane, normal, inf);
  if(robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }

  return true;
}

bool InteractiveGraph::add_edge_prior_distance(long plane_vertex_id, double distance, double information_scale, const std::string& robust_kernel, double robust_kernel_delta) {
  auto vertex = graph->vertex(plane_vertex_id);
  if(vertex == nullptr) {
    return false;
  }

  g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(vertex);
  if(vertex_plane == nullptr) {
    return false;
  }

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(1, 1) * information_scale;
  auto edge = this->add_plane_distance_prior_edge(vertex_plane, distance, inf);
  if(robust_kernel != "NONE") {
    add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
  }

  return true;
}

void InteractiveGraph::optimize(int num_iterations) {
  if(anchor_node) {
    // move the anchor node to the position of the very first keyframe
    // so that the keyframe can move freely while trying to stay around the origin
    g2o::VertexSE3* first_keyframe = dynamic_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1]);
    if(first_keyframe == nullptr) {
      std::cerr << "failed to cast the node which is likely to be the fist keyframe to VertexSE3";
    } else {
      anchor_node->setEstimate(first_keyframe->estimate());
    }
  }

  // !! bad tech !!
  // override std::cerr with optimization_stream to catch optimization progress messages
  optimization_stream.str("");
  optimization_stream.clear();

  std::streambuf* cerr_buf = std::cerr.rdbuf();
  std::cerr.rdbuf(optimization_stream.rdbuf());

  g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
  auto t1 = std::chrono::high_resolution_clock::now();

  if(num_iterations < 0) {
    num_iterations = params.param<int>("g2o_solver_num_iterations", 64);
  }

  chi2_before = graph->chi2();
  iterations = GraphSLAM::optimize(num_iterations);
  chi2_after = graph->chi2();

  auto t2 = std::chrono::high_resolution_clock::now();
  elapsed_time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000.0;

  std::cerr.rdbuf(cerr_buf);
}

void InteractiveGraph::optimize_background(int num_iterations) {
  if(optimization_thread.joinable()) {
    optimization_thread.join();
  }

  if(anchor_node) {
    // move the anchor node to the position of the very first keyframe
    // so that the keyframe can move freely while trying to stay around the origin
    g2o::VertexSE3* first_keyframe = dynamic_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1]);
    if(first_keyframe == nullptr) {
      std::cerr << "failed to cast the node which is likely to be the fist keyframe to VertexSE3";
    } else {
      anchor_node->setEstimate(first_keyframe->estimate());
    }
  }

  // !! bad tech !!
  // override std::cerr with optimization_stream to catch optimization progress messages
  optimization_stream.str("");
  optimization_stream.clear();

  auto task = [this, num_iterations]() {
    std::streambuf* cerr_buf = std::cerr.rdbuf();
    std::cerr.rdbuf(optimization_stream.rdbuf());

    std::lock_guard<std::mutex> lock(optimization_mutex);
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    auto t1 = std::chrono::high_resolution_clock::now();

    int max_iterations = num_iterations;
    if(num_iterations < 0) {
      max_iterations = params.param<int>("g2o_solver_num_iterations", 64);
    }

    chi2_before = graph->chi2();
    iterations = GraphSLAM::optimize(max_iterations);
    chi2_after = graph->chi2();

    auto t2 = std::chrono::high_resolution_clock::now();
    elapsed_time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000.0;

    std::cerr.rdbuf(cerr_buf);
  };

  optimization_thread = std::thread(task);
}

std::string InteractiveGraph::graph_statistics(bool update) {
  if(optimization_mutex.try_lock()) {
    std::stringstream sst;
    sst << "Graph\n";
    sst << boost::format("# vertices: %d") % num_vertices() << "\n";
    sst << boost::format("# edges: %d") % num_edges() << "\n";
    sst << boost::format("time: %.1f[msec]") % elapsed_time_msec << "\n";
    sst << boost::format("chi2: %.3f -> %.3f") % chi2_before % chi2_after << "\n";
    sst << boost::format("iterations: %d") % iterations;

    graph_stats = sst.str();
    optimization_mutex.unlock();
  }

  return graph_stats;
}

std::string InteractiveGraph::optimization_messages() const {
  return optimization_stream.str();
}

void InteractiveGraph::dump(const std::string& directory, guik::ProgressInterface& progress) {
  progress.set_maximum(keyframes.size());
  progress.set_text("saving graph");
  progress.increment();

  save(directory + "/graph.g2o");

  progress.set_text("saving keyframes");

  int keyframe_id = 0;
  for(const auto& keyframe : keyframes) {
    progress.increment();

    std::stringstream sst;
    sst << boost::format("%s/%06d") % directory % (keyframe_id++);
    keyframe.second->save(sst.str());
  }

  std::ofstream ofs(directory + "/special_nodes.csv");
  ofs << "anchor_node " << (anchor_node != nullptr ? anchor_node->id() : -1) << std::endl;
  ofs << "anchor_edge " << -1 << std::endl;
  ofs << "floor_node " << (floor_node != nullptr ? floor_node->id() : -1) << std::endl;
}

bool InteractiveGraph::save_pointcloud(const std::string& filename, guik::ProgressInterface& progress) {
  progress.set_maximum(keyframes.size() + 1);
  progress.set_text("accumulate points");

  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());
  for(const auto& keyframe : keyframes) {
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