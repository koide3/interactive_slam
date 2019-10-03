#include <hdl_graph_slam/edge_refinement_window.hpp>

#include <functional>
#include <g2o/types/slam3d/edge_se3.h>

#include <pclomp/ndt_omp.h>
#include <hdl_graph_slam/information_matrix_calculator.hpp>

namespace hdl_graph_slam {

EdgeInfo::EdgeInfo(g2o::EdgeSE3* edge) : edge(edge), begin(edge->vertices()[0]->id()), end(edge->vertices()[1]->id()), num_evaluations(0) {}

bool EdgeInfo::operator < (const EdgeInfo& rhs) const {
  return score() < rhs.score();
}

bool EdgeInfo::operator > (const EdgeInfo& rhs) const {
  return score() > rhs.score();
}

double EdgeInfo::error() const {
  return (edge->information() * edge->error()).array().abs().sum();
}

double EdgeInfo::score() const {
  return error() / (num_evaluations + 1);
}

EdgeRefinementWindow::EdgeRefinementWindow(std::shared_ptr<InteractiveGraphView>& graph)
    : show_window(false), graph(graph), running(false), scan_matching_method(0), scan_matching_resolution(2.0f), robust_kernel(1), robust_kernel_delta(0.01f) {}

EdgeRefinementWindow::~EdgeRefinementWindow() {
  running = false;
  refinement_thread.join();
}

void EdgeRefinementWindow::draw_ui() {
  if(!show_window) {
    running = false;
    return;
  }

  ImGui::Begin("edge refinement", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::Text("Scan matching");
  const char* methods[] = {"NDT"};
  ImGui::Combo("Method", &scan_matching_method, methods, IM_ARRAYSIZE(methods));
  ImGui::DragFloat("Resolution", &scan_matching_resolution, 0.1f, 0.1f, 20.0f);

  ImGui::Text("Robust kernel");
  const char* kernels[] = {"NONE", "Huber"};
  ImGui::Combo("Kernel type", &robust_kernel, kernels, IM_ARRAYSIZE(kernels));
  ImGui::DragFloat("Kernel delta", &robust_kernel_delta, 0.01f, 0.01f, 10.0f);

  if(ImGui::Button("Apply to all SE3 edges")) {
    apply_robust_kernel();
  }

  if(ImGui::Button("Start")) {
    if(!running) {
      running = true;
      refinement_thread = std::thread([this]() { refinement_task(); });
    }
    }

  ImGui::SameLine();
  if(ImGui::Button("Stop")) {
    if(running) {
      running = false;
      refinement_thread.join();
    }
  }

  if(running) {
    ImGui::SameLine();
    ImGui::Text("%c Running", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]);
  }

  ImGui::End();

  ImGui::Begin("edges", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

  ImGui::BeginChild("edge pane", ImVec2(256, 256));
  ImGui::Columns(5, "edge_columns");
  ImGui::Separator();
  ImGui::Text("Begin");
  ImGui::NextColumn();
  ImGui::Text("End");
  ImGui::NextColumn();
  ImGui::Text("Score");
  ImGui::NextColumn();
  ImGui::Text("Error");
  ImGui::NextColumn();
  ImGui::Text("Evaluated");
  ImGui::NextColumn();
  ImGui::Separator();

  for(const auto& edge: edges) {
    ImGui::Text("%d", edge.begin);
    ImGui::NextColumn();
    ImGui::Text("%d", edge.end);
    ImGui::NextColumn();
    ImGui::Text("%.5f", edge.score());
    ImGui::NextColumn();
    ImGui::Text("%.5f", edge.error());
    ImGui::NextColumn();
    ImGui::Text("%d", edge.num_evaluations);
    ImGui::NextColumn();
  }

  ImGui::EndChild();
  ImGui::End();
}

void EdgeRefinementWindow::apply_robust_kernel() {
  if(running) {
    running = false;
    refinement_thread.join();
  }

  for(const auto& edge: edges) {
    const char* kernels[] = {"NONE", "Huber"};
    graph->apply_robust_kernel(edge.edge, kernels[robust_kernel], robust_kernel_delta);
  }
  graph->optimize();
}

void EdgeRefinementWindow::refinement() {
  {
    std::lock_guard<std::mutex> lock(edges_mutex);
    std::sort(edges.begin(), edges.end(), std::greater<EdgeInfo>());
    edges.front().num_evaluations++;
  }
  auto& edge = edges.front();

  auto v1 = graph->keyframes.find(edge.begin);
  auto v2 = graph->keyframes.find(edge.end);

  if(v1 == graph->keyframes.end() || v2 == graph->keyframes.end()) {
    return;
  }

  double fitness_score_before = InformationMatrixCalculator::calc_fitness_score(v1->second->cloud, v2->second->cloud, edge.edge->measurement(), 2.0f);

  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
  auto ndt = boost::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
  ndt->setResolution(scan_matching_resolution);
  registration = ndt;

  Eigen::Isometry3d relative = v1->second->estimate().inverse() * v2->second->estimate();
  double fitness_score_before2 = InformationMatrixCalculator::calc_fitness_score(v1->second->cloud, v2->second->cloud, relative, 2.0f);

  registration->setInputTarget(v1->second->cloud);
  registration->setInputSource(v2->second->cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
  registration->align(*aligned, relative.matrix().cast<float>());

  relative.matrix() = registration->getFinalTransformation().cast<double>();
  double fitness_score_after = InformationMatrixCalculator::calc_fitness_score(v1->second->cloud, v2->second->cloud, relative, 2.0f);

  std::cout << registration->getFitnessScore() << std::endl;
  std::cout << fitness_score_before << "->" << fitness_score_before2 << "->" << fitness_score_after << std::endl;

  if(fitness_score_after < fitness_score_before) {
    std::lock_guard<std::mutex> lock(graph->optimization_mutex);
    edge.edge->setMeasurement(relative);
    graph->optimize();
  }

  {
    std::lock_guard<std::mutex> lock(edges_mutex);
    std::sort(edges.begin(), edges.end());
  }
}

void EdgeRefinementWindow::refinement_task() {
  while(running) {
    refinement();
  }
}

    void
    EdgeRefinementWindow::draw_gl(glk::GLSLShader& shader) {
  if(!running) {
    return;
  }
}

void EdgeRefinementWindow::show() {
  show_window = true;

  graph->optimize();

  edges.clear();
  for(const auto& edge : graph->graph->edges()) {
    g2o::EdgeSE3* e = dynamic_cast<g2o::EdgeSE3*>(edge);
    if(e == nullptr) {
      continue;
    }

    edges.push_back(EdgeInfo(e));
  }

  std::sort(edges.begin(), edges.end(), std::greater<EdgeInfo>());
}

void EdgeRefinementWindow::close() {
  if(running) {
    running = false;
  }
  show_window = false;
  edges.clear();
}

}  // namespace hdl_graph_slam