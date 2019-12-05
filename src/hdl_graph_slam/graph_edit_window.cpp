#include <hdl_graph_slam/graph_edit_window.hpp>

#include <sstream>
#include <g2o/core/optimizable_graph.h>

namespace hdl_graph_slam {

GraphEditWindow::GraphEditWindow(std::shared_ptr<InteractiveGraphView>& graph) : show_window(false), selected_vertex(0), graph(graph) {}
GraphEditWindow::~GraphEditWindow() {}

void GraphEditWindow::draw_ui() {
  if(!show_window) {
    return;
  }

  ImGui::Begin("graph edit", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

  ImGuiTabBarFlags flags = ImGuiTabBarFlags_None;
  if(ImGui::BeginTabBar("tabbar", flags)) {
    if(ImGui::BeginTabItem("Fixed vertices")) {
      g2o::VertexSE3* anchor_node = dynamic_cast<g2o::VertexSE3*>(graph->graph->vertex(graph->anchor_node_id()));
      if(anchor_node) {
        bool fixed = anchor_node->fixed();
        if(ImGui::Checkbox("Anchor Fixed", &fixed)) {
          anchor_node->setFixed(fixed);
        }
      } else {
        ImGui::Text("warning: No anchor node created!!");
      }

      bool fixed_vertex_exists = false;
      for(const auto& vertex : graph->graph->vertices()) {
        auto v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vertex.second);
        assert(v != nullptr);

        if(!v->fixed() || v->id() == graph->anchor_node_id()) {
          continue;
        }

        std::stringstream sst;
        sst << "Vertex " << v->id();

        if(ImGui::Button(("Unfix##" + std::to_string(v->id())).c_str())) {
          v->setFixed(false);
        }
        ImGui::SameLine();
        ImGui::Text("Vertex %d", v->id());
        fixed_vertex_exists = true;
      }

      if(!fixed_vertex_exists) {
        ImGui::Text("No fixed vertices!!");
      }

      ImGui::EndTabItem();
    }

    if(ImGui::BeginTabItem("Some fancy function")) {
      for(const auto& edge : graph->graph->edges()) {
        auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
        assert(e != nullptr);
      }

      ImGui::Text("hello");
      ImGui::EndTabItem();
    }

    ImGui::EndTabBar();
  }

  ImGui::End();
}

}  // namespace hdl_graph_slam