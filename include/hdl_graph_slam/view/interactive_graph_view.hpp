#ifndef HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_VIEW_HPP
#define HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_VIEW_HPP

#include <unordered_map>
#include <glk/glsl_shader.hpp>

#include <hdl_graph_slam/interactive_graph.hpp>

#include <hdl_graph_slam/view/edge_view.hpp>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/line_buffer.hpp>
#include <hdl_graph_slam/view/drawable_object.hpp>

namespace hdl_graph_slam {

class InteractiveGraphView : public InteractiveGraph {
public:
  InteractiveGraphView() {
    line_buffer.reset(new LineBuffer());
  }
  virtual ~InteractiveGraphView() {}

  void update_view() {
    for(const auto& key_item : keyframes) {
      auto& keyframe = key_item.second;
      auto found = keyframes_view_map.find(keyframe);
      if(found == keyframes_view_map.end()) {
        keyframes_view.push_back(std::make_shared<KeyFrameView>(keyframe));
        keyframes_view_map[keyframe] = keyframes_view.back();

        drawables.push_back(keyframes_view.back());
      }
    }

    for(const auto& edge: graph->edges()) {
      auto found = edges_view_map.find(edge);
      if(found != edges_view_map.end()) {
        continue;
      }

      auto edge_view = EdgeView::create(edge, *line_buffer);
      if(edge_view) {
        edges_view.push_back(edge_view);
        edges_view_map[edge] = edge_view;

        drawables.push_back(edge_view);
      }
    }
  }

  void draw(glk::GLSLShader& shader) {
    update_view();
    line_buffer->clear();

    for(auto& drawable : drawables) {
      if(drawable->available()) {
        drawable->draw(shader);
      }
    }

    line_buffer->draw(shader);
  }

public:
  std::unique_ptr<LineBuffer> line_buffer;

  std::vector<KeyFrameView::Ptr> keyframes_view;
  std::unordered_map<KeyFrame::Ptr, KeyFrameView::Ptr> keyframes_view_map;

  std::vector<EdgeView::Ptr> edges_view;
  std::unordered_map<g2o::HyperGraph::Edge*, EdgeView::Ptr> edges_view_map;

  std::vector<DrawableObject::Ptr> drawables;
};

}  // namespace hdl_graph_slam

#endif