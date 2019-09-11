#ifndef HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_VIEW_HPP
#define HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_VIEW_HPP

#include <unordered_map>
#include <glk/glsl_shader.hpp>

#include <hdl_graph_slam/interactive_graph.hpp>

#include <hdl_graph_slam/view/edge_view.hpp>
#include <hdl_graph_slam/view/keyframe_view.hpp>
#include <hdl_graph_slam/view/drawable_object.hpp>

namespace hdl_graph_slam {

class InteractiveGraphView : public InteractiveGraph {
public:
  InteractiveGraphView() {}
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
      auto edge_view = EdgeView::create(edge);
      if(edge_view) {
        edges_view.push_back(edge_view);

        drawables.push_back(edge_view);
      }
    }
  }

  void draw(glk::GLSLShader& shader) {
    update_view();

    for(auto& drawable : drawables) {
      if(drawable->available()) {
        drawable->draw(shader);
      }
    }
  }

public:
  std::vector<KeyFrameView::Ptr> keyframes_view;
  std::unordered_map<KeyFrame::Ptr, KeyFrameView::Ptr> keyframes_view_map;

  std::vector<EdgeView::Ptr> edges_view;

  std::vector<DrawableObject::Ptr> drawables;
};

}  // namespace hdl_graph_slam

#endif