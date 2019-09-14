#ifndef HDL_GRAPH_SLAM_DRAWABLE_HPP
#define HDL_GRAPH_SLAM_DRAWABLE_HPP

#include <memory>
#include <glk/glsl_shader.hpp>

namespace hdl_graph_slam {

struct DrawFlags {
public:
  DrawFlags() {
    draw_verticies = true;
    draw_edges = true;

    draw_keyframe_vertices = true;
    draw_plane_vertices = true;

    draw_se3_edges = true;
    draw_se3_plane_edges = true;
    draw_floor_edges = false;
  }

  bool draw_verticies;
  bool draw_edges;

  bool draw_keyframe_vertices;
  bool draw_plane_vertices;

  bool draw_se3_edges;
  bool draw_se3_plane_edges;
  bool draw_floor_edges;
};

class DrawableObject {
public:
  enum OBJECT_TYPE {
    POINTS = 1,
    KEYFRAME,
    VERTEX,
    EDGE,
  };

  using Ptr = std::shared_ptr<DrawableObject>;

  DrawableObject() {}
  virtual ~DrawableObject() {}

  virtual bool available() const { return true; }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) {}

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix) {}
};

}  // namespace hdl_graph_slam

#endif