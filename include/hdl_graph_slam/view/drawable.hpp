#ifndef HDL_GRAPH_SLAM_DRAWABLE_HPP
#define HDL_GRAPH_SLAM_DRAWABLE_HPP

#include <memory>
#include <glk/glsl_shader.hpp>

namespace hdl_graph_slam {

enum OBJECT_TYPE {
  POINTS = 0,
  VERTEX,
  EDGE,
};

class Drawable {
public:
  using Ptr = std::shared_ptr<Drawable>;

  virtual bool available() const = 0;

  virtual void draw(glk::GLSLShader& shader) = 0;
};

}

#endif