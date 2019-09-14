#ifndef HDL_GRAPH_SLAM_DRAWABLE_HPP
#define HDL_GRAPH_SLAM_DRAWABLE_HPP

#include <memory>
#include <glk/glsl_shader.hpp>

namespace hdl_graph_slam {

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

    virtual bool available() const = 0;

    virtual void draw(glk::GLSLShader& shader) = 0;

    virtual void draw(glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix) = 0;
};

}  // namespace hdl_graph_slam

#endif