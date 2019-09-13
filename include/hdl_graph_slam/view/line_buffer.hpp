#ifndef HDL_GRAPH_SLAM_VIEW_LINE_BUFFER_HPP
#define HDL_GRAPH_SLAM_VIEW_LINE_BUFFER_HPP

#include <vector>
#include <Eigen/Core>
#include <glk/lines.hpp>
#include <glk/glsl_shader.hpp>

namespace hdl_graph_slam {

class LineBuffer {
public:
  LineBuffer() {}

  void clear() {
    vertices.clear();
    colors.clear();
    infos.clear();
  }

  void add_line(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector4f& c0, Eigen::Vector4f& c1, const Eigen::Vector4i& info) {
    vertices.push_back(v0);
    vertices.push_back(v1);
    colors.push_back(c0);
    colors.push_back(c1);
    infos.push_back(info);
    infos.push_back(info);
  }

  void draw(glk::GLSLShader& shader) const {
    shader.set_uniform("color_mode", 2);
    shader.set_uniform("model_matrix", Eigen::Matrix4f::Identity().eval());

    glk::Lines lines(0.1f, vertices, colors, infos);
    lines.draw(shader);
  }

private:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
  std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>> infos;
};
}

#endif
