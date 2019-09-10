#ifndef GLK_PRIMITIVES_COORDINATE_SYSTEM_HPP
#define GLK_PRIMITIVES_COORDINATE_SYSTEM_HPP

#include <vector>
#include <Eigen/Core>

#include <GL/gl3w.h>
#include <glk/drawble.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

class CoordinateSystem : public glk::Drawable {
public:
  CoordinateSystem() {
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;

    vertices.push_back(Eigen::Vector3f::Zero());
    vertices.push_back(Eigen::Vector3f::UnitX());
    vertices.push_back(Eigen::Vector3f::Zero());
    vertices.push_back(Eigen::Vector3f::UnitY());
    vertices.push_back(Eigen::Vector3f::Zero());
    vertices.push_back(Eigen::Vector3f::UnitZ());

    for (const auto& vertex : vertices) {
      colors.push_back(Eigen::Vector4f(vertex[0], vertex[1], vertex[2], 1.0f));
    }

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  virtual ~CoordinateSystem() {
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &vao);
  }

  virtual void draw(glk::GLSLShader& shader) const override {
    shader.set_uniform("color_mode", 1);
    GLint position_loc = shader.attrib("vert_position");

    glBindVertexArray(vao);
    glEnableVertexAttribArray(position_loc);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    // large overhead implementation due to the shader design...
    shader.set_uniform("material_color", Eigen::Vector4f::UnitX().eval());
    glDrawArrays(GL_LINES, 0, 2);
    shader.set_uniform("material_color", Eigen::Vector4f::UnitY().eval());
    glDrawArrays(GL_LINES, 2, 2);
    shader.set_uniform("material_color", Eigen::Vector4f::UnitZ().eval());
    glDrawArrays(GL_LINES, 4, 2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(0);
    glBindVertexArray(0);
  }

public:
  GLuint vao;
  GLuint vbo;
};
}  // namespace glk

#endif