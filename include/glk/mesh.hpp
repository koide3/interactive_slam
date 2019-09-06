#ifndef GLK_MESH_HPP
#define GLK_MESH_HPP

#include <vector>
#include <Eigen/Dense>

#include <GL/glew.h>
#include <glk/drawble.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

class Mesh : public Drawable {
public:
  Mesh(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, const std::vector<int>& indices) : num_vertices(vertices.size()), num_indices(indices.size()) {
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  }

  virtual ~Mesh() {
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
    glDeleteBuffers(1, &vao);
  }

  virtual void draw(glk::GLSLShader& shader) const override {
    GLint position_loc = shader.attrib("vert_position");

    glBindVertexArray(vao);
    glEnableVertexAttribArray(position_loc);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

    glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(0);
    glBindVertexArray(0);
  }

private:
  Mesh(const Mesh&);
  Mesh& operator=(const Mesh&);

private:
  int num_vertices;
  int num_indices;

  GLuint vao;
  GLuint vbo;
  GLuint ebo;
};

}  // namespace glk

#endif