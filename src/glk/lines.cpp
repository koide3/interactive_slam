#include <glk/lines.hpp>

#include <iostream>

namespace glk {

Lines::Lines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors, const std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>& infos)
: num_vertices(vertices.size())
{
  vao = vbo = cbo = ibo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 3, vertices.data(), GL_STATIC_DRAW);

  if(!colors.empty()){
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * colors.size() * 4, colors.data(), GL_STATIC_DRAW);
  }

  if(!infos.empty()) {
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ARRAY_BUFFER, ibo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(int) * infos.size() * 4, infos.data(), GL_STATIC_DRAW);
  }

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

Lines::~Lines() {
  glDeleteBuffers(1, &vbo);
  if(cbo) {
    glDeleteBuffers(1, &cbo);
  }
  if(ibo) {
    glDeleteBuffers(1, &ibo);
  }
  glDeleteVertexArrays(1, &vao);
}

void Lines::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  GLint color_loc = 0;
  GLint info_loc = 0;

  glBindVertexArray(vao);

  glEnableVertexAttribArray(position_loc);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

  if(cbo) {
    color_loc = shader.attrib("vert_color");
    glEnableVertexAttribArray(color_loc);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
  }

  if(ibo) {
    info_loc = shader.attrib("vert_info");
    glEnableVertexAttribArray(info_loc);
    glBindBuffer(GL_ARRAY_BUFFER, ibo);
    glVertexAttribPointer(info_loc, 4, GL_INT, GL_FALSE, 0, 0);
  }

  glDrawArrays(GL_LINES, 0, num_vertices);

  glDisableVertexAttribArray(position_loc);

  if(cbo) {
    glDisableVertexAttribArray(color_loc);
  }
  if(ibo) {
    glDisableVertexAttribArray(info_loc);
  }

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}
}