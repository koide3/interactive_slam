#include <glk/lines.hpp>

#include <iostream>
#include <Eigen/Geometry>

namespace glk {

Lines::Lines(float line_width,
const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors,
const std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>& infos)
: num_vertices(vertices.size()) {
  vao = vbo = cbo = ibo = 0;

  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices_ext(vertices.size() * 4);
  for(int i=0; i<vertices.size(); i+=2) {
    Eigen::Vector3f x = Eigen::Vector3f::UnitZ().cross(vertices[i+1] - vertices[i]).normalized();
    Eigen::Vector3f y = x.cross(vertices[i + 1] - vertices[i]).normalized();

    vertices_ext[i * 4] = vertices[i] - x * line_width;
    vertices_ext[i * 4 + 1] = vertices[i + 1] - x * line_width;
    vertices_ext[i * 4 + 2] = vertices[i] + x * line_width;
    vertices_ext[i * 4 + 3] = vertices[i + 1] + x * line_width;

    vertices_ext[i * 4 + 4] = vertices[i] - y * line_width;
    vertices_ext[i * 4 + 5] = vertices[i + 1] - y * line_width;
    vertices_ext[i * 4 + 6] = vertices[i] + y * line_width;
    vertices_ext[i * 4 + 7] = vertices[i + 1] + y * line_width;
  }

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices_ext.size() * 3, vertices_ext.data(), GL_STATIC_DRAW);

  if(!colors.empty()){
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors_ext(colors.size() * 4);
    for(int i = 0; i < colors.size(); i+=2) {
      for(int j = 0; j < 4; j++) {
        colors_ext[i * 4 + j * 2] = colors[i];
        colors_ext[i * 4 + j * 2 + 1] = colors[i + 1];
      }
    }
    glGenBuffers(1, &cbo);
    glBindBuffer(GL_ARRAY_BUFFER, cbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * colors_ext.size() * 4, colors_ext.data(), GL_STATIC_DRAW);
  }

  if(!infos.empty()) {
    std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>> infos_ext(infos.size() * 4);
    for(int i = 0; i < infos.size(); i += 2) {
      for(int j = 0; j < 4; j++) {
        infos_ext[i * 4 + j * 2] = infos[i];
        infos_ext[i * 4 + j * 2 + 1] = infos[i + 1];
      }
    }
    glGenBuffers(1, &ibo);
    glBindBuffer(GL_ARRAY_BUFFER, ibo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(int) * infos_ext.size() * 4, infos_ext.data(), GL_STATIC_DRAW);
  }

  /*
  std::vector<int> indices;
  for(int i = 0; i < vertices_ext.size(); i += 4) {
    indices.push_back(i);
    indices.push_back(i + 1);
    indices.push_back(i + 2);
    indices.push_back(i + 1);
    indices.push_back(i + 3);
    indices.push_back(i + 2);
  }
  */

  std::vector<int> sub_indices = {
      0, 1, 4,
      1, 5, 4,
      4, 5, 2,
      5, 3, 2,
      2, 3, 6,
      3, 6, 7,
      6, 7, 0,
      7, 1, 0
  };

  std::vector<int> indices;
  for(int i = 0; i < vertices_ext.size(); i += 8) {
    for(int j=0; j<sub_indices.size(); j++) {
      indices.push_back(sub_indices[j] + i);
    }
  }
  num_indices = indices.size();

  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), indices.data(), GL_STATIC_DRAW);

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

  glDeleteBuffers(1, &ebo);
  glDeleteVertexArrays(1, &vao);
}

void Lines::draw(glk::GLSLShader& shader) const {
  GLint position_loc = shader.attrib("vert_position");
  GLint direction_loc = shader.attrib("vert_direction");
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
    glVertexAttribIPointer(info_loc, 4, GL_INT, 0, 0);
  }

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glDisableVertexAttribArray(position_loc);
  glDisableVertexAttribArray(direction_loc);

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