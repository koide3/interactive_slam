#ifndef GLK_LINES_HPP
#define GLK_LINES_HPP

#include <GL/gl3w.h>

#include <vector>
#include <Eigen/Core>

#include <glk/drawble.hpp>
#include <glk/glsl_shader.hpp>

namespace glk {

class Lines : public Drawable {
public:
  Lines(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
        const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors,
        const std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>& infos);
  virtual ~Lines() override;

  virtual void draw(glk::GLSLShader& shader) const override;

private:
  Lines(const Lines&);
  Lines& operator=(const Lines&);

private:
  int num_vertices;

  GLuint vao;
  GLuint vbo;
  GLuint cbo;
  GLuint ibo;
};
}

#endif