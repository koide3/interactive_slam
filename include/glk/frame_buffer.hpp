#ifndef GLK_FRAME_BUFFER_HPP
#define GLK_FRAME_BUFFER_HPP

#include <memory>
#include <GL/gl3w.h>
#include <Eigen/Core>

#include <glk/texture.hpp>

namespace glk {

/**
 * @brief OpenGL FrameBuffer wrapper class
 *
 */
class FrameBuffer {
public:
  FrameBuffer(const Eigen::Vector2i& size);

  ~FrameBuffer();

  void bind();
  void unbind() const;

  void add_color_buffer(GLuint internal_format = GL_RGBA, GLuint format = GL_RGBA, GLuint type = GL_UNSIGNED_BYTE);

  const Texture& color() { return *color_buffers[0]; }
  const Texture& color(int i) { return *color_buffers[i]; }
  const Texture& depth() { return *depth_buffer; }

private:
  int width;
  int height;

  GLint viewport[4];

  std::vector<std::shared_ptr<Texture>> color_buffers;
  std::shared_ptr<Texture> depth_buffer;

  GLuint frame_buffer;
};

}  // namespace glk

#endif