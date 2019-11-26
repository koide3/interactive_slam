#ifndef GLK_DRAWABLE_HPP
#define GLK_DRAWABLE_HPP

#include <vector>
#include <Eigen/Core>

#include <GL/gl3w.h>
#include <glk/glsl_shader.hpp>

namespace glk {

/**
 * @brief Drawable object interface
 *
 */
class Drawable {
public:
  virtual ~Drawable() {}

  virtual void draw(glk::GLSLShader& shader) const = 0;
};

}  // namespace glk

#endif