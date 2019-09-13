#ifndef GLK_PRIMITIVES_HPP
#define GLK_PRIMITIVES_HPP

#include <glk/drawble.hpp>

namespace glk {

class Primitives {
private:
  Primitives() { meshes.resize(NUM_PRIMITIVES, nullptr); }

public:
  enum PrimitiveType { ICOSAHEDRON = 0, SPHERE, CUBE, GRID, COORDINATE_SYSTEM, BUNNY, NUM_PRIMITIVES };

  static Primitives *instance() {
    if (instance_ == nullptr) {
      instance_ = new Primitives();
    }
    return instance_;
  }

  const glk::Drawable &primitive(PrimitiveType type);

private:
  static Primitives *instance_;

  std::vector<std::shared_ptr<glk::Drawable>> meshes;
};
}  // namespace glk

#endif