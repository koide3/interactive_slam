#ifndef GLK_PRIMITIVES_HPP
#define GLK_PRIMITIVES_HPP

#include <glk/mesh.hpp>

namespace glk {

enum PrimitiveType { ICOSAHEDRON=0, SPHERE, COORDINATE_SYSTEM, NUM_PRIMITIVES };

class Primitives {
private:
  Primitives() {
    meshes.resize(NUM_PRIMITIVES, nullptr);
  }

public:
  static Primitives* instance() {
    if(instance_ == nullptr) {
      instance_ = new Primitives();
    }
    return instance_;
  }

  const glk::Drawable& primitive(PrimitiveType type);

private:
  static Primitives* instance_;

  std::vector<std::shared_ptr<glk::Drawable>> meshes;
};
}

#endif