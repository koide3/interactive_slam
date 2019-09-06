#include <glk/primitives/primitives.hpp>
#include <glk/primitives/icosahedron.hpp>
#include <glk/primitives/coordinate_system.hpp>

namespace glk {

Primitives* Primitives::instance_ = nullptr;

const glk::Drawable& Primitives::primitive(PrimitiveType type) {
  if(meshes[type] == nullptr) {
    switch(type) {
      default:
        std::cerr << "error : unknown primitive type " << type << std::endl;
        break;
      case ICOSAHEDRON:
        {
        glk::Icosahedron icosahedron;
        meshes[type].reset(new glk::Mesh(icosahedron.vertices, icosahedron.indices));
        }
        break;
      case SPHERE:
        {
        glk::Icosahedron icosahedron;
        icosahedron.subdivide();
        icosahedron.subdivide();
        icosahedron.spherize();
        meshes[type].reset(new glk::Mesh(icosahedron.vertices, icosahedron.indices));
        }
        break;
      case COORDINATE_SYSTEM:
        {
          meshes[type].reset(new glk::CoordinateSystem());
        }
        break;
    }
  }

  return *meshes[type];
}
}