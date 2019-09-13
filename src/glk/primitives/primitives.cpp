#include <glk/primitives/primitives.hpp>

#include <iostream>
#include <glk/mesh.hpp>
#include <glk/lines.hpp>
#include <glk/mesh_utils.hpp>

#include <glk/primitives/grid.hpp>
#include <glk/primitives/cube.hpp>
#include <glk/primitives/icosahedron.hpp>
#include <glk/primitives/coordinate_system.hpp>
#include <glk/loaders/ply_loader.hpp>

namespace glk {

Primitives* Primitives::instance_ = nullptr;

const glk::Drawable& Primitives::primitive(PrimitiveType type) {
  if (meshes[type] == nullptr) {
    switch (type) {
      default:
        std::cerr << "error : unknown primitive type " << type << std::endl;
        break;
      case ICOSAHEDRON: {
        glk::Icosahedron icosahedron;
        glk::Flatize flat(icosahedron.vertices, icosahedron.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices));
      } break;
      case SPHERE: {
        glk::Icosahedron icosahedron;
        icosahedron.subdivide();
        icosahedron.subdivide();
        icosahedron.spherize();
        meshes[type].reset(new glk::Mesh(icosahedron.vertices, icosahedron.normals, icosahedron.indices));
      } break;
      case CUBE: {
        glk::Cube cube;
        glk::Flatize flat(cube.vertices, cube.indices);
        meshes[type].reset(new glk::Mesh(flat.vertices, flat.normals, flat.indices));
      } break;
      case GRID: {
        glk::Grid grid;
        meshes[type].reset(new glk::Lines(0.01f, grid.vertices));
      } break;
      case BUNNY: {
        glk::PLYLoader ply("data/models/bunny.ply");
        meshes[type].reset(new glk::Mesh(ply.vertices, ply.normals, ply.indices));
      } break;
      case COORDINATE_SYSTEM: {
        glk::CoordinateSystem coord;
        meshes[type].reset(new glk::Lines(0.01f, coord.vertices, coord.colors));
      } break;
    }
  }

  return *meshes[type];
}
}  // namespace glk