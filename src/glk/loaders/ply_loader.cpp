#include <glk/loaders/ply_loader.hpp>

#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Core>

#include <glk/mesh_utils.hpp>

namespace glk {

PLYLoader::PLYLoader(const std::string &filename) {
  std::ifstream ifs(filename);
  if (!ifs) {
    std::cerr << "error: failed to open " << filename << std::endl;
    return;
  }

  int num_vertices = 0;
  int num_faces = 0;
  std::vector<std::string> properties;
  while (!ifs.eof()) {
    std::string line;
    std::getline(ifs, line);

    if (line.empty()) {
      continue;
    }

    std::stringstream sst(line);
    std::string token;

    if (line.find("element vertex") != std::string::npos) {
      sst >> token >> token >> num_vertices;
    }
    if (line.find("element face") != std::string::npos) {
      sst >> token >> token >> num_faces;
    }

    if (line.find("property float32") != std::string::npos) {
      std::string property;
      sst >> token >> token >> property;
      properties.push_back(property);
    }

    if (line.find("end") != std::string::npos) {
      break;
    }
  }

  vertices.resize(num_vertices);
  for (int i = 0; i < num_vertices; i++) {
    std::string line;
    std::getline(ifs, line);

    std::stringstream sst(line);
    sst >> vertices[i][0] >> vertices[i][1] >> vertices[i][2];
  }

  indices.resize(num_faces * 3);
  for (int i = 0; i < num_faces; i++) {
    std::string line;
    std::getline(ifs, line);

    int faces = 0;
    std::stringstream sst(line);
    sst >> faces >> indices[i * 3 + 2] >> indices[i * 3 + 1] >> indices[i * 3];

    if (faces != 3) {
      std::cerr << "error : only faces with three vertices are supported!!" << std::endl;
    }
  }

  NormalEstimater nest(vertices, indices);
  normals = nest.normals;
}

}  // namespace glk
