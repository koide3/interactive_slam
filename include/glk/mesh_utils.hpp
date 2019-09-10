#ifndef GLK_PRIMITIVES_MESH_UTILS_HPP
#define GLK_PRIMITIVES_MESH_UTILS_HPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace glk {

class Flatize {
public:
  Flatize(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices_, const std::vector<int>& indices_) {
    vertices.resize(indices_.size());
    normals.resize(indices_.size());
    indices.resize(indices_.size());
    for (int i = 0; i < indices_.size(); i += 3) {
      Eigen::Vector3f v1 = vertices_[indices_[i]];
      Eigen::Vector3f v2 = vertices_[indices_[i + 1]];
      Eigen::Vector3f v3 = vertices_[indices_[i + 2]];

      Eigen::Vector3f n = (v2 - v1).cross(v3 - v2);

      for (int j = 0; j < 3; j++) {
        vertices[i + j] = vertices_[indices_[i + j]];
        normals[i + j] = n;
        indices[i + j] = i + j;
      }
    }
  }

public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<int> indices;
};

class NormalEstimater {
public:
  NormalEstimater(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices, const std::vector<int>& indices) {
    normals.resize(vertices.size(), Eigen::Vector3f::Zero());
    for (int i = 0; i < indices.size(); i += 3) {
      Eigen::Vector3f v1 = vertices[indices[i]];
      Eigen::Vector3f v2 = vertices[indices[i + 1]];
      Eigen::Vector3f v3 = vertices[indices[i + 2]];

      Eigen::Vector3f n = (v2 - v1).cross(v3 - v2);

      normals[indices[i]] += n;
      normals[indices[i + 1]] += n;
      normals[indices[i + 2]] += n;
    }

    for (auto& normal : normals) {
      normal.normalize();
    }
  }

public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
};

}  // namespace glk

#endif