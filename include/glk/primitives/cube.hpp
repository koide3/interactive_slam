#ifndef GLK_PRIMITIVES_CUBE_HPP
#define GLK_PRIMITIVES_CUBE_HPP

#include <map>
#include <vector>
#include <Eigen/Core>

namespace glk {

class Cube {
public:
  Cube() {
    //   6 ------ 7     X
    //  /|       /|     |
    // 4 ------ 5 |     0 --- Z
    // | |      | |    /
    // | 2 -----| 3   Y
    // |/       |/
    // 0 ------ 1
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices = {
        Eigen::Vector3f(-0.5f, -0.5f, -0.5f),  // 0
        Eigen::Vector3f(-0.5f, -0.5f, 0.5f),   // 1
        Eigen::Vector3f(-0.5f, 0.5f, -0.5f),   // 2
        Eigen::Vector3f(-0.5f, 0.5f, 0.5f),    // 3
        Eigen::Vector3f(0.5f, -0.5f, -0.5f),   // 4
        Eigen::Vector3f(0.5f, -0.5f, 0.5f),    // 5
        Eigen::Vector3f(0.5f, 0.5f, -0.5f),    // 6
        Eigen::Vector3f(0.5f, 0.5f, 0.5f),     // 7
    };

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals(vertices.size());
    for (int i = 0; i < vertices.size(); i++) {
      normals[i] = vertices[i].normalized();
    }

    std::vector<int> indices = {0, 1, 2, 1, 3, 2, 1, 5, 3, 3, 5, 7, 0, 4, 1, 1, 4, 5, 3, 6, 2, 3, 7, 6, 0, 2, 4, 2, 6, 4, 4, 6, 5, 5, 6, 7};

    this->vertices.swap(vertices);
    this->indices.swap(indices);
  }

public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<int> indices;
};

}  // namespace glk

#endif