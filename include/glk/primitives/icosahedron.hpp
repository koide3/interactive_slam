#ifndef GLK_PRIMITIVES_ICOSAHEDRON_HPP
#define GLK_PRIMITIVES_ICOSAHEDRON_HPP

#include <map>
#include <vector>
#include <Eigen/Core>

namespace glk {

class Icosahedron {
public:
  Icosahedron() {
    double t = (1.0 + std::sqrt(5.0)) / 2.0;

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices = {
        Eigen::Vector3f(-1, t, 0), Eigen::Vector3f(1, t, 0), Eigen::Vector3f(-1, -t, 0), Eigen::Vector3f(1, -t, 0),
        Eigen::Vector3f(0, -1, t), Eigen::Vector3f(0, 1, t), Eigen::Vector3f(0, -1, -t), Eigen::Vector3f(0, 1, -t),
        Eigen::Vector3f(t, 0, -1), Eigen::Vector3f(t, 0, 1), Eigen::Vector3f(-t, 0, -1), Eigen::Vector3f(-t, 0, 1)};

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals(vertices.size());
    for(int i = 0; i < vertices.size(); i++) {
      normals[i] = vertices[i].normalized();
    }

    std::vector<int> indices = {0, 11, 5,  0, 5,  1, 0, 1, 7, 0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11,
                                4, 11, 10, 2, 10, 7, 6, 7, 1, 8, 3,  9,  4, 3,  4,  2, 3, 2, 6, 3,
                                6, 8,  3,  8, 9,  4, 9, 5, 2, 4, 11, 6,  2, 10, 8,  6, 7, 9, 8, 1};

    this->vertices.swap(vertices);
    this->normals.swap(normals);
    this->indices.swap(indices);
  }

  void subdivide() {
    std::vector<int> new_indices;
    for(int i = 0; i < indices.size(); i += 3) {
      int a = insert_middle_point(indices[i], indices[i + 1]);
      int b = insert_middle_point(indices[i + 1], indices[i + 2]);
      int c = insert_middle_point(indices[i + 2], indices[i]);

      std::vector<int> tessellated = {indices[i], a, c, indices[i + 1], b, a, indices[i + 2], c, b, a, b, c};

      new_indices.insert(new_indices.end(), tessellated.begin(), tessellated.end());
    }

    indices.swap(new_indices);

    normals.resize(vertices.size());
    for(int i = 0; i < vertices.size(); i++) {
      normals[i] = vertices[i].normalized();
    }
  }

  void spherize() {
    for(auto& vertex : vertices) {
      vertex.normalize();
    }

    normals.resize(vertices.size());
    for(int i = 0; i < vertices.size(); i++) {
      normals[i] = vertices[i].normalized();
    }
  }

private:
  int insert_middle_point(int v1, int v2) {
    int smaller = std::min(v1, v2);
    int greater = std::max(v1, v2);
    int key = (smaller << 16) + greater;

    auto found = middle_points_cache.find(key);
    if(found != middle_points_cache.end()) {
      return found->second;
    }

    Eigen::Vector3f new_v = (vertices[v1] + vertices[v2]) / 2.0f;
    vertices.push_back(new_v);

    middle_points_cache.insert(found, std::make_pair(key, vertices.size() - 1));
    return vertices.size() - 1;
  }

public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<int> indices;

  std::map<int, int> middle_points_cache;
};

}  // namespace glk

#endif