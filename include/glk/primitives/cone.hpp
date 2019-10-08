#ifndef GLK_PRIMITIVES_CONE_HPP
#define GLK_PRIMITIVES_CONE_HPP

#include <map>
#include <vector>
#include <Eigen/Core>

namespace glk {

class Cone {
public:
  Cone(int div=10) {
    vertices.push_back(Eigen::Vector3f::Zero());
    vertices.push_back(Eigen::Vector3f::UnitZ());

    double step = 2.0 * M_PI / div;
    for(int i = 0; i < div; i++) {
      double rad = step * i;
      vertices.push_back(Eigen::Vector3f(std::cos(rad), std::sin(rad), 0.0f));

      int current_index = i + 2;
      int next_index = ((i + 1) % div) + 2;

      indices.push_back(0);
      indices.push_back(current_index);
      indices.push_back(next_index);

      indices.push_back(current_index);
      indices.push_back(1);
      indices.push_back(next_index);
    }
  }

public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<int> indices;
};

}  // namespace glk

#endif