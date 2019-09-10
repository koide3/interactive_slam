#ifndef GLK_PLY_LOADER_HPP
#define GLK_PLY_LOADER_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

class PLYLoader {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PLYLoader(const std::string& filename);

public:
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
  std::vector<int> indices;
};

}  // namespace glk

#endif