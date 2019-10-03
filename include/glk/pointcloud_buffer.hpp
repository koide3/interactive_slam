#ifndef GLK_POINTCLOUD_BUFFER_HPP
#define GLK_POINTCLOUD_BUFFER_HPP

#include <memory>
#include <Eigen/Dense>
#include <glk/glsl_shader.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

class PointCloudBuffer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<PointCloudBuffer>;

    PointCloudBuffer(const std::string& cloud_filename);
    PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
    ~PointCloudBuffer();

    void draw(glk::GLSLShader& shader);

private:
    GLuint vao;
    GLuint vbo;
    int stride;
    int num_points;
};

}

#endif