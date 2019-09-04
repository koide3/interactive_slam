#include <glk/pointcloud_buffer.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

    PointCloudBuffer::PointCloudBuffer(const std::string& cloud_filename)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if(pcl::io::loadPCDFile(cloud_filename, *cloud)) {
            std::cerr << "error: failed to load " << cloud_filename << std::endl;
            num_points = 0;
            return;
        }

        stride = sizeof(pcl::PointXYZI);
        num_points = cloud->size();
        std::cout << "num_points " << num_points << std::endl;

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GL_FLOAT) * cloud->size() * sizeof(pcl::PointXYZI), cloud->points.data(), GL_STATIC_DRAW);

        model_matrix.setIdentity();
    }

    PointCloudBuffer::PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
        stride = sizeof(pcl::PointXYZI);
        num_points = cloud->size();
        std::cout << "num_points " << num_points << ", stride " << stride << std::endl;

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GL_FLOAT) * cloud->size() * sizeof(pcl::PointXYZI), cloud->points.data(), GL_STATIC_DRAW);

        model_matrix.setIdentity();
    }


    PointCloudBuffer::~PointCloudBuffer() {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
    }

    void PointCloudBuffer::draw(glk::GLSLShader& shader) {
        if(num_points == 0) {
            return;
        }

        GLint position_loc = shader.attrib("position");
        shader.set_uniform("model_matrix", model_matrix);

        glBindVertexArray(vao);
        glEnableVertexAttribArray(position_loc);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * stride, 0);

        glDrawArrays(GL_POINTS, 0, num_points);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glDisableVertexAttribArray(0);
    }

}
