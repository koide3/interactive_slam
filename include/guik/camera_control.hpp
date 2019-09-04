#ifndef GLK_CAMERA_CONTROL_HPP
#define GLK_CAMERA_CONTROL_HPP

#include <memory>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Dense>

namespace guik {


class CameraControl {
public:
    virtual ~CameraControl() {}

    virtual void set_size(const Eigen::Vector2i& size) = 0;

    virtual void mouse(const Eigen::Vector2i& p, int button, bool down) = 0;

    virtual void drag(const Eigen::Vector2i& p, const Eigen::Vector2i& rel, int button) = 0;

    virtual void scroll(const Eigen::Vector2f& rel) = 0;

    virtual Eigen::Matrix4f view_matrix() const = 0;

};


class ArcCameraControl : public CameraControl {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ArcCameraControl()
    : size(400, 400),
    center(0.0f, 0.0f, 0.0f),
    distance(100.0f),
    left_button_down(false),
    theta(0.0f),
    phi(60.0f * M_PI / 180.0f)
    {
        left_button_down = false;
        middle_button_down = false;
    }

    virtual ~ArcCameraControl() override {}

    void set_size(const Eigen::Vector2i& size) override {
        this->size = size;
    }

    void mouse(const Eigen::Vector2i& p, int button, bool down) override {
        if(button == 0) {
            left_button_down = down;
        }
        if(button == 0) {
            middle_button_down = down;
        }
    }

    void drag(const Eigen::Vector2i& p, const Eigen::Vector2i& rel, int button) override {
        if(left_button_down) {
            theta -= rel[0] * 0.01f;;
            phi -= rel[1] * 0.01f;

            phi = std::min(M_PI_2 - 0.1, std::max(-M_PI_2 + 0.1, phi));
        }

        if(middle_button_down) {
            center += Eigen::AngleAxisf(theta + M_PI_2, Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(-rel[0], rel[1], 0.0f) * distance * 0.001f;
        }
    }

    void scroll(const Eigen::Vector2f& rel) override {
        if(rel[1] > 0) {
            distance = distance * 0.5f;
        } else if(rel[1] < 0) {
            distance = distance * 2.0f;
        }

        distance = std::max(0.1, distance);
    }

    Eigen::Quaternionf rotation() const {
        return Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitY());
    }

    Eigen::Matrix4f view_matrix() const override {
        Eigen::Vector3f offset = rotation() * Eigen::Vector3f(distance, 0.0f, 0.0f);
        Eigen::Vector3f eye = center + offset;

        glm::mat4 mat = glm::lookAt(glm::vec3(eye[0], eye[1], eye[2]), glm::vec3(center[0], center[1], center[2]), glm::vec3(0.0f, 0.0f, 1.0f));
        return Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(mat));
    }

private:
    Eigen::Vector2i size;
    Eigen::Vector3f center;
    double distance;

    bool left_button_down;
    double theta;
    double phi;

    bool middle_button_down;
};

}

#endif