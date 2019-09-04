#ifndef GLK_POINTCLOUD_CANVAS_HPP
#define GLK_POINTCLOUD_CANVAS_HPP

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <nanogui/nanogui.h>

#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/texture_renderer.hpp>
#include <glk/pointcloud_buffer.hpp>

#include <glk/mesh.hpp>
#include <glk/primitives/icosahedron.hpp>

#include <guik/camera_control.hpp>

#include <opencv2/opencv.hpp>

namespace guik
{

class PointCloudCanvas : public nanogui::GLCanvas {
public:
    PointCloudCanvas(nanogui::Widget* parent, const std::string& shader_path) : nanogui::GLCanvas(parent) {
        shader.reset(new glk::GLSLShader());
        shader->init(shader_path);
        camera_control.reset(new guik::ArcCameraControl());

        texture_renderer.reset(new glk::TextureRenderer());

        glk::Icosahedron icosahedron;
        icosahedron.subdivide();
        icosahedron.subdivide();
        icosahedron.spherize();
        mesh.reset(new glk::Mesh(icosahedron.vertices, icosahedron.indices));

    }

    virtual void drawGL() override {
        frame_buffer->bind();
        glDisable(GL_SCISSOR_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        GLint clear_color[] = {-1, -1, -1, -1};
        glClearTexImage(frame_buffer->color(1).id(), 0, GL_RGBA_INTEGER, GL_INT, clear_color);

        shader->use();

        Eigen::Matrix4f view_matrix = camera_control->view_matrix();
        glm::mat4 proj = glm::perspective<float>(120.0, 1.0, 1.0, 500.0);
        Eigen::Matrix4f projection_matrix = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));
        // Eigen::Matrix4f projection_matrix = nanogui::frustum(0.0f, mSize[0], mSize[1], 0.0f, 0.1f, 500.0f);

        shader->set_uniform("view_matrix", view_matrix);
        shader->set_uniform("projection_matrix", projection_matrix);
        shader->set_uniform("info_values", Eigen::Vector4i(0, 128, 0, 255));

        glEnable(GL_DEPTH_TEST);
        for(const auto& cloud_buffer: cloud_buffers) {
            cloud_buffer->draw(*shader);
        }
        mesh->draw(*shader);
        glDisable(GL_DEPTH_TEST);
        glFlush();

        frame_buffer->unbind();

        glEnable(GL_SCISSOR_TEST);
        texture_renderer->draw(frame_buffer->color(0).id());
    }

    void setSize(const Eigen::Vector2i& size) {
        GLCanvas::setSize(size);
        camera_control->set_size(size);

        frame_buffer.reset(new glk::FrameBuffer(size, 2));
    }

    void add_cloud(const glk::PointCloudBuffer::Ptr& cloud_buffer) {
        cloud_buffers.push_back(cloud_buffer);
    }

    virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override {
        if(button == GLFW_MOUSE_BUTTON_RIGHT) {
            float depth = pick_depth(p);
            if(depth < 1.0f) {
                std::cout << "picked:" << unproject(p, depth) << std::endl;
            } else {
                std::cout << "no depth" << std::endl;
            }
        }

        camera_control->mouse(p, button, down);
        return true;
    }

    virtual bool mouseDragEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) override {
        camera_control->drag(p, rel, button);
        return true;
    }

    virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) override {
        camera_control->drag(p, rel, button);
        return true;
    }

    virtual bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) override {
        camera_control->scroll(rel);
        return true;
    }

    Eigen::Vector4i pick_info(const Eigen::Vector2i& p, int window=2) const {
        if(p[0] < 5 || p[1] < 5 || p[0] > mSize[0] - 5 || p[1] > mSize[1] - 5) {
            return Eigen::Vector4i(-1, -1, -1, -1);
        }

        std::vector<int> pixels = frame_buffer->color(1).read_pixels<int>(GL_RGBA_INTEGER, GL_INT);

        std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

        for(int i=-window; i<=window; i++) {
            for(int j=-window; j<=window; j++) {
                ps.push_back(Eigen::Vector2i(i, j));
            }
        }

        std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
        for(int i=0; i<ps.size(); i++) {
            Eigen::Vector2i p_ = p + ps[i];
            int index = (p[1] * mSize[0] + p_[0]) * 4;
            Eigen::Vector4i info = Eigen::Map<Eigen::Vector4i>(&pixels[index]);

            if(info[3] >= 0) {
                return info;
            }
        }

        return Eigen::Vector4i(-1, -1, -1, -1);
    }

    float pick_depth(const Eigen::Vector2i& p, int window=2) const {
        if(p[0] < 5 || p[1] < 5 || p[0] > mSize[0] - 5 || p[1] > mSize[1] - 5) {
            return -1.0f;
        }

        std::vector<float> pixels = frame_buffer->depth().read_pixels<float>(GL_DEPTH_COMPONENT, GL_FLOAT);

        std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> ps;

        for(int i=-window; i<=window; i++) {
            for(int j=-window; j<=window; j++) {
                ps.push_back(Eigen::Vector2i(i, j));
            }
        }

        std::sort(ps.begin(), ps.end(), [=](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) { return lhs.norm() < rhs.norm(); });
        for(int i=0; i<ps.size(); i++) {
            Eigen::Vector2i p_ = p + ps[i];
            int index = ((mSize[1] - p[1]) * mSize[0] + p_[0]);
            float depth = pixels[index];

            if(depth < 1.0f) {
                return depth;
            }
        }

        return 1.0f;
    }

    Eigen::Vector3f unproject(const Eigen::Vector2i& p, float depth) const {
        Eigen::Matrix4f view_matrix = camera_control->view_matrix();
        glm::mat4 proj = glm::perspective<float>(120.0, 1.0, 1.0, 500.0);
        Eigen::Matrix4f projection_matrix = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));

        Eigen::Matrix4f vp = projection_matrix * view_matrix;

        Eigen::Vector2f p_(static_cast<float>(p[0]) / mSize[0], static_cast<float>(p[1]) / mSize[1]);
        Eigen::Vector4f unprojected = vp.inverse() * Eigen::Vector4f(p_[0], 1.0f - p_[1], depth, 1.0f);

        return unprojected.head<3>();
    }

private:
    std::unique_ptr<glk::FrameBuffer> frame_buffer;
    std::unique_ptr<glk::TextureRenderer> texture_renderer;

    std::unique_ptr<glk::GLSLShader> shader;
    std::vector<glk::PointCloudBuffer::Ptr> cloud_buffers;

    std::unique_ptr<glk::Mesh> mesh;

    std::unique_ptr<guik::CameraControl> camera_control;
};


}

#endif