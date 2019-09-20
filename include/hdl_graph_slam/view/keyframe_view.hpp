#ifndef HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP
#define HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP

#include <memory>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>

#include <g2o/types/slam3d/vertex_se3.h>
#include <hdl_graph_slam/view/vertex_view.hpp>
#include <hdl_graph_slam/interactive_keyframe.hpp>

namespace hdl_graph_slam {

class KeyFrameView : public VertexView {
public:
  using Ptr = std::shared_ptr<KeyFrameView>;

  KeyFrameView(const InteractiveKeyFrame::Ptr& kf)
  : VertexView(kf->node)
  {
    keyframe = kf;

    std::cout << kf->id() << " : ";
    pointcloud_buffer.reset(new glk::PointCloudBuffer(kf->cloud));
  }

  InteractiveKeyFrame::Ptr lock() const { return keyframe.lock(); }

  virtual bool available() const override { return !keyframe.expired(); }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) override {
    InteractiveKeyFrame::Ptr kf = keyframe.lock();
    Eigen::Matrix4f model_matrix = kf->estimate().matrix().cast<float>();
    shader.set_uniform("color_mode", 0);
    shader.set_uniform("model_matrix", model_matrix);
    shader.set_uniform("info_values", Eigen::Vector4i(POINTS, 0, 0, 0));

    pointcloud_buffer->draw(shader);

    if (!flags.draw_verticies || !flags.draw_keyframe_vertices) {
      return;
    }

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
    shader.set_uniform("info_values", Eigen::Vector4i(VERTEX | KEYFRAME, kf->id(), 0, 0));

    model_matrix.block<3, 3>(0, 0) *= 0.35;
    shader.set_uniform("model_matrix", model_matrix);
    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    sphere.draw(shader);
  }

  virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader, const Eigen::Vector4f& color, const Eigen::Matrix4f& model_matrix) override {
    if (!available()) {
      return;
    }

    InteractiveKeyFrame::Ptr kf = keyframe.lock();

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("material_color", color);

    shader.set_uniform("model_matrix", model_matrix);
    shader.set_uniform("info_values", Eigen::Vector4i(POINTS, 0, 0, 0));

    pointcloud_buffer->draw(shader);

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("info_values", Eigen::Vector4i(VERTEX | KEYFRAME, kf->id(), 0, 0));
    const auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    sphere.draw(shader);
  }

private:
  std::weak_ptr<InteractiveKeyFrame> keyframe;
  std::unique_ptr<glk::PointCloudBuffer> pointcloud_buffer;
};

}  // namespace hdl_graph_slam

#endif