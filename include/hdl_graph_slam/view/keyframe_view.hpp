#ifndef HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP
#define HDL_GRAPH_SLAM_KEYFRAME_VIEW_HPP

#include <memory>
#include <glk/pointcloud_buffer.hpp>

#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/view/drawable.hpp>

namespace hdl_graph_slam {

class KeyFrameView : public Drawable {
public:
  using Ptr = std::shared_ptr<KeyFrameView>;

  KeyFrameView(const KeyFrame::Ptr& kf) {
    keyframe = kf;

    pointcloud_buffer.reset(new glk::PointCloudBuffer(kf->cloud));
    pointcloud_buffer->set_model_matrix(kf->estimate().matrix().cast<float>());
  }

  virtual bool available() const override {
    return !keyframe.expired();
  }

  virtual void draw(glk::GLSLShader& shader) override {
    pointcloud_buffer->draw(shader);
  }

private:
  std::weak_ptr<KeyFrame> keyframe;
  std::unique_ptr<glk::PointCloudBuffer> pointcloud_buffer;
};

}

#endif