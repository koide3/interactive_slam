#include <memory>
#include <imgui.h>
#include <portable-file-dialogs.h>

#include <boost/format.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <glk/lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/progress_modal.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <hdl_graph_slam/version_modal.hpp>

#include <ros/package.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace hdl_graph_slam {

/**
 * @brief Odometry frame
 *
 */
struct OdometryFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<OdometryFrame>;

  static OdometryFrame::Ptr load(const std::string& cloud_filename, const std::string& pose_filename) {
    std::ifstream ifs(pose_filename);
    if(!ifs) {
      std::cerr << "error : failed to load " << pose_filename << std::endl;
      return nullptr;
    }

    Eigen::Matrix4d mat;
    for(int i = 0; i < 4; i++) {
      for(int j = 0; j < 4; j++) {
        ifs >> mat(i, j);
      }
    }

    Eigen::Isometry3d pose(mat);

    return std::make_shared<OdometryFrame>(cloud_filename, pose);
  }

public:
  OdometryFrame(const std::string& raw_cloud_path, const Eigen::Isometry3d& pose)
  :  raw_cloud_path(raw_cloud_path),
  cloud_(nullptr),
  pose(pose),
  downsample_resolution(0.2f)
  {
    char underscore;
    std::stringstream sst(boost::filesystem::path(raw_cloud_path).filename().string());
    sst >> stamp_sec >> underscore >> stamp_usec;
  }

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud() {
    return cloud(downsample_resolution);
  }

  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud(float downsample_resolution) {
      if(cloud_ == nullptr || std::abs(this->downsample_resolution - downsample_resolution) > 0.01f) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        if(pcl::io::loadPCDFile(raw_cloud_path, *raw_cloud)) {
          std::cerr << "error : failed to load " << raw_cloud_path << std::endl;
          abort();
        }

        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
        voxel_grid.setInputCloud(raw_cloud);
        voxel_grid.filter(*downsampled);

        this->downsample_resolution = downsample_resolution;
        cloud_ = downsampled;
        cloud_buffer.reset(new glk::PointCloudBuffer(cloud_));
      }

      return cloud_;
  }

  void draw(glk::GLSLShader& shader, float downsample_resolution=0.1f) {
    cloud(downsample_resolution);

    shader.set_uniform("color_mode", 0);
    shader.set_uniform("model_matrix", pose.cast<float>().matrix());
    cloud_buffer->draw(shader);

    shader.set_uniform("color_mode", 1);
    shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
    auto& sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);
    sphere.draw(shader);
  }

  unsigned long stamp_sec;
  unsigned long stamp_usec;
  Eigen::Isometry3d pose;
  std::string raw_cloud_path;

private:
  float downsample_resolution;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

  std::unique_ptr<glk::PointCloudBuffer> cloud_buffer;
};


/**
 * @brief Odometry frame set
 *
 */
class OdometrySet {
public:
  OdometrySet(guik::ProgressInterface& progress, const std::string& directory) {
    progress.set_text("sweeping the directory");

    boost::filesystem::directory_iterator itr(directory);
    boost::filesystem::directory_iterator end;

    std::vector<std::string> filenames;
    for(itr; itr != end; itr++) {
      if(itr->path().extension() != ".pcd") {
        continue;
      }

      std::string odom_filename = itr->path().parent_path().string() + "/" + itr->path().stem().string() + ".odom";

      if(!boost::filesystem::exists(odom_filename)) {
        continue;
      }

      filenames.push_back(itr->path().stem().string());
    }

    progress.set_text("loading odometry frames");
    progress.set_maximum(filenames.size());
    progress.set_current(0);

    std::sort(filenames.begin(), filenames.end());
    for(const auto& filename: filenames) {
      progress.increment();

      auto frame = OdometryFrame::load(directory + "/" + filename + ".pcd", directory + "/" + filename + ".odom");
      if(frame == nullptr) {
        continue;
      }

      frames.push_back(frame);
    }
  }

  void select_keyframes(float keyframe_delta_x, float keyframe_delta_angle) {
    if(frames.empty()) {
      return;
    }

    keyframes.clear();
    keyframes.push_back(frames.front());
    for(const auto& frame: frames) {
      const auto& last_keyframe_pose = keyframes.back()->pose;
      const auto& current_frame_pose = frame->pose;

      Eigen::Isometry3d delta = last_keyframe_pose.inverse() * current_frame_pose;
      double delta_x = delta.translation().norm();
      double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();

      if(delta_x > keyframe_delta_x || delta_angle > keyframe_delta_angle) {
        keyframes.push_back(frame);
      }
    }
  }

  void draw(glk::GLSLShader& shader, float downsample_resolution=0.1f) {
    for(const auto& keyframe: keyframes) {
      keyframe->draw(shader, downsample_resolution);
    }
  }

  bool save(guik::ProgressInterface& progress, const std::string& dst_directory) {
    if(keyframes.empty()) {
      return false;
    }

    if(!save_graph(progress, dst_directory + "/graph.g2o")) {
      return false;
    }

    if(!save_keyframes(progress, dst_directory)) {
      return false;
    }

    return true;
  }

private:
  bool save_graph(guik::ProgressInterface& progress, const std::string& filename) const {
    progress.set_text("save graph file");
    progress.increment();

    std::ofstream ofs(filename);
    if(!ofs) {
      return false;
    }

    for(int i=0; i<keyframes.size(); i++) {
      std::unique_ptr<g2o::VertexSE3> v(new g2o::VertexSE3());
      v->setEstimate(keyframes[i]->pose);

      ofs << "VERTEX_SE3:QUAT " << i << " ";
      v->write(ofs);
      ofs << std::endl;
    }
    ofs << "FIX 0" << std::endl;

    for(int i=0; i<keyframes.size()-1; i++) {
      const auto& delta_pose = keyframes[i]->pose.inverse() * keyframes[i + 1]->pose;
      std::unique_ptr<g2o::EdgeSE3> e(new g2o::EdgeSE3());
      e->setMeasurement(delta_pose);
      e->setInformation(Eigen::MatrixXd::Identity(6, 6));

      ofs << "EDGE_SE3:QUAT " << i << " " << i + 1 << " ";
      e->write(ofs);
      ofs << std::endl;
    }

    ofs.close();

    return true;
  }

  bool save_keyframes(guik::ProgressInterface& progress, const std::string& directory) const {
    for(int i=0; i<keyframes.size(); i++) {
      std::string keyframe_directory = (boost::format("%s/%06d") % directory % i).str();
      boost::filesystem::create_directories(keyframe_directory);

      boost::filesystem::copy_file(keyframes[i]->raw_cloud_path, keyframe_directory + "/raw.pcd");
      pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *keyframes[i]->cloud());

      std::ofstream ofs(keyframe_directory + "/data");
      if(!ofs) {
        return false;
      }

      ofs << "stamp " << keyframes[i]->stamp_sec << " " << keyframes[i]->stamp_usec << std::endl;
      ofs << "estimate" << std::endl << keyframes[i]->pose.matrix() << std::endl;
      ofs << "odom " << std::endl << keyframes[i]->pose.matrix() << std::endl;
      ofs << "id " << i << std::endl;
    }

    return true;
  }

private:
  std::vector<OdometryFrame::Ptr> frames;
  std::vector<OdometryFrame::Ptr> keyframes;
};


/**
 * @brief Application to convert an odometry sequence into the graph description format
 *
 */
class Odometry2GraphApplication : public guik::Application {
public:
  Odometry2GraphApplication() : Application() {}
  ~Odometry2GraphApplication() {}

  /**
   * @brief initialize the application
   *
   * @param size            window size
   * @param glsl_version    glsl version
   * @return if successfully initialized
   */
  bool init(const Eigen::Vector2i& size, const char* glsl_version = "#version 330") override {
    if(!Application::init(size, glsl_version)) {
      return false;
    }

    framebuffer_size = size;
    progress.reset(new guik::ProgressModal("progress modal"));

    std::string package_path = ros::package::getPath("interactive_slam");
    std::string data_directory = package_path + "/data";

    main_canvas.reset(new guik::GLCanvas(data_directory, size));
    if(!main_canvas->ready()) {
      close();
    }

    version_modal.reset(new VersionModal());

    delta_updated = false;
    keyframe_delta_x = 3.0f;
    keyframe_delta_angle = 1.0f;
    downsample_resolution = 0.2f;

    return true;
  }

  /**
   * @brief draw ImGui-based UI
   *
   */
  virtual void draw_ui() override {
    main_canvas->draw_ui();

    {
      ImGui::Begin("keyframe settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::DragFloat("downsample_resolution", &downsample_resolution, 0.01f, 0.01f, 1.0f);

      bool updated = false;
      updated |= ImGui::DragFloat("keyframe_delta_x", &keyframe_delta_x, 0.1f, 0.1f, 100.0f);
      updated |= ImGui::DragFloat("keyframe_delta_angle", &keyframe_delta_angle, 0.01f, 0.01f, 3.15f);

      if(delta_updated && !updated) {
        odometry_set->select_keyframes(keyframe_delta_x, keyframe_delta_angle);
      }
      delta_updated = updated;

      ImGui::End();
    }

    main_menu();
    mouse_control();
  }

  /**
   * @brief draw OpenGL-related things
   *
   */
  virtual void draw_gl() override {
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    main_canvas->bind();
    main_canvas->shader->set_uniform("color_mode", 2);
    main_canvas->shader->set_uniform("model_matrix", (Eigen::UniformScaling<float>(3.0f) * Eigen::Isometry3f::Identity()).matrix());

    const auto& coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
    coord.draw(*main_canvas->shader);

    main_canvas->shader->set_uniform("color_mode", 1);
    main_canvas->shader->set_uniform("model_matrix", (Eigen::Translation3f(Eigen::Vector3f::UnitZ() * -0.02) * Eigen::Isometry3f::Identity()).matrix());
    main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
    const auto& grid = glk::Primitives::instance()->primitive(glk::Primitives::GRID);
    grid.draw(*main_canvas->shader);

    main_canvas->shader->set_uniform("point_scale", 50.0f);
    if(odometry_set) {
      odometry_set->draw(*main_canvas->shader, downsample_resolution);
    }

    main_canvas->unbind();
    main_canvas->render_to_screen();

    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  }

  /**
   * @brief frame buffer size change callback
   * @param size  frame buffer size
   */
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size) override {
    main_canvas->set_size(size);
    framebuffer_size = size;
  }

private:
  /**
   * @brief draw main menu
   *
   */
  void main_menu() {
    ImGui::BeginMainMenuBar();

    bool open_dialog = false;
    bool save_dialog = false;
    if(ImGui::BeginMenu("File")) {
      if(ImGui::MenuItem("Open")) {
        open_dialog = true;
      }

      if(ImGui::MenuItem("Save")) {
        save_dialog = true;
      }

      if(ImGui::MenuItem("Quit")) {
        close();
      }

      ImGui::EndMenu();
    }

    open(open_dialog);
    save(save_dialog);

    if(ImGui::BeginMenu("View")) {
      if(ImGui::MenuItem("Reset camera")) {
        main_canvas->reset_camera();
      }
      if(ImGui::MenuItem("Projection setting")) {
        main_canvas->show_projection_setting();
      }
      ImGui::EndMenu();
    }

    bool show_version = false;
    if(ImGui::BeginMenu("Help")) {
      if(ImGui::MenuItem("About")) {
        show_version = true;
      }

      ImGui::EndMenu();
    }

    if(show_version) {
      version_modal->open();
    }
    version_modal->run();

    ImGui::EndMainMenuBar();
  }

private:
  /**
   * @brief open odometry data
   *
   * @param open_dialog
   */
  void open(bool open_dialog) {
    if(progress->run("open")) {
      auto result = progress->result<std::shared_ptr<OdometrySet>>();
      if(result == nullptr) {
        pfd::message message("Error", "failed to load odometry data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }

      odometry_set = result;
      odometry_set->select_keyframes(keyframe_delta_x, keyframe_delta_angle);
    }

    if(!open_dialog) {
      return;
    }

    pfd::select_folder dialog("choose odometry directory");
    while(!dialog.ready()) {
      usleep(100);
    }

    if(dialog.result().empty()) {
      return;
    }

    std::string directory = dialog.result();
    auto open_task = [=](guik::ProgressInterface& p) { return std::make_shared<OdometrySet>(p, directory); };
    progress->open<std::shared_ptr<OdometrySet>>("open", open_task);
  }

  /**
   * @brief save graph description
   *
   * @param save_dialog
   */
  void save(bool save_dialog) {
    if(progress->run("save")) {
      auto result = progress->result<bool>();
      if(!result) {
        pfd::message message("Error", "failed to save graph data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }
    }

    if(!save_dialog) {
      return;
    }

    pfd::select_folder dialog("choose odometry directory");
    while(!dialog.ready()) {
      usleep(100);
    }

    if(dialog.result().empty()) {
      return;
    }

    std::string directory = dialog.result();
    auto save_task = [this, directory](guik::ProgressInterface& p) { return odometry_set->save(p, directory); };
    progress->open<bool>("save", save_task);
  }

  /**
   * @brief mouse event handler
   *
   */
  void mouse_control() {
    ImGuiIO& io = ImGui::GetIO();
    if(!io.WantCaptureMouse) {
      main_canvas->mouse_control();
    }
  }

private:
  Eigen::Vector2i framebuffer_size;

  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<guik::ProgressModal> progress;

  std::unique_ptr<VersionModal> version_modal;

  bool delta_updated;
  float keyframe_delta_x;
  float keyframe_delta_angle;
  float downsample_resolution;
  std::shared_ptr<OdometrySet> odometry_set;
};

}  // namespace hdl_graph_slam


/**
 * @brief main
 */
int main(int argc, char** argv) {
  std::unique_ptr<guik::Application> app(new hdl_graph_slam::Odometry2GraphApplication());

  if(!app->init(Eigen::Vector2i(1920, 1080))) {
    return 1;
  }

  app->run();

  return 0;
}