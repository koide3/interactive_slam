#include <hdl_graph_slam/manual_loop_close_model.hpp>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

#include <hdl_graph_slam/information_matrix_calculator.hpp>

namespace hdl_graph_slam {

ManualLoopCloseModal::ManualLoopCloseModal(InteractiveGraphView& graph, const std::string& data_directory)
    : graph(graph),
      fitness_score(0),
      registration_method(0),
      fpfh_normal_estimation_radius(0.25f),
      fpfh_search_radius(0.5f),
      fpfh_max_iterations(20000),
      fpfh_num_samples(3),
      fpfh_correspondence_randomness(5),
      fpfh_similarity_threshold(0.85f),
      fpfh_max_correspondence_distance(0.30f),
      fpfh_inlier_fraction(0.25f),
      scan_matching_method(0),
      scan_matching_resolution(2.0) {
  canvas.reset(new guik::GLCanvas(data_directory, Eigen::Vector2i(512, 512)));
}

ManualLoopCloseModal::~ManualLoopCloseModal() {}

bool ManualLoopCloseModal::set_begin_keyframe(int keyframe_id) {
  auto found = graph.keyframes.find(keyframe_id);
  if (found == graph.keyframes.end()) {
    return false;
  }

  begin_keyframe_pose = found->second->estimate();
  begin_keyframe = graph.keyframes_view_map[found->second];
  return true;
}

bool ManualLoopCloseModal::set_end_keyframe(int keyframe_id) {
  auto found = graph.keyframes.find(keyframe_id);
  if (found == graph.keyframes.end()) {
    return false;
  }

  end_keyframe_pose = found->second->estimate();
  end_keyframe_pose_init = end_keyframe_pose;
  end_keyframe = graph.keyframes_view_map[found->second];
  update_fitness_score();
  return true;
}

void ManualLoopCloseModal::close() {
  begin_keyframe = nullptr;
  end_keyframe = nullptr;
}

bool ManualLoopCloseModal::run() {
  bool close_window = false;
  if (ImGui::BeginPopupModal("manual loop close", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    if (begin_keyframe == nullptr || end_keyframe == nullptr) {
      if (begin_keyframe == nullptr) {
        ImGui::Text("begin_keyframe has not been set");
      }
      if (end_keyframe == nullptr) {
        ImGui::Text("begin_keyframe has not been set");
      }
    } else {
      // create OpenGL canvas
      ImGuiWindowFlags flags = ImGuiWindowFlags_ChildWindow | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNavFocus;
      ImGui::BeginChild("canvas", ImVec2(512, 512), false, flags);
      if (ImGui::IsWindowFocused()) {
        canvas->mouse_control();
      }

      draw_canvas();
      ImGui::Image((void*)canvas->frame_buffer->color().id(), ImVec2(512, 512), ImVec2(0, 1), ImVec2(1, 0));
      ImGui::EndChild();

      ImGui::Text("fitness_score:%.4f", fitness_score);

      if (ImGui::Button("Auto align")) {
        ImGui::OpenPopup("auto align");
      }
      auto_align();

      ImGui::SameLine();
      if (ImGui::Button("Scan matching")) {
        ImGui::OpenPopup("scan matching");
      }
      scan_matching();

      ImGui::SameLine();
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.05f);
      float px = 0.0f;
      if (ImGui::DragFloat("##PX", &px, 0.01, 0.0f, 0.0f, "PX")) {
        end_keyframe_pose.translation() += end_keyframe_pose.linear().block<3, 1>(0, 0) * px;
        update_fitness_score();
      }

      ImGui::SameLine();
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.05f);
      float py = 0.0f;
      if (ImGui::DragFloat("##PY", &py, 0.01, 0.0f, 0.0f, "PY")) {
        end_keyframe_pose.translation() += end_keyframe_pose.linear().block<3, 1>(0, 1) * py;
        update_fitness_score();
      }

      ImGui::SameLine();
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.05f);
      float pz = 0.0f;
      if (ImGui::DragFloat("##PZ", &pz, 0.01, 0.0f, 0.0f, "PZ")) {
        end_keyframe_pose.translation() += end_keyframe_pose.linear().block<3, 1>(0, 2) * pz;
        update_fitness_score();
      }

      ImGui::SameLine();
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.05f);
      float rx = 0.0f;
      if (ImGui::DragFloat("##RX", &rx, 0.01, 0.0f, 0.0f, "RX")) {
        end_keyframe_pose = end_keyframe_pose * Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
        update_fitness_score();
      }

      ImGui::SameLine();
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.05f);
      float ry = 0.0f;
      if (ImGui::DragFloat("##RY", &ry, 0.01, 0.0f, 0.0f, "RY")) {
        end_keyframe_pose = end_keyframe_pose * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY());
        update_fitness_score();
      }

      ImGui::SameLine();
      ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.05f);
      float rz = 0.0f;
      if (ImGui::DragFloat("##RZ", &rz, 0.01, 0.0f, 0.0f, "RZ")) {
        end_keyframe_pose = end_keyframe_pose * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
        update_fitness_score();
      }

      ImGui::SameLine();
      if (ImGui::Button("Reset")) {
        end_keyframe_pose = end_keyframe_pose_init;
        update_fitness_score();
      }

      if (ImGui::Button("Add edge")) {
        Eigen::Isometry3d relative = begin_keyframe_pose.inverse() * end_keyframe_pose;
        graph.add_edge(begin_keyframe->lock(), end_keyframe->lock(), relative);
        graph.optimize();

        ImGui::CloseCurrentPopup();
        begin_keyframe = nullptr;
        end_keyframe = nullptr;
        close_window = true;
      }
      ImGui::SameLine();
    }

    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
      begin_keyframe = nullptr;
      end_keyframe = nullptr;
      close_window = true;
    }

    ImGui::EndPopup();
  }
  return close_window;
}

void ManualLoopCloseModal::update_fitness_score() { fitness_score = InformationMatrixCalculator::calc_fitness_score(begin_keyframe->lock()->cloud, end_keyframe->lock()->cloud, begin_keyframe_pose.inverse() * end_keyframe_pose, 1.0); }

void ManualLoopCloseModal::auto_align() {
  if (ImGui::BeginPopupModal("auto align", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    const char* items[] = {"FPFH"};
    ImGui::Combo("Method", &registration_method, items, IM_ARRAYSIZE(items));

    // FPFH
    if (registration_method == 0) {
      ImGui::DragFloat("Normal estimation radius", &fpfh_normal_estimation_radius, 0.01f, 0.01f, 5.0f);
      ImGui::DragFloat("Search radius", &fpfh_search_radius, 0.01f, 0.01f, 5.0f);
      ImGui::DragInt("Max iterations", &fpfh_max_iterations, 1000, 1000, 100000);
      ImGui::DragInt("Num samples", &fpfh_num_samples, 1, 1, 100);
      ImGui::DragInt("Correspondence randomness", &fpfh_correspondence_randomness, 1, 1, 100);
      ImGui::DragFloat("Similarity threshold", &fpfh_similarity_threshold, 0.01f, 0.01f, 1.0f);
      ImGui::DragFloat("Max correspondence distance", &fpfh_max_correspondence_distance, 0.01f, 0.01f, 1.0f);
      ImGui::DragFloat("Inlier fraction", &fpfh_inlier_fraction, 0.01f, 0.01f, 1.0f);
    }

    if (ImGui::Button("OK")) {
      auto_alignment_progress = 0;
      auto_alignment_result = std::async(std::launch::async, [&]() {
        using FeatureT = pcl::FPFHSignature33;

        auto_alignment_progress = 1;
        pcl::PointCloud<pcl::PointNormal>::Ptr begin_keyframe_cloud(new pcl::PointCloud<pcl::PointNormal>());
        pcl::PointCloud<pcl::PointNormal>::Ptr end_keyframe_cloud(new pcl::PointCloud<pcl::PointNormal>());
        pcl::PointCloud<FeatureT>::Ptr begin_keyframe_features(new pcl::PointCloud<FeatureT>());
        pcl::PointCloud<FeatureT>::Ptr end_keyframe_features(new pcl::PointCloud<FeatureT>());

        pcl::copyPointCloud(*begin_keyframe->lock()->cloud, *begin_keyframe_cloud);
        pcl::copyPointCloud(*end_keyframe->lock()->cloud, *end_keyframe_cloud);

        auto_alignment_progress = 2;
        pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
        nest.setRadiusSearch(fpfh_normal_estimation_radius);
        nest.setInputCloud(begin_keyframe_cloud);
        nest.compute(*begin_keyframe_cloud);
        nest.setInputCloud(end_keyframe_cloud);
        nest.compute(*end_keyframe_cloud);

        auto_alignment_progress = 3;
        pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
        fest.setRadiusSearch(fpfh_search_radius);
        fest.setInputCloud(begin_keyframe_cloud);
        fest.setInputNormals(begin_keyframe_cloud);
        fest.compute(*begin_keyframe_features);
        fest.setInputCloud(end_keyframe_cloud);
        fest.setInputNormals(end_keyframe_cloud);
        fest.compute(*end_keyframe_features);

        auto_alignment_progress = 4;
        pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
        align.setInputSource(end_keyframe_cloud);
        align.setSourceFeatures(end_keyframe_features);
        align.setInputTarget(begin_keyframe_cloud);
        align.setTargetFeatures(begin_keyframe_features);

        align.setMaximumIterations(fpfh_max_iterations);
        align.setNumberOfSamples(fpfh_num_samples);
        align.setCorrespondenceRandomness(fpfh_correspondence_randomness);
        align.setSimilarityThreshold(fpfh_similarity_threshold);
        align.setMaxCorrespondenceDistance(fpfh_max_iterations);
        align.setInlierFraction(fpfh_inlier_fraction);

        pcl::PointCloud<pcl::PointNormal>::Ptr aligned(new pcl::PointCloud<pcl::PointNormal>());
        align.align(*aligned);

        Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
        relative.matrix() = align.getFinalTransformation().cast<double>();
        auto_alignment_progress = 5;

        usleep(500000);

        return std::make_shared<Eigen::Isometry3d>(relative);
      });

      ImGui::OpenPopup("progress");
    }

    bool close_parent = false;
    if (ImGui::BeginPopupModal("progress", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar)) {
      std::future_status result = auto_alignment_result.wait_for(std::chrono::milliseconds(100));
      const char* status_texts[] = {"Initializing", "Copy point clouds", "Normal estimation", "Feature extraction", "Sample consensus", "Ready"};
      ImGui::Text("%c %s", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3], status_texts[auto_alignment_progress]);

      float fraction = auto_alignment_progress / 5.0f;
      ImGui::ProgressBar(fraction, ImVec2(128, 16));

      if (result != std::future_status::timeout) {
        close_parent = true;
        end_keyframe_pose = begin_keyframe_pose * (*auto_alignment_result.get());
        update_fitness_score();
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      close_parent = true;
    }

    if (close_parent) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void ManualLoopCloseModal::scan_matching() {
  if (ImGui::BeginPopupModal("scan matching", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    const char* items[] = {"NDT", "GICP", "ICP"};
    ImGui::Combo("Method", &scan_matching_method, items, IM_ARRAYSIZE(items));
    ImGui::DragFloat("Resolution", &scan_matching_resolution, 0.1, 0.1f, 15.0f);

    if (ImGui::Button("OK")) {
      pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
      switch (scan_matching_method) {
        case 0: {
          auto ndt = boost::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
          ndt->setResolution(scan_matching_resolution);
          registration = ndt;
        } break;
        case 1:
          registration = boost::make_shared<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
          break;
        case 2:
          registration = boost::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
          break;
      }

      registration->setInputTarget(begin_keyframe->lock()->cloud);
      registration->setInputSource(end_keyframe->lock()->cloud);

      pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
      Eigen::Isometry3d relative = begin_keyframe_pose.inverse() * end_keyframe_pose;
      registration->align(*aligned, relative.matrix().cast<float>());

      relative.matrix() = registration->getFinalTransformation().cast<double>();
      end_keyframe_pose = begin_keyframe_pose * relative;

      update_fitness_score();
      ImGui::CloseCurrentPopup();
    }

    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void ManualLoopCloseModal::draw_gl(glk::GLSLShader& shader) {
  DrawFlags draw_flags;
  shader.set_uniform("point_scale", 100.0f);
  if (begin_keyframe) {
    begin_keyframe->draw(draw_flags, shader, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), begin_keyframe->lock()->estimate().matrix().cast<float>());
  }

  if (end_keyframe) {
    end_keyframe->draw(draw_flags, shader, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), end_keyframe->lock()->estimate().matrix().cast<float>());
  }
}

void ManualLoopCloseModal::draw_canvas() {
  if (!begin_keyframe || !end_keyframe) {
    return;
  }
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

  DrawFlags draw_flags;

  canvas->bind();
  canvas->shader->set_uniform("color_mode", 1);
  canvas->shader->set_uniform("point_scale", 100.0f);

  Eigen::Isometry3d relative = begin_keyframe_pose.inverse() * end_keyframe_pose;
  begin_keyframe->draw(draw_flags, *canvas->shader, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), Eigen::Matrix4f::Identity());
  end_keyframe->draw(draw_flags, *canvas->shader, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), relative.cast<float>().matrix());

  canvas->shader->set_uniform("color_mode", 1);
  canvas->shader->set_uniform("model_matrix", (relative * Eigen::UniformScaling<double>(3.0)).cast<float>().matrix());
  glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM).draw(*canvas->shader);
  canvas->unbind();

  glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
}
}  // namespace hdl_graph_slam