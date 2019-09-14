#include <memory>
#include <imgui.h>
#include <portable-file-dialogs.h>

#include <glk/lines.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <hdl_graph_slam/parameter_server.hpp>
#include <hdl_graph_slam/plane_detection_modal.hpp>
#include <hdl_graph_slam/manual_loop_close_model.hpp>
#include <hdl_graph_slam/automatic_loop_close_window.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

#include <ros/package.h>

namespace hdl_graph_slam {

class InteractiveMapCorrectionApplication : public guik::Application {
public:
  InteractiveMapCorrectionApplication() : Application() {}

  bool init(const Eigen::Vector2i& size, const char* glsl_version = "#version 330") override {
    if (!Application::init(size, glsl_version)) {
      return false;
    }

    right_clicked_pos.setZero();

    std::string package_path = ros::package::getPath("interactive_map_correction");
    std::string data_directory = package_path + "/data";

    main_canvas.reset(new guik::GLCanvas(data_directory, Eigen::Vector2i(1920, 1080)));
    if (!main_canvas->ready()) {
      close();
    }

    graph.reset(new InteractiveGraphView());
    plane_detection_modal.reset(new PlaneDetectionModal(*graph));
    manual_loop_close_modal.reset(new ManualLoopCloseModal(*graph, data_directory));
    automatic_loop_close_window.reset(new AutomaticLoopCloseWindow(*graph, data_directory));

    return true;
  }

  virtual void draw_ui() override {
    main_menu();

    ImGui::ShowDemoWindow();
    ImGuiIO& io = ImGui::GetIO();

    {
      std::lock_guard<std::mutex> lock(graph->optimization_mutex);

      ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground;
      ImGui::Begin("##stats", nullptr, flags);
      ImGui::Text("Graph");
      ImGui::Text("# vertices: %d", graph->num_vertices());
      ImGui::Text("# edges: %d", graph->num_edges());
      ImGui::Text("time: %.1f[msec]", graph->elapsed_time_msec);
      ImGui::Text("chi2: %.3f -> %.3f", graph->chi2_before, graph->chi2_after);
      ImGui::Text("iterations: %d", graph->iterations);

      ImGui::Text("\nFPS: %.3f fps", io.Framerate);
      ImGui::End();
    }

    main_canvas->draw_ui();

    plane_detection_modal->draw_ui();
    automatic_loop_close_window->draw_ui();

    draw_flags_config();
    context_menu();
    mouse_control();
  }

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
    {
      std::lock_guard<std::mutex> lock(graph->optimization_mutex);
      graph->draw(draw_flags, *main_canvas->shader);
    }

    plane_detection_modal->draw_gl(*main_canvas->shader);
    manual_loop_close_modal->draw_gl(*main_canvas->shader);
    automatic_loop_close_window->draw_gl(*main_canvas->shader);

    main_canvas->unbind();
    main_canvas->render_to_screen();

    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
  }

private:
  void draw_flags_config() {
    if (!show_draw_config_window) {
      return;
    }

    ImGui::Begin("graph rendering config", &show_draw_config_window, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("General");
    ImGui::Checkbox("Vertices", &draw_flags.draw_verticies);
    ImGui::SameLine();
    ImGui::Checkbox("Edges", &draw_flags.draw_edges);

    ImGui::Text("Vertices");
    ImGui::Checkbox("Keyframes", &draw_flags.draw_keyframe_vertices);
    ImGui::SameLine();
    ImGui::Checkbox("Planes", &draw_flags.draw_plane_vertices);

    ImGui::Text("Edges");
    ImGui::Checkbox("SE3", &draw_flags.draw_se3_edges);
    ImGui::SameLine();
    ImGui::Checkbox("SE3Planes", &draw_flags.draw_se3_plane_edges);
    ImGui::SameLine();
    ImGui::Checkbox("SE3Floor", &draw_flags.draw_floor_edges);

    ImGui::End();
  }

  void main_menu() {
    ImGui::BeginMainMenuBar();

    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Open..")) {
        open_map_data();
      }

      if (ImGui::MenuItem("Save Map Data")) {
        save_map_data();
      }

      if (ImGui::MenuItem("Export PointCloud")) {
        export_pointcloud();
      }

      if (ImGui::MenuItem("Quit")) {
        close();
      }

      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Projection setting")) {
        main_canvas->show_projection_setting();
      }
      if (ImGui::MenuItem("Graph rendering setting")) {
        show_draw_config_window = true;
      }

      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Graph")) {
      if (ImGui::MenuItem("Automatic loop detection")) {
        automatic_loop_close_window->show();
      }

      if (ImGui::MenuItem("Optimize")) {
        graph->optimize();
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

private:
  void open_map_data() {
    pfd::select_folder dialog("choose graph directory");
    while (!dialog.ready()) {
      usleep(10);
    }

    auto result = dialog.result();
    if (result.empty()) {
      return;
    }

    if (!graph->load_map_data(result)) {
      pfd::message notify("Error", "failed to load graph data", pfd::choice::ok);
      while (!notify.ready()) {
        usleep(10);
      }
      return;
    }

    graph->optimize();
  }

  void save_map_data() {
    std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file"));
    while (!dialog->ready()) {
      usleep(10);
    }

    auto result = dialog->result();
    std::cout << result << std::endl;
  }

  void export_pointcloud() {
    std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file"));
    while (!dialog->ready()) {
      usleep(10);
    }

    auto result = dialog->result();

    std::lock_guard<std::mutex> lock(graph->optimization_mutex);
    graph->save_pointcloud(result);
  }

  void mouse_control() {
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureMouse) {
      main_canvas->mouse_control();

      if (ImGui::IsMouseClicked(1)) {
        auto mouse_pos = ImGui::GetMousePos();
        right_clicked_pos = Eigen::Vector2i(mouse_pos.x, mouse_pos.y);
      }
    }
  }

  void context_menu() {
    if (ImGui::BeginPopupContextVoid("context menu")) {
      auto mouse_pos = ImGui::GetMousePos();
      Eigen::Vector4i picked_info = main_canvas->pick_info(right_clicked_pos);
      int picked_type = picked_info[0];
      int picked_id = picked_info[1];

      float depth = main_canvas->pick_depth(right_clicked_pos);
      Eigen::Vector3f pos_3d = main_canvas->unproject(right_clicked_pos, depth);

      if (picked_type == DrawableObject::POINTS) {
        ImGui::Text("Map point");
        ImGui::Text("Pos: %.3f %.3f %.3f", pos_3d[0], pos_3d[1], pos_3d[2]);

        if (ImGui::Button("Plane detection")) {
          plane_detection_modal->set_center_point(pos_3d);
          plane_detection_modal->show();
          ImGui::CloseCurrentPopup();
        }
      }

      if (picked_type == DrawableObject::KEYFRAME) {
        ImGui::Text("Keyframe %d", picked_id);
        ImGui::Text("Pos: %.3f %.3f %.3f", pos_3d[0], pos_3d[1], pos_3d[2]);

        ImGui::Text("\nLoop close");
        if (ImGui::Button("Loop begin")) {
          manual_loop_close_modal->set_begin_keyframe(picked_id);
          ImGui::CloseCurrentPopup();
        }
        if (ImGui::Button("Loop end")) {
          manual_loop_close_modal->set_end_keyframe(picked_id);
          ImGui::OpenPopup("manual loop close");
        }
      }

      if (picked_type == DrawableObject::VERTEX) {
        ImGui::Text("Vertex %d", picked_id);
        ImGui::Text("Pos: %.3f %.3f %.3f", pos_3d[0], pos_3d[1], pos_3d[2]);
      }

      if (picked_type == DrawableObject::EDGE) {
        ImGui::Text("Edge %d", picked_id);
        ImGui::Text("Pos: %.3f %.3f %.3f", pos_3d[0], pos_3d[1], pos_3d[2]);
      }

      if (manual_loop_close_modal->run()) {
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndPopup();
    }
  }

private:
  Eigen::Vector2i right_clicked_pos;
  std::unique_ptr<PlaneDetectionModal> plane_detection_modal;
  std::unique_ptr<ManualLoopCloseModal> manual_loop_close_modal;
  std::unique_ptr<AutomaticLoopCloseWindow> automatic_loop_close_window;

  bool show_draw_config_window;
  DrawFlags draw_flags;
  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<InteractiveGraphView> graph;
};

}  // namespace hdl_graph_slam

int main(int argc, char** argv) {
  std::unique_ptr<guik::Application> app(new hdl_graph_slam::InteractiveMapCorrectionApplication());

  if (!app->init(Eigen::Vector2i(1920, 1080))) {
    return 1;
  }

  app->run();

  return 0;
}