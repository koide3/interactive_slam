#include <memory>
#include <imgui.h>
#include <imgui_internal.h>
#include <portable-file-dialogs.h>

#include <glk/lines.hpp>
#include <guik/gl_canvas.hpp>
#include <guik/progress_modal.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <hdl_graph_slam/parameter_server.hpp>
#include <hdl_graph_slam/version_modal.hpp>
#include <hdl_graph_slam/graph_edit_window.hpp>
#include <hdl_graph_slam/plane_detection_window.hpp>
#include <hdl_graph_slam/plane_alignment_modal.hpp>
#include <hdl_graph_slam/manual_loop_close_model.hpp>
#include <hdl_graph_slam/edge_refinement_window.hpp>
#include <hdl_graph_slam/automatic_loop_close_window.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

#include <ros/package.h>

namespace hdl_graph_slam {

class InteractiveSLAMApplication : public guik::Application {
public:
  InteractiveSLAMApplication() : Application() {}
  ~InteractiveSLAMApplication() {}

  /**
   * @brief initialize the application
   * @param window_name
   * @param size
   * @param glsl_version
   * @return true
   * @return false
   */
  bool init(const char* window_name, const Eigen::Vector2i& size, const char* glsl_version = "#version 330") override {
    if(!Application::init(window_name, size, glsl_version)) {
      return false;
    }

    show_imgui_demo = false;
    show_draw_config_window = false;

    right_clicked_pos.setZero();
    progress.reset(new guik::ProgressModal("progress modal"));

    // initialize the main OpenGL canvas
    std::string package_path = ros::package::getPath("interactive_slam");
    std::string data_directory = package_path + "/data";

    main_canvas.reset(new guik::GLCanvas(data_directory, framebuffer_size()));
    if(!main_canvas->ready()) {
      close();
    }

    // initialize the pose graph
    graph.reset(new InteractiveGraphView());
    graph->init_gl();

    // initialize sub-windows and modals
    version_modal.reset(new VersionModal());
    graph_edit_window.reset(new GraphEditWindow(graph));
    plane_detection_window.reset(new PlaneDetectionWindow(graph));
    plane_alignment_modal.reset(new PlaneAlignmentModal(graph));
    manual_loop_close_modal.reset(new ManualLoopCloseModal(graph, data_directory));
    automatic_loop_close_window.reset(new AutomaticLoopCloseWindow(graph));
    edge_refinement_window.reset(new EdgeRefinementWindow(graph));

    return true;
  }

  /**
   * @brief draw ImGui-based UI
   *
   */
  virtual void draw_ui() override {
    // draw main menu bar
    main_menu();

    // just for debug and development
    if(show_imgui_demo) {
      ImGui::ShowDemoWindow(&show_imgui_demo);
    }

    // show basic graph statistics and FPS
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground;
    ImGui::Begin("##stats", nullptr, flags);
    std::string graph_stats = graph->graph_statistics();
    ImGui::Text(graph_stats.c_str());
    ImGui::Text("\nFPS: %.3f fps", ImGui::GetIO().Framerate);
    ImGui::End();

    // show graph optimization progress
    if(graph->optimization_mutex.try_lock()) {
      graph->optimization_mutex.unlock();
    } else {
      ImGui::Begin("##optimization progress", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground);
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "%c %s", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3], "optimizing");

      std::string messages = graph->optimization_messages();
      std::stringstream sst(messages);

      // calculate the window width
      float max_line_length = 0;
      std::vector<std::string> lines;
      while(!sst.eof()) {
        std::string line;
        std::getline(sst, line);

        lines.push_back(line);
        max_line_length = std::max(max_line_length, ImGui::CalcTextSize(line.c_str()).x);
      }

      // optimization progress messages
      ImGui::BeginChild("##messages", ImVec2(max_line_length + 30.0f, ImGui::GetFontSize() * 32), true, ImGuiWindowFlags_AlwaysAutoResize);
      for(const auto& line: lines) {
        ImGui::Text(line.c_str());
      }
      ImGui::SetScrollHere();
      ImGui::EndChild();

      ImGui::End();
    }

    // draw windows
    main_canvas->draw_ui();
    graph_edit_window->draw_ui();
    plane_detection_window->draw_ui();
    automatic_loop_close_window->draw_ui();
    edge_refinement_window->draw_ui();

    draw_flags_config();
    context_menu();
    mouse_control();
  }

  /**
   * @brief draw OpenGL related stuffs on the main canvas
   *
   */
  virtual void draw_gl() override {
    if(graph->optimization_mutex.try_lock()) {
      glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

      main_canvas->bind();

      // draw coordinate system
      main_canvas->shader->set_uniform("color_mode", 2);
      main_canvas->shader->set_uniform("model_matrix", (Eigen::UniformScaling<float>(3.0f) * Eigen::Isometry3f::Identity()).matrix());
      const auto& coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
      coord.draw(*main_canvas->shader);

      // draw grid
      main_canvas->shader->set_uniform("color_mode", 1);
      main_canvas->shader->set_uniform("model_matrix", (Eigen::Translation3f(Eigen::Vector3f::UnitZ() * -0.02) * Eigen::Isometry3f::Identity()).matrix());
      main_canvas->shader->set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
      const auto& grid = glk::Primitives::instance()->primitive(glk::Primitives::GRID);
      grid.draw(*main_canvas->shader);

      // draw pose graph
      graph->draw(draw_flags, *main_canvas->shader);

      // let the windows draw something on the main canvas
      plane_detection_window->draw_gl(*main_canvas->shader);
      plane_alignment_modal->draw_gl(*main_canvas->shader);
      manual_loop_close_modal->draw_gl(*main_canvas->shader);
      automatic_loop_close_window->draw_gl(*main_canvas->shader);
      edge_refinement_window->draw_gl(*main_canvas->shader);

      // flush to the screen
      main_canvas->unbind();
      main_canvas->render_to_screen();

      glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
      graph->optimization_mutex.unlock();
    } else {
      // in case the optimization is going, show the last rendered image
      main_canvas->render_to_screen();
    }
  }

  /**
   * @brief frame buffer size change callback
   *
   * @param size
   */
  virtual void framebuffer_size_callback(const Eigen::Vector2i& size) override {
    main_canvas->set_size(size);
  }

private:
  /**
   * @brief show rendering setting switches
   *
   */
  void draw_flags_config() {
    if(!show_draw_config_window) {
      return;
    }

    ImGui::Begin("graph rendering config", &show_draw_config_window, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("General");
    ImGui::Checkbox("Vertices", &draw_flags.draw_verticies);
    ImGui::SameLine();
    ImGui::Checkbox("Edges", &draw_flags.draw_edges);

    ImGui::Text("Vertex");
    ImGui::Checkbox("Keyframes", &draw_flags.draw_keyframe_vertices);
    ImGui::SameLine();
    ImGui::Checkbox("Planes", &draw_flags.draw_plane_vertices);

    ImGui::Text("Edge");
    ImGui::Checkbox("SE3", &draw_flags.draw_se3_edges);
    ImGui::SameLine();
    ImGui::Checkbox("Plane", &draw_flags.draw_plane_edges);
    ImGui::SameLine();
    ImGui::Checkbox("SE3Plane", &draw_flags.draw_se3_plane_edges);
    ImGui::SameLine();
    ImGui::Checkbox("SE3Floor", &draw_flags.draw_floor_edges);

    ImGui::End();
  }

  /**
   * @brief draw main menu
   *
   */
  void main_menu() {
    ImGui::BeginMainMenuBar();

    /*** File menu ***/
    // flags to open dialogs
    // this trick is necessary to open ImGUI popup modals from the menubar
    bool open_map_dialog = false;
    bool merge_map_dialog = false;
    bool save_map_dialog = false;
    bool export_map_dialog = false;
    if(ImGui::BeginMenu("File")) {
      if(ImGui::BeginMenu("Open")) {
        if(ImGui::MenuItem("New map")) {
          open_map_dialog = true;
        }
        if(ImGui::MenuItem("Merge map")) {
          merge_map_dialog = true;
        }
        ImGui::EndMenu();
      }

      if(ImGui::BeginMenu("Save")) {
        if(ImGui::MenuItem("Save map data")) {
          save_map_dialog = true;
        }
        if(ImGui::MenuItem("Export PointCloud")) {
          export_map_dialog = true;
        }
        ImGui::EndMenu();
      }

      if(ImGui::MenuItem("Close Map")) {
        close_map_data();
      }

      if(ImGui::MenuItem("Quit")) {
        close();
      }

      ImGui::EndMenu();
    }

    // open dialogs
    open_map_data(open_map_dialog);
    merge_map_data(merge_map_dialog);
    save_map_data(save_map_dialog);
    export_pointcloud(export_map_dialog);

    /*** View menu ***/
    if(ImGui::BeginMenu("View")) {
      if(ImGui::MenuItem("Reset camera")) {
        main_canvas->reset_camera();
      }
      if(ImGui::MenuItem("Projection setting")) {
        main_canvas->show_projection_setting();
      }
      if(ImGui::MenuItem("Graph rendering setting")) {
        show_draw_config_window = true;
      }
      if(ImGui::MenuItem("Clear selections")) {
        clear_selections();
      }

      ImGui::EndMenu();
    }

    /*** Graph menu ***/
    if(ImGui::BeginMenu("Graph")) {
      if(ImGui::MenuItem("Graph editor")) {
        graph_edit_window->show();
      }

      if(ImGui::MenuItem("Automatic loop detection")) {
        clear_selections();
        automatic_loop_close_window->show();
      }

      if(ImGui::MenuItem("Edge refinement")) {
        clear_selections();
        edge_refinement_window->show();
      }

      if(ImGui::MenuItem("Optimize")) {
        graph->optimize_background(128);
      }

      ImGui::EndMenu();
    }

    /*** Help menu ***/
    bool show_version = false;
    if(ImGui::BeginMenu("Help")) {
      if(ImGui::MenuItem("ImGuiDemo")) {
        show_imgui_demo = true;
      }

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
   * @brief make sure that all the windows and modals are closed (to avoid confliction of optimization threads)
   */
  void clear_selections() {
    manual_loop_close_modal->close();
    automatic_loop_close_window->close();
    edge_refinement_window->close();
    plane_detection_window->close();
    plane_alignment_modal->close();
  }

  /**
   * @brief open map data
   * @param open_dialog
   */
  void open_map_data(bool open_dialog) {
    // show the progress modal until loading will be finished
    if(progress->run("graph load")) {
      auto result = progress->result<std::shared_ptr<InteractiveGraphView>>();
      if(result == nullptr) {
        pfd::message message("Error", "failed to load graph data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }

      // initialize OpenGL stuffs in this main thread
      result->init_gl();
      graph = result;
    }

    if(!open_dialog) {
      return;
    }
    pfd::select_folder dialog("choose graph directory");
    while(!dialog.ready()) {
      usleep(100);
    }

    std::string result = dialog.result();
    if(result.empty()) {
      return;
    }

    clear_selections();
    if(graph->num_vertices() != 0) {
      pfd::message dialog("Confirm", "The current map data will be closed, and unsaved data will be lost.\nDo you want to continue?");
      while(!dialog.ready()) {
        usleep(100);
      }

      if(dialog.result() != pfd::button::ok) {
        return;
      }
    }

    std::string input_graph_filename = result;

    // open the progress modal and load the graph in a background thread
    progress->open<std::shared_ptr<InteractiveGraphView>>("graph load", [=](guik::ProgressInterface& p) {
      std::shared_ptr<InteractiveGraphView> graph(new InteractiveGraphView());
      if(!graph->load_map_data(input_graph_filename, p)) {
        return std::shared_ptr<InteractiveGraphView>();
      }
      return graph;
    });
  }

  /**
   * @brief merge map data
   * @param open_dialog
   */
  void merge_map_data(bool open_dialog) {
    // show the progress modal
    if(progress->run("graph merge")) {
      auto result = progress->result<InteractiveGraph*>();
      if(result == nullptr) {
        pfd::message message("Error", "failed to load graph data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }

      // TODO: automatic multiple map alignment
      Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
      relative.translation() = Eigen::Vector3d(0.0, 10.0, 0.0);
      graph->merge_map_data(*result, graph->keyframes[0], result->keyframes[0], relative);
    }

    if(!open_dialog) {
      return;
    }
    pfd::select_folder dialog("choose graph directory");
    while(!dialog.ready()) {
      usleep(100);
    }

    std::string result = dialog.result();
    if(result.empty()) {
      return;
    }

    if(graph->num_vertices() == 0) {
      pfd::message dialog("Error", "The current graph is empty!!");
      while(!dialog.ready()) {
        usleep(100);
      }
      return;
    }

    clear_selections();
    std::string input_graph_filename = result;
    progress->open<InteractiveGraph*>("graph merge", [=](guik::ProgressInterface& p) {
      InteractiveGraph* graph = new InteractiveGraph();
      if(!graph->load_map_data(input_graph_filename, p)) {
        delete graph;
        graph = nullptr;
        return graph;
      }
      return graph;
    });
  }

  /**
   * @brief save map data
   * @param save_map_dialog
   */
  void save_map_data(bool save_map_dialog) {
    if(progress->run("graph save")) {
      bool result = progress->result<bool>();
      if(!result) {
        pfd::message message("Error", "failed to save graph data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }
    }

    if(!save_map_dialog) {
      return;
    }

    std::unique_ptr<pfd::select_folder> dialog(new pfd::select_folder("choose destination"));
    while(!dialog->ready()) {
      usleep(100);
    }

    auto result = dialog->result();
    if(result.empty()) {
      return;
    }

    clear_selections();
    progress->open<bool>("graph save", [=](guik::ProgressInterface& p) {
      graph->dump(result, p);

      p.set_text("done");
      usleep(500000);

      return true;
    });
  }

  /**
   * @brief export point cloud
   * @param export_map_dialog
   */
  void export_pointcloud(bool export_map_dialog) {
    if(progress->run("graph export")) {
      int result = progress->result<int>();
      if(result) {
        pfd::message message("Error", "failed to export graph data", pfd::choice::ok);
        while(!message.ready()) {
          usleep(100);
        }
        return;
      }
    }

    if(!export_map_dialog) {
      return;
    }

    std::vector<std::string> filters = {"Point cloud file (.pcd)", "*.pcd"};
    std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file", "", filters));
    while(!dialog->ready()) {
      usleep(100);
    }

    auto result = dialog->result();
    if(result.empty()) {
      return;
    }

    clear_selections();
    progress->open<int>("graph export", [=](guik::ProgressInterface& p) { return graph->save_pointcloud(result, p); });
  }

  /**
   * @brief close map data
   */
  void close_map_data() {
    std::unique_ptr<pfd::message> dialog(new pfd::message("confirm", "Do you want to close the map file?"));
    while(!dialog->ready()) {
      usleep(100);
    }

    if(dialog->result() != pfd::button::ok) {
      return;
    }

    clear_selections();
    graph.reset(new InteractiveGraphView());
    graph->init_gl();
  }

  /**
   * @brief handling mouse input
   */
  void mouse_control() {
    ImGuiIO& io = ImGui::GetIO();
    if(!io.WantCaptureMouse) {
      // let the main canvas handle the mouse input
      main_canvas->mouse_control();

      // remember the right click position
      if(ImGui::IsMouseClicked(1)) {
        auto mouse_pos = ImGui::GetMousePos();
        right_clicked_pos = Eigen::Vector2i(mouse_pos.x, mouse_pos.y);
      }
    }
  }

  /**
   * @brief context menu
   */
  void context_menu() {
    if(ImGui::BeginPopupContextVoid("context menu")) {
      // pickup information of the right clicked object
      Eigen::Vector4i picked_info = main_canvas->pick_info(right_clicked_pos);
      int picked_type = picked_info[0];   // object type (point, vertex, edge, etc...)
      int picked_id = picked_info[1];     // object ID

      // calculate the 3D position of the right clicked pixel
      float depth = main_canvas->pick_depth(right_clicked_pos);
      Eigen::Vector3f pos_3d = main_canvas->unproject(right_clicked_pos, depth);

      // map point
      if(picked_type & DrawableObject::POINTS) {
        ImGui::Text("Map point");
        ImGui::Text("Pos: %.3f %.3f %.3f", pos_3d[0], pos_3d[1], pos_3d[2]);

        if(ImGui::Button("Plane detection")) {
          clear_selections();
          plane_detection_window->set_center_point(pos_3d);
          plane_detection_window->show();
          ImGui::CloseCurrentPopup();
        }
      }

      // vertex object
      if(picked_type & DrawableObject::VERTEX) {
        ImGui::Text("Vertex %d", picked_id);
        ImGui::Text("Pos: %.3f %.3f %.3f", pos_3d[0], pos_3d[1], pos_3d[2]);

        auto found = graph->vertices_view_map.find(picked_id);
        if(found != graph->vertices_view_map.end()) {
          found->second->context_menu();
        }
      }

      // edge object
      if(picked_type & DrawableObject::EDGE) {
        ImGui::Text("Edge %d", picked_id);
        ImGui::Text("Pos: %.3f %.3f %.3f", pos_3d[0], pos_3d[1], pos_3d[2]);

        for(auto& edge : graph->edges_view) {
          if(edge->id() == picked_id) {
            bool do_delete = false;
            edge->context_menu(do_delete);

            if(do_delete) {
              clear_selections();
              graph->delete_edge(edge);
              ImGui::CloseCurrentPopup();
            }

            break;
          }
        }
      }

      // keyframe object
      if(picked_type & DrawableObject::KEYFRAME) {
        ImGui::Text("\nKeyframe");
        if(ImGui::Button("Loop begin")) {
          clear_selections();
          manual_loop_close_modal->set_begin_keyframe(picked_id);
          ImGui::CloseCurrentPopup();
        }
        if(!manual_loop_close_modal->has_begin_keyframe()){
          ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
          ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        if(ImGui::Button("Loop end")) {
          manual_loop_close_modal->set_end_keyframe(picked_id);
          ImGui::OpenPopup("manual loop close");
        }
        if(!manual_loop_close_modal->has_begin_keyframe()){
          ImGui::PopItemFlag();
          ImGui::PopStyleVar();
        }
      }

      // plane vertex
      if(picked_type & DrawableObject::PLANE) {
        ImGui::Text("\nPlane correction");
        if(ImGui::Button("Loop begin")) {
          clear_selections();
          plane_alignment_modal->set_begin_plane(picked_id);
          ImGui::CloseCurrentPopup();
        }
        if(!plane_alignment_modal->has_begin_plane()){
          ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
          ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        }
        if(ImGui::Button("Loop end")) {
          plane_alignment_modal->set_end_plane(picked_id);
          ImGui::OpenPopup("plane alignment");
        }
        if(!plane_alignment_modal->has_begin_plane()){
          ImGui::PopItemFlag();
          ImGui::PopStyleVar();
        }

        if(ImGui::BeginMenu("Prior")) {
          if(ImGui::MenuItem("Ground(UnitZ, Zero)")) {
            graph->add_edge_prior_normal(picked_id, Eigen::Vector3d::UnitZ(), 1000.0);
            graph->add_edge_prior_distance(picked_id, 0.0, 1000.0);
            graph->optimize_background(128);
          }

          if(ImGui::MenuItem("Normal:UnitX")) {
            graph->add_edge_prior_normal(picked_id, Eigen::Vector3d::UnitX(), 1000.0);
            graph->optimize_background();
          }
          if(ImGui::MenuItem("Normal:UnitY")) {
            graph->add_edge_prior_normal(picked_id, Eigen::Vector3d::UnitY(), 1000.0);
            graph->optimize_background();
          }
          if(ImGui::MenuItem("Normal:UnitZ")) {
            graph->add_edge_prior_normal(picked_id, Eigen::Vector3d::UnitZ(), 1000.0);
            graph->optimize_background();
          }
          if(ImGui::MenuItem("Distance:Zero")) {
            graph->add_edge_prior_distance(picked_id, 0.0, 1000.0);
            graph->optimize_background();
          }

          ImGui::EndMenu();
        }
      }

      if(plane_alignment_modal->run() || manual_loop_close_modal->run()) {
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndPopup();
    }
  }

private:
  DrawFlags draw_flags;
  Eigen::Vector2i right_clicked_pos;

  std::shared_ptr<InteractiveGraphView> graph;

  std::unique_ptr<guik::GLCanvas> main_canvas;
  std::unique_ptr<guik::ProgressModal> progress;

  bool show_imgui_demo;
  bool show_draw_config_window;
  std::unique_ptr<VersionModal> version_modal;
  std::unique_ptr<GraphEditWindow> graph_edit_window;
  std::unique_ptr<PlaneDetectionWindow> plane_detection_window;
  std::unique_ptr<PlaneAlignmentModal> plane_alignment_modal;
  std::unique_ptr<ManualLoopCloseModal> manual_loop_close_modal;
  std::unique_ptr<AutomaticLoopCloseWindow> automatic_loop_close_window;
  std::unique_ptr<EdgeRefinementWindow> edge_refinement_window;
};

}  // namespace hdl_graph_slam

int main(int argc, char** argv) {
  std::unique_ptr<guik::Application> app(new hdl_graph_slam::InteractiveSLAMApplication());

  if(!app->init("Interactive SLAM", Eigen::Vector2i(1920, 1080))) {
    return 1;
  }

  app->run();

  return 0;
}