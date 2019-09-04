#include <memory>
#include <imgui.h>
#include <imgui_internal.h>
#include <portable-file-dialogs.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <hdl_graph_slam/parameter_server.hpp>
#include <hdl_graph_slam/view/interactive_graph_view.hpp>

#include <ros/package.h>


class InteractiveMapCorrectionApplication : public guik::Application {
public:
    InteractiveMapCorrectionApplication()
    : Application()
    {}

    bool init() override{
      if(!Application::init()) {
        return false;
      }

      std::string package_path = ros::package::getPath("interactive_map_correction");
      std::string data_directory = package_path + "/data";

      shader.reset(new glk::GLSLShader());
      if(!shader->init(data_directory + "/shader/rainbow")) {
        glfwWindowShouldClose(window);
      }

      camera_control.reset(new guik::ArcCameraControl());
      graph.reset(new hdl_graph_slam::InteractiveGraphView());

      return true;
    }

    virtual void draw_ui() override {
        main_menu();

    }

    virtual void draw_gl() override {
      shader->use();

      Eigen::Matrix4f view_matrix = camera_control->view_matrix();
      glm::mat4 proj = glm::perspective<float>(120.0, 1.0, 1.0, 500.0);
      Eigen::Matrix4f projection_matrix = Eigen::Map<Eigen::Matrix4f>(glm::value_ptr(proj));

      shader->set_uniform("view_matrix", view_matrix);
      shader->set_uniform("projection_matrix", projection_matrix);

      glEnable(GL_DEPTH_TEST);
      graph->draw(*shader);
      glDisable(GL_DEPTH_TEST);


    }

private:
    void main_menu() {
        ImGui::BeginMainMenuBar();

        if(ImGui::BeginMenu("File")) {
            if(ImGui::MenuItem("Open..")) {
                open_map_data();
            }

            if(ImGui::MenuItem("Save Map Data")) {
                save_map_data();
            }

            if(ImGui::MenuItem("Export PointCloud")) {
                export_pointcloud();
            }

            if(ImGui::MenuItem("Quit")) {
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }

            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

private:
    void open_map_data() {
        pfd::select_folder dialog("choose graph directory");
        while(!dialog.ready()) {
            usleep(10);
        }

        auto result = dialog.result();
        if(result.empty()) {
            return;
        }

        if(!graph->load(result)) {
            pfd::message notify("Error", "failed to load graph data", pfd::choice::ok);
            while(!notify.ready()) {
                usleep(10);
            }
            return;
        }
    }

    void save_map_data() {
        std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file"));
        while(!dialog->ready()) {
            usleep(10);
        }

        auto result = dialog->result();
        std::cout << result << std::endl;
    }

    void export_pointcloud() {
        std::unique_ptr<pfd::save_file> dialog(new pfd::save_file("choose file"));
        while(!dialog->ready()) {
            usleep(10);
        }

        auto result = dialog->result();
        std::cout << result << std::endl;
    }


private:
    std::unique_ptr<glk::GLSLShader> shader;
    std::unique_ptr<guik::CameraControl> camera_control;
    std::unique_ptr<hdl_graph_slam::InteractiveGraphView> graph;
};

int main(int argc, char** argv) {
    std::unique_ptr<guik::Application> app(new InteractiveMapCorrectionApplication());

    if(!app->init()) {
        return 1;
    }

    app->run();

    return 0;
}