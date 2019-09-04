#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <nanogui/nanogui.h>

#include <guik/pointcloud_canvas.hpp>


class Application : public nanogui::Screen {
public:
    Application() : nanogui::Screen(Eigen::Vector2i(1920, 1080), "viewer", false) {
        // toolbox
        nanogui::Window* window = new nanogui::Window(this, "tools");
        window->setLayout(new nanogui::GroupLayout());

        nanogui::Widget* tools = new nanogui::Widget(window);
        tools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Middle, 0, 5));
        nanogui::Button* open_button = new nanogui::Button(tools, "open");

        open_button->setCallback([this]() { open_cloud(); } );

        nanogui::Button* close_button = new nanogui::Button(tools, "close");
        close_button->setCallback([this]() { setVisible(false); } );

        // point cloud viewer
        nanogui::Window* viewer = new nanogui::Window(this, "main screen");
        viewer->setLayout(new nanogui::GroupLayout());

        canvas = new guik::PointCloudCanvas(viewer, "data/shader/rainbow");
        canvas->setSize({1024, 1024});
        viewer->center();

        context_menu = new nanogui::Window(this, "");
        context_menu->setLayout(new nanogui::GroupLayout());

        nanogui::Widget* context_menu_buttons = new nanogui::Widget(context_menu);
        context_menu_buttons->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Middle, 0, 5));
        nanogui::PopupButton* loop_closure = new nanogui::PopupButton(context_menu_buttons, "loop closure");
        loop_closure->popup()->setLayout(new nanogui::GroupLayout());
        (new nanogui::Button(loop_closure->popup(), "loop begin"));
        (new nanogui::Button(loop_closure->popup(), "loop end"));
        (new nanogui::Button(loop_closure->popup(), "align"));
        (new nanogui::Button(loop_closure->popup(), "add loop edge"));

        (new nanogui::Button(context_menu_buttons, "close"))->setCallback([this]() { context_menu->setVisible(false); });
        context_menu->setVisible(false);

        performLayout();
    }

    virtual void draw(NVGcontext* context) override {
        Screen::draw(context);
    }

    virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override {
        bool result = Screen::mouseButtonEvent(p, button, down, modifiers);

        /*
        if(button == GLFW_MOUSE_BUTTON_RIGHT && !down) {
            context_menu->setPosition(p);
            context_menu->setVisible(true);
        } else if(!context_menu->focused()) {
            context_menu->setVisible(false);
        }
        */

        return result;
    }

    void open_cloud() {
        std::vector<std::pair<std::string, std::string>> extensions = { std::make_pair("pcd", "poind cloud") };
        std::string pcd_filename = nanogui::file_dialog(extensions, false);

        if(pcd_filename.empty()) {
            std::cout << "no file" << std::endl;
            return;
        }

        std::cout << pcd_filename << std::endl;
        canvas->add_cloud(std::make_shared<glk::PointCloudBuffer>(pcd_filename));
    }

private:
    guik::PointCloudCanvas* canvas;

    nanogui::Window* context_menu;
};


int main(int argc, char** argv) {
    nanogui::init();

    nanogui::ref<Application> app = new Application();
    app->drawAll();
    app->setVisible(true);

    nanogui::mainloop();

    nanogui::shutdown();

    return 0;
}