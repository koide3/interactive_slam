#include <hdl_graph_slam/version.h>
#include <hdl_graph_slam/version_modal.hpp>

namespace hdl_graph_slam {

VersionModal::VersionModal() {}

VersionModal ::~VersionModal() {}

void VersionModal::open() {
  ImGui::OpenPopup("Interactive SLAM");
}

bool VersionModal::run() {
  bool close_modal = false;
  ImGuiWindowFlags flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar;
  if(ImGui::BeginPopupModal("Interactive SLAM", nullptr, flags)) {
    ImGui::Text("Interactive SLAM");
    ImGui::Text("Version: %d.%d.%d", IS_VERSION_MAJOR, IS_VERSION_MINOR, IS_VERSION_REVISION);
    ImGui::Text("Commit hash: %s", LAST_COMMIT_HASH);
    ImGui::Text("Commit date: %s", LAST_COMMIT_DATE);
    ImGui::Text("\nK.Koide (k.koide@aist.go.jp)");
    ImGui::Text("Smart Mobility Research Team");
    ImGui::Text("AIST, Japan");

    if(ImGui::Button("OK")) {
      ImGui::CloseCurrentPopup();
      close_modal = true;
    }
    ImGui::EndPopup();
  }

  return close_modal;
}
}