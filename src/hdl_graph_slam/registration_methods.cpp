#include <hdl_graph_slam/registration_methods.hpp>

#include <imgui.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

namespace hdl_graph_slam {

RegistrationMethods::RegistrationMethods() : registration_method(1), registration_resolution(2.0f), transformation_epsilon(1e-4), max_iterations(64), registration_methods({"ICP", "GICP", "NDT", "GICP_OMP", "NDT_OMP"}) {}

RegistrationMethods::~RegistrationMethods() {}

void RegistrationMethods::draw_ui() {
  ImGui::Text("Scan matching");
  ImGui::Combo("Method", &registration_method, registration_methods.data(), registration_methods.size());
  if(std::string(registration_methods[registration_method]).find("NDT") != std::string::npos) {
    ImGui::DragFloat("Resolution", &registration_resolution, 0.1f, 0.1f, 20.0f);
  }
  ImGui::DragFloat("Transformation epsilon", &transformation_epsilon, 1e-5f, 1e-5f, 1e-2f, "%.6f");
  ImGui::DragInt("Max iterations", &max_iterations, 1, 1, 256);
}

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr RegistrationMethods::method() const {
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

  switch(registration_method) {
    case 0: {
      auto icp = boost::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
      registration = icp;
    } break;

    default:
      std::cerr << "warning: unknown registration method!!" << std::endl;
      std::cerr << "       : use GICP" << std::endl;
    case 1: {
      auto gicp = boost::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
      registration = gicp;
    } break;

    case 2: {
      auto ndt = boost::make_shared<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
      ndt->setResolution(registration_resolution);
      registration = ndt;
    } break;

    case 3: {
      auto gicp = boost::make_shared<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>();
      registration = gicp;
    } break;

    case 4: {
      auto ndt = boost::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>();
      ndt->setResolution(registration_resolution);
      registration = ndt;
    } break;
  }

  registration->setTransformationEpsilon(transformation_epsilon);
  registration->setMaximumIterations(max_iterations);

  return registration;
}
}  // namespace hdl_graph_slam