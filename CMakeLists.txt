cmake_minimum_required(VERSION 2.8.3)
project(interactive_slam)

# add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS roscpp ndt_omp hdl_graph_slam)

find_package(GLM REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

##################
# catking config #
##################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interactive_map_correction
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  thirdparty/gl3w
  thirdparty/imgui
  thirdparty/imgui/examples
  thirdparty/portable-file-dialogs
  ${GLM_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# imgui
add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

###########
## Build ##
###########

add_library(imgui STATIC
  thirdparty/gl3w/gl3w.c
  thirdparty/imgui/imgui.cpp
  thirdparty/imgui/imgui_demo.cpp
  thirdparty/imgui/imgui_draw.cpp
  thirdparty/imgui/imgui_widgets.cpp
  thirdparty/imgui/examples/imgui_impl_glfw.cpp
  thirdparty/imgui/examples/imgui_impl_opengl3.cpp
)
target_link_libraries(imgui
  ${OPENGL_LIBRARIES}
  glfw
  dl
)

add_library(guik STATIC
  src/glk/mesh.cpp
  src/glk/lines.cpp
  src/glk/colormap.cpp
  src/glk/glsl_shader.cpp
  src/glk/frame_buffer.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/primitives/primitives.cpp
  src/glk/loaders/ply_loader.cpp
  src/guik/gl_canvas.cpp
  src/guik/model_control.cpp
  src/guik/camera_control.cpp
  src/guik/projection_control.cpp
  src/guik/imgui_application.cpp
)

add_executable(odometry2graph
  src/odometry2graph.cpp
  src/hdl_graph_slam/version_modal.cpp
)
target_link_libraries(odometry2graph
  ${PCL_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${catkin_LIBRARIES}
  glfw
  guik
  imgui
)

add_executable(interactive_slam
  src/interactive_slam.cpp
  src/hdl_graph_slam/interactive_graph.cpp
  src/hdl_graph_slam/interactive_keyframe.cpp
  src/hdl_graph_slam/version_modal.cpp
  src/hdl_graph_slam/graph_edit_window.cpp
  src/hdl_graph_slam/edge_refinement_window.cpp
  src/hdl_graph_slam/plane_detection_window.cpp
  src/hdl_graph_slam/plane_alignment_modal.cpp
  src/hdl_graph_slam/manual_loop_close_modal.cpp
  src/hdl_graph_slam/automatic_loop_close_window.cpp
  src/hdl_graph_slam/view/edge_view.cpp
  src/hdl_graph_slam/view/vertex_view.cpp
)
target_link_libraries(interactive_slam
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  glfw
  guik
  imgui
  hdl_graph_slam_nodelet
)

file(COPY data DESTINATION .)