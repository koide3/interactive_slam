#include <hdl_graph_slam/interactive_keyframe.hpp>

#include <g2o/types/slam3d/vertex_se3.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>

namespace hdl_graph_slam {

InteractiveKeyFrame::InteractiveKeyFrame(const std::string& directory, g2o::HyperGraph* graph)
: KeyFrame(directory, graph)
{
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>());
  kdtree->setInputCloud(cloud);
  kdtree_ = kdtree;
}

InteractiveKeyFrame::~InteractiveKeyFrame() {

}

std::vector<int> InteractiveKeyFrame::neighbors(const Eigen::Vector3f& pt, double radius) {
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree = boost::any_cast<pcl::search::KdTree<pcl::PointXYZI>::Ptr>(kdtree_);

  std::vector<int> indices;
  std::vector<float> squared_distances;

  pcl::PointXYZI p;
  p.getVector4fMap() = node->estimate().inverse().cast<float>() * Eigen::Vector4f(pt[0], pt[1], pt[2], 1.0f);
  kdtree->radiusSearch(p, radius, indices, squared_distances);

  return indices;
}

pcl::PointCloud<pcl::Normal>::Ptr InteractiveKeyFrame::normals() {
  if (!normals_) {
    normals_.reset(new pcl::PointCloud<pcl::Normal>());

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.25f);

    ne.compute(*normals_);
  }

  return normals_;
}

}