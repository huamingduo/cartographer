#include "cartographer/mapping/internal/2d/scan_matching/ndt_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::NDTScanMatcherOptions2D CreateNDTScanMatcherOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::NDTScanMatcherOptions2D options;
//   options.set_occupied_space_weight(
//       parameter_dictionary->GetDouble("occupied_space_weight"));
//   options.set_translation_weight(
//       parameter_dictionary->GetDouble("translation_weight"));
//   options.set_rotation_weight(
//       parameter_dictionary->GetDouble("rotation_weight"));
//   *options.mutable_ceres_solver_options() =
//       common::CreateCeresSolverOptionsProto(
//           parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

NDTScanMatcher2D::NDTScanMatcher2D(
    const proto::NDTScanMatcherOptions2D& options)
    : options_(options) {
  approximate_voxel_filter_.setLeafSize(0.2, 0.2, 0.2);
  ndt_.setTransformationEpsilon(0.01);
  ndt_.setStepSize(0.1);
  ndt_.setResolution(1.0);
  ndt_.setMaximumIterations(35);
  prev_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  current_point_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  prev_point_cloud_->clear();
  current_point_cloud_->clear();
}

NDTScanMatcher2D::~NDTScanMatcher2D() {}

void NDTScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                               const transform::Rigid2d& initial_pose_estimate,
                               const sensor::PointCloud& point_cloud,
                               const Grid2D& grid,
                               transform::Rigid2d* const pose_estimate,
                               ceres::Solver::Summary* const summary) {
  if (prev_point_cloud_->empty()) {
    for (const auto& point : point_cloud.points()) {
      prev_point_cloud_->push_back(pcl::PointXYZ(point.position[0], point.position[1], point.position[2]));
    }
    return;
  }

  current_point_cloud_->clear();
  for (const auto& point : point_cloud.points()) {
    current_point_cloud_->push_back(pcl::PointXYZ(point.position[0], point.position[1], point.position[2]));
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud;
  approximate_voxel_filter_.setInputCloud(current_point_cloud_);
  approximate_voxel_filter_.filter(*filtered_point_cloud);
  ndt_.setInputSource(filtered_point_cloud);
  ndt_.setInputTarget(prev_point_cloud_);

  Eigen::AngleAxisf initial_rotation{initial_pose_estimate.rotation().angle(), Eigen::Vector3f::UnitZ()};
  Eigen::Translation3f initial_translation{initial_pose_estimate.translation().x(), initial_pose_estimate.translation().y(), 0};
  Eigen::Matrix4f initial_guess{(initial_translation * initial_rotation).matrix()};

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_point_cloud;
  ndt_.align(*output_point_cloud, initial_guess);
  if (ndt_.getFitnessScore() < 0.5) {
    Eigen::Matrix4f final_transform{ndt_.getFinalTransformation()};
    Eigen::AngleAxisf final_rotation{final_transform.topLeftCorner<3,3>()};
    *pose_estimate = transform::Rigid2d({final_transform(0,3),
                                         final_transform(1,3)},
                                         final_rotation.angle());
    return;
  }
}

} // namespace scan_matching
} // namespace mapping
} // namespace cartographer