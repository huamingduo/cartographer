#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/proto/scan_matching/ndt_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"

#include "pcl/point_types.h"
#include "pcl/registration/ndt.h"
#include "pcl/filters/approximate_voxel_grid.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::NDTScanMatcherOptions2D CreateNDTScanMatcherOptions2D(
  common::LuaParameterDictionary* paramter_dictionary);

class NDTScanMatcher2D {
  public:
    explicit NDTScanMatcher2D(const proto::NDTScanMatcherOptions2D& options);
    virtual ~NDTScanMatcher2D();
    NDTScanMatcher2D(const NDTScanMatcher2D&) = delete;
    NDTScanMatcher2D& operator=(const NDTScanMatcher2D&) = delete;
    void Match(const Eigen::Vector2d& target_translation,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud, const Grid2D& grid,
      transform::Rigid2d* pose_estimate,
      ceres::Solver::Summary* summary);
  private:
    const proto::NDTScanMatcherOptions2D options_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_point_cloud_, current_point_cloud_;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter_;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
}; // class

} // namespace scan_matching
} // namespace mapping
} // namespace cartographer
