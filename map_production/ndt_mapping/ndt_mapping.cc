#include <cassert>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include "gflags/gflags.h"


using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = pcl::PointCloud<Point>::Ptr;
using PointCloudConstPtr = const pcl::PointCloud<Point>::Ptr;
using NormalDistributionsTransform = 
          pcl::NormalDistributionsTransform<Point, Point>;
using PassThrough = pcl::PassThrough<Point>;
using VoxelGrid = pcl::VoxelGrid<Point>;

DEFINE_double(min_scan_range, 5.0, "min scan range.");
DEFINE_double(max_scan_range, 200.0, "min scan range.");
DEFINE_double(voxel_leaf_size, 2.0, "voxel leaf size");
// ndt
DEFINE_double(trans_eps, 0.01, "trans_eps");
DEFINE_double(step_size, 0.1, "step_size");
DEFINE_double(ndt_res, 1.0, "ndt_res");
DEFINE_int32(max_iter, 30, "max_iter");
// map
DEFINE_string(map_file_name, "", "map save file path");


PointCloudPtr map_ptr(new PointCloud());
bool is_first_map = true;

Eigen::Affine3d tf_btol, tf_ltob;

NormalDistributionsTransform ndt;

void Init() {
  // ndt
  ndt.setTransformationEpsilon(FLAGS_trans_eps);
  ndt.setStepSize(FLAGS_step_size);
  ndt.setResolution(FLAGS_ndt_res);
  ndt.setMaximumIterations(FLAGS_max_iter);


}

void RangeFilter(PointCloudConstPtr input, PointCloudPtr output) {
  for (PointCloud::const_iterator item = input->begin(); item != input->end(); 
        item++) {
    Point point;
    point.x = item->x;
    point.y = item->y;
    point.z = item->z;
    point.intensity = item->intensity;

    double r = std::sqrt(std::pow(point.x, 2.0) + std::pow(point.y, 2.0));
    if (FLAGS_min_scan_range <= r && r <= FLAGS_max_scan_range) {
      output->push_back(point);
    }
  }
}

void VoxelFilter(PointCloudConstPtr input, PointCloudPtr output) {
  VoxelGrid voxel_grid_filter;
  voxel_grid_filter.setLeafSize(FLAGS_voxel_leaf_size, FLAGS_voxel_leaf_size, 
      FLAGS_voxel_leaf_size);
  voxel_grid_filter.setInputCloud(input);
  voxel_grid_filter.filter(*output);
}

void LidarProcess(PointCloudPtr cloud_ptr) {
  PointCloudPtr scan_ptr(new PointCloud());
  RangeFilter(cloud_ptr, scan_ptr);

  if (is_first_map) {
    *map_ptr += *scan_ptr;
    is_first_map = false;
    return;
  }

  PointCloudPtr voxel_ptr(new PointCloud());
  VoxelFilter(scan_ptr, voxel_ptr);

  ndt.setInputSource(voxel_ptr);
  ndt.setInputTarget(map_ptr);

  Eigen::Matrix4f init_guess;

  PointCloudPtr output_cloud(new PointCloud());
  ndt.align(*output_cloud, init_guess);

  double fitness_score = ndt.getFitnessScore();
  Eigen::Matrix4f t_localizer = ndt.getFinalTransformation();
  bool has_converged = ndt.hasConverged();
  int final_num_iteration = ndt.getFinalNumIteration();
  double transformation_probability = ndt.getTransformationProbability();
  
  // Todo(zero): add fitness score
  if (shift >= min_add_scan_shift) {
    PointCloudPtr transformed_scan_ptr(new PointCloud());
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
    *map_ptr += *transformed_scan_ptr;
  }

  // Todo(zero): Add for online mode
  // Publish map
  // Publihs pose
}

void SaveMap() {
  assert(map_ptr != nullptr);

  pcl::io::savePCDFileASCII(FLAGS_map_file_name, *map_ptr);
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
}