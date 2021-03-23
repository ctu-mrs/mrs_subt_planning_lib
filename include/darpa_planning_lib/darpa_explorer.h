#ifndef __DARPA_EXPLORER_H__
#define __DARPA_EXPLORER_H__

/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <visualization_msgs/Marker.h>

#include <tuple>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <stack>
#include <nav_msgs/Odometry.h>
#include "darpa_planning_lib/pcl_map.h"

//}

namespace darpa_planning
{

using octomap::OcTree;
using octomap::OcTreeKey;
using octomap::OcTreeNode;
using octomap::point3d;

struct area_desc
{
  float occupied, unknown, free;

  area_desc() = default;
  area_desc(float occupied, float unknown, float free) : occupied(occupied), unknown(unknown), free(free){};
};

enum node_state_t
{
  unknown_state,
  free_state,
  occupied_state
};

class DarpaExplorer {

public:
  DarpaExplorer();

  std::vector<octomap::point3d> generateExplorationCells(std::shared_ptr<octomap::OcTree> tree, octomap::point3d current_pose);
  void initialize(int min_nodes_stacked_above, double group_dist, int max_n_of_exploration_nodes, int min_dist_to_next_exploration_goal, std::string map_frame,
                  bool use_preferred_flight_angle, double preferred_flight_angle, double min_dist_between_exp_nodes, double min_dist_from_visited_position,
                  std::vector<double> exp_coeffs, octomap::point3d staging_area_ur, octomap::point3d staging_area_bl, double last_flight_yaw_angle,
                  int n_unknown_cells_threshold);
  void integratePosesToVisitedOctree(std::vector<point3d> visited_path);
  void addPoseToUsedExplorationNodes(point3d exp_node);
  void addPoseToVisitedPositions(point3d exp_node);
  void setPreferredFlightAngle(double angle);
  void setStagingArea(const octomap::point3d &staging_area_bl, const octomap::point3d &staging_area_ur);
  void setVerbose(const bool verbose);

protected:
  std::string         map_frame;
  bool                octomap_subscribed;
  int                 min_n_nodes_stacked_above;
  int                 n_unknown_cells_threshold_;
  double              grouping_distance;
  int                 max_number_of_exploration_nodes;
  double              min_dist_to_next_exploration_goal;
  bool                got_odometry;
  double              visited_octree_resolution_;
  bool                is_initialized_;
  double              last_flight_yaw_angle_;
  double              preferred_flight_angle_;
  bool                use_preferred_flight_angle_;
  std::vector<double> exploration_coeffs_;
  PCLMap              used_exploration_nodes_map_;
  PCLMap              visited_positions_map_;
  double              min_dist_between_exp_nodes_;
  double              min_dist_from_visited_positions_;
  octomap::point3d    staging_area_max_;
  octomap::point3d    staging_area_min_;
  double              last_exploration_elevation_angle_;
  double              last_exploration_heading_;
  bool                verbose_;

  std::shared_ptr<octomap::OcTree> visited_octree_;
  octomap_msgs::Octomap            octomap_msg_;
  point3d                          local_current_pose;
  octomap::point3d                 last_exp_goal_;
  std::vector<octomap::point3d>    used_exploration_nodes_;

  area_desc    calculateNodeStats(const OcTree &, const OcTreeKey &, uint8_t);
  node_state_t getNodeState(std::shared_ptr<octomap::OcTree> tree, const OcTreeKey &key, uint8_t depth = 16);
  void         printStatistics(std::shared_ptr<octomap::OcTree> tree);
  double       euclideanDistance(point3d p1, point3d p2);
  void         callbackOctomap(const octomap_msgs::OctomapConstPtr &msg);
  void         callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  void         closeGateInPointCloud(octomap::point3d bottom_left, octomap::point3d upper_right);
  void         publishExplorationPoints(std::vector<octomap::point3d> octo_points);
  void         createVisitedOctree();
  double       computeXYAngleDifference(point3d p1, point3d p2, bool use_preferred_flight_angle);
  double       computeAngleDifference(double a1, double a2);
  double       computeZAngle(point3d p1, point3d p2);
  double       computeZDist(point3d p1, point3d p2);
  double       computeXYDist(point3d p1, point3d p2);

  visualization_msgs::Marker convertPointsToMsg(std::vector<octomap::point3d> points, std::string frame_id);
};
}  // namespace darpa_planning

#endif
