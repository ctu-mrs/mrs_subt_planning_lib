#include "darpa_planning_lib/darpa_explorer.h"

using namespace std;
using namespace darpa_planning;

DarpaExplorer::DarpaExplorer() {

  is_initialized_ = false;
  createVisitedOctree();
  got_odometry       = false;
  octomap_subscribed = false;
  ROS_INFO("[Darpa Explorer] Initialized");
}

void DarpaExplorer::initialize(int min_nodes_stacked_above, double group_dist, int max_n_of_exploration_nodes, int min_dist_to_next_exp_goal,
                               std::string frame_id, bool use_preferred_flight_angle, double preferred_flight_angle, double min_dist_between_exp_nodes,
                               double min_dist_from_visited_positions, std::vector<double> exp_coeffs, octomap::point3d staging_area_ur,
                               octomap::point3d staging_area_bl, double last_flight_yaw_angle, int n_unknown_cells_threshold) {
  min_n_nodes_stacked_above         = min_nodes_stacked_above;
  grouping_distance                 = group_dist;
  max_number_of_exploration_nodes   = max_n_of_exploration_nodes;
  min_dist_to_next_exploration_goal = min_dist_to_next_exp_goal;
  map_frame                         = frame_id;
  exploration_coeffs_               = exp_coeffs;
  use_preferred_flight_angle_       = use_preferred_flight_angle;
  preferred_flight_angle_           = preferred_flight_angle;
  min_dist_between_exp_nodes_       = min_dist_between_exp_nodes;
  min_dist_from_visited_positions_  = min_dist_from_visited_positions;
  last_flight_yaw_angle_            = last_flight_yaw_angle;
  last_exploration_elevation_angle_ = 0.0;
  last_exploration_heading_         = 0.0;
  n_unknown_cells_threshold_        = n_unknown_cells_threshold;
  staging_area_max_                 = {fmax(staging_area_ur.x(), staging_area_bl.x()), fmax(staging_area_ur.y(), staging_area_bl.y()),
                       fmax(staging_area_ur.z(), staging_area_bl.z())};
  staging_area_min_                 = {fmin(staging_area_ur.x(), staging_area_bl.x()), fmin(staging_area_ur.y(), staging_area_bl.y()),
                       fmin(staging_area_ur.z(), staging_area_bl.z())};
  ;
  is_initialized_ = true;
  ROS_INFO("[DarpaExplorer]: Initializing cloud");
  used_exploration_nodes_map_.initCloud();
  visited_positions_map_.initCloud();
  ROS_INFO("[DarpaExplorer]: Inserting first used exploration node");
  used_exploration_nodes_map_.insertPoint(pcl::PointXYZ(0, 0, 1.5));
  visited_positions_map_.insertPoint(pcl::PointXYZ(0, 0, 1.5));
}

/* generateExplorationCells() //{ */
std::vector<octomap::point3d> DarpaExplorer::generateExplorationCells(std::shared_ptr<octomap::OcTree> octree_, octomap::point3d current_pose) {
  ros::Time start = ros::Time::now();
  if (!is_initialized_) {
    return std::vector<octomap::point3d>(1);
  }
  local_current_pose = current_pose;
  std::vector<octomap::point3d>          points;
  std::vector<octomap::point3d>          points_filtered;
  std::vector<octomap::point3d>          points_grouped;
  std::map<std::tuple<int, int>, double> sum_z;
  std::map<std::tuple<int, int>, int>    n_points;
  int                                    aroundRegion = 1;
  octomap::KeySet                        allKeys;
  float                                  resolution = octree_->getResolution();
  std::vector<OcTreeKey>                 keys;
  double                                 max_exp_dist = 60.0;
  octomap::point3d                       bbx_min, bbx_max;
  bbx_min.x()                    = current_pose.x() - max_exp_dist;
  bbx_min.y()                    = current_pose.y() - max_exp_dist;
  bbx_min.z()                    = current_pose.z() - max_exp_dist / 3.0;
  bbx_max.x()                    = current_pose.x() + max_exp_dist;
  bbx_max.y()                    = current_pose.y() + max_exp_dist;
  bbx_max.z()                    = current_pose.z() + max_exp_dist / 3.0;
  octomap::OcTreeKey bbx_min_key = octree_->coordToKey(bbx_min);
  octomap::OcTreeKey bbx_max_key = octree_->coordToKey(bbx_max);
  for (OcTree::leaf_bbx_iterator it = octree_->begin_leafs_bbx(bbx_min_key, bbx_max_key), end = octree_->end_leafs_bbx(); it != end; ++it) {
    /* if (it.getDepth() != octree_->getTreeDepth()) */
    /*   continue; */

    /* if (!octree_->castRay(current_pose, direction, end_key, true, v_size) && getNodeState(octree_, it.getKey()) != occupied_state) { */
    /* if (getNodeState(octree_, it.getKey()) != occupied_state) { */
    if (getNodeState(octree_, it.getKey()) == free_state) {
      point3d point = it.getCoordinate();

      if (point.x() < staging_area_max_.x() && point.x() > staging_area_min_.x() && point.y() < staging_area_max_.y() && point.y() > staging_area_min_.y() &&
          point.z() < staging_area_max_.z() && point.z() > staging_area_min_.z()) {
        /* ROS_WARN("[DarpaExplorer]: skipping point: [%.2f %.2f %.2f] inside staging area defined by min: [%.2f %.2f %.2f] max: [%.2f %.2f %.2f]", point.x(),
         * point.y(), point.z(), staging_area_min_.x(), staging_area_min_.y(), staging_area_min_.z(), staging_area_max_.x(), staging_area_max_.y(),
         * staging_area_max_.z()); */
        continue;
      }

      bool  any_neighbor_occupied       = false;
      float n_neighboring_free_cells    = 0;
      float n_neighboring_unknown_cells = 0;
      for (int dx = -aroundRegion; dx <= aroundRegion; ++dx) {
        for (int dy = -aroundRegion; dy <= aroundRegion; ++dy) {
          for (int dz = -aroundRegion; dz <= aroundRegion; ++dz) {
            point3d      point_o = point3d(point.x() + dx * resolution, point.y() + dy * resolution, point.z() + dz * resolution);
            OcTreeKey    key_o   = octree_->coordToKey(point_o);
            node_state_t state   = getNodeState(octree_, key_o);
            if (state == free_state) {
              n_neighboring_free_cells++;
            } else if (state == occupied_state) {
              any_neighbor_occupied = true;
            } else {
              n_neighboring_unknown_cells++;
            }
          }
        }
      }
      if (n_neighboring_unknown_cells > n_unknown_cells_threshold_ && !any_neighbor_occupied) {
        points.push_back(point);
        keys.push_back(it.getKey());
      }
    }
  }

  ROS_INFO_COND(verbose_, "[DarpaExplorer]: Generation of basic set of en takes %.2f ms, nof points = %lu", (ros::Time::now() - start).toSec() * 1000, points.size());
  start = ros::Time::now();

  // find number of points for each (x, y) and sum z over these points
  for (uint k = 0; k < keys.size(); k++) {
    sum_z[std::make_tuple(keys[k].k[0], keys[k].k[1])]    = 0;
    n_points[std::make_tuple(keys[k].k[0], keys[k].k[1])] = 0;
  }

  for (uint k = 0; k < points.size(); k++) {
    sum_z[std::make_tuple(keys[k].k[0], keys[k].k[1])] += keys[k].k[2];
    n_points[std::make_tuple(keys[k].k[0], keys[k].k[1])] += 1;
  }

  std::vector<std::pair<std::tuple<int, int>, double>> z_sum_vector    = std::vector<std::pair<std::tuple<int, int>, double>>(sum_z.begin(), sum_z.end());
  std::vector<std::pair<std::tuple<int, int>, int>>    n_points_vector = std::vector<std::pair<std::tuple<int, int>, int>>(n_points.begin(), n_points.end());
  std::vector<int>                                     idx(n_points.size());
  std::iota(idx.begin(), idx.end(), 0);

  stable_sort(idx.begin(), idx.end(), [&n_points_vector](int i1, int i2) { return n_points_vector[i1].second < n_points_vector[i2].second; });

  // filter nodes based on presence of others above and below
  for (std::vector<int>::iterator it = idx.begin(); it != idx.end(); it++) {
    if (n_points_vector[*it].second >= min_n_nodes_stacked_above) {
      OcTreeKey tmp_key;
      tmp_key.k[0]  = std::get<0>(n_points_vector[*it].first);
      tmp_key.k[1]  = std::get<1>(n_points_vector[*it].first);
      tmp_key.k[2]  = floor(z_sum_vector[*it].second / n_points_vector[*it].second);
      point3d point = octree_->keyToCoord(tmp_key);
      points_filtered.push_back(point);
    }
  }

  // group points
  bool point_present = false;
  for (uint k = 0; k < points_filtered.size(); k++) {
    point_present = false;
    for (uint m = 0; m < points_grouped.size(); m++) {
      if (euclideanDistance(points_filtered[k], points_grouped[m]) < grouping_distance) {
        point_present = true;
        break;
      }
    }
    if (!point_present) {
      points_grouped.push_back(points_filtered[k]);
      /* if (points_grouped.size() >= (uint)max_number_of_exploration_nodes) { */
      /*   break; */
      /* } */
    }
  }

  ROS_INFO_COND(verbose_, "[DarpaExplorer]: Generation of third set of en takes %.2f ms, nof points = %lu", (ros::Time::now() - start).toSec() * 1000, points_grouped.size());
  start = ros::Time::now();

  ROS_INFO_COND(verbose_, "[DarpaExplorer]: Points before filtering near already visited = %lu", points_grouped.size());
  std::vector<octomap::point3d> filtered_grouped_points;
  for (uint k = 0; k < points_grouped.size(); k++) {
    if ((used_exploration_nodes_map_.radiusSearch(points_grouped[k].x(), points_grouped[k].y(), points_grouped[k].z(), min_dist_between_exp_nodes_) < 0) &&
        (visited_positions_map_.radiusSearch(points_grouped[k].x(), points_grouped[k].y(), points_grouped[k].z(), min_dist_from_visited_positions_) < 0)) {
      filtered_grouped_points.push_back(points_grouped[k]);
    }
  }
  ROS_INFO_COND(verbose_, "[DarpaExplorer]: Points after filtering near already visited = %lu", filtered_grouped_points.size());

  ROS_INFO_COND(verbose_, "[DarpaExplorer]: Generation of fourth set of en takes %.2f ms, nof points = %lu", (ros::Time::now() - start).toSec() * 1000,
           filtered_grouped_points.size());
  start = ros::Time::now();

  // setting priority based on angle and distance
  std::vector<double> yaw_angles;
  std::vector<double> dists;
  std::vector<double> xy_dists;
  std::vector<double> z_dists;
  std::vector<double> z_angles;
  std::vector<double> elevation_angle_diffs;
  std::vector<double> heading_angle_diffs;
  std::vector<double> total_score;
  for (uint m = 0; m < filtered_grouped_points.size(); m++) {
    dists.push_back(euclideanDistance(filtered_grouped_points[m], local_current_pose));
    yaw_angles.push_back(computeXYAngleDifference(local_current_pose, filtered_grouped_points[m], use_preferred_flight_angle_));
    xy_dists.push_back(computeXYDist(local_current_pose, filtered_grouped_points[m]));
    z_dists.push_back(computeZDist(local_current_pose, filtered_grouped_points[m]));
    z_angles.push_back(computeZAngle(local_current_pose, filtered_grouped_points[m]));
    elevation_angle_diffs.push_back(computeAngleDifference(z_angles.back(), last_exploration_elevation_angle_));
    heading_angle_diffs.push_back(computeXYAngleDifference(local_current_pose, filtered_grouped_points[m], false));
    total_score.push_back(exploration_coeffs_[0] * dists[m] + exploration_coeffs_[1] * xy_dists[m] + exploration_coeffs_[2] * z_dists[m] +
                          exploration_coeffs_[3] * yaw_angles[m] + exploration_coeffs_[4] * z_angles[m] + exploration_coeffs_[5] * elevation_angle_diffs[m] +
                          exploration_coeffs_[6] * heading_angle_diffs[m]);
  }

  std::vector<int> idx2(filtered_grouped_points.size());
  std::iota(idx2.begin(), idx2.end(), 0);
  stable_sort(idx2.begin(), idx2.end(), [&total_score](int i1, int i2) { return total_score[i1] < total_score[i2]; });
  std::vector<point3d> points_sorted;

  for (std::vector<int>::iterator it = idx2.begin(); it != idx2.end(); it++) {
    if (dists[*it] > min_dist_to_next_exploration_goal) {
      points_sorted.push_back(filtered_grouped_points[*it]);
    }
  }

  ROS_INFO_COND(verbose_, "[DarpaExplorer]: Generation of fifth set of en takes %.2f ms, nof points = %lu", (ros::Time::now() - start).toSec() * 1000, points_sorted.size());
  ROS_INFO_COND(points_sorted.size() > 0, "[DarpaExplorer]: Size of sorted points = %lu. Points sorted [0] = [%.2f, %.2f, %.2f] with yaw = %.2f",
                points_sorted.size(), points_sorted[0].x(), points_sorted[0].y(), points_sorted[0].z(), computeXYAngleDifference(local_current_pose, points_sorted[0], use_preferred_flight_angle_));
  return points_sorted;
  /* printStatistics(octree_); */
}
//}

void DarpaExplorer::setStagingArea(const octomap::point3d &staging_area_bl, const octomap::point3d &staging_area_ur) {
  staging_area_max_ = {fmax(staging_area_ur.x(), staging_area_bl.x()), fmax(staging_area_ur.y(), staging_area_bl.y()),
                       fmax(staging_area_ur.z(), staging_area_bl.z())};
  staging_area_min_ = {fmin(staging_area_ur.x(), staging_area_bl.x()), fmin(staging_area_ur.y(), staging_area_bl.y()),
                       fmin(staging_area_ur.z(), staging_area_bl.z())};
  ROS_INFO("[DarpaExplorer]: Staging area updated to min: [%.2f, %.2f, %.2f] max: [%.2f, %.2f, %.2f]", staging_area_min_.x(), staging_area_min_.y(),
           staging_area_min_.z(), staging_area_max_.x(), staging_area_max_.y(), staging_area_max_.z());
}

void DarpaExplorer::integratePosesToVisitedOctree(std::vector<point3d> visited_path) {
  for (uint k = 0; k < visited_path.size(); k++) {
    visited_octree_->setNodeValue(visited_path[k], 2.0);
  }
}

void DarpaExplorer::setPreferredFlightAngle(double angle) {
  preferred_flight_angle_ = angle;
}

void DarpaExplorer::addPoseToUsedExplorationNodes(point3d exp_node) {
  used_exploration_nodes_.push_back(exp_node);
  used_exploration_nodes_map_.insertPoint(pcl::PointXYZ(exp_node.x(), exp_node.y(), exp_node.z()));
  last_flight_yaw_angle_ = fmod(atan2(exp_node.y() - last_exp_goal_.y(), exp_node.x() - last_exp_goal_.x()) + 2 * M_PI, 2 * M_PI);
  last_exploration_elevation_angle_ =
      fmod(atan2(exp_node.z() - local_current_pose.z(), sqrt(pow(exp_node.x() - local_current_pose.x(), 2) + pow(exp_node.y() - local_current_pose.y(), 2))) +
               2 * M_PI,
           2 * M_PI);
  last_exp_goal_ = exp_node;
}

void DarpaExplorer::addPoseToVisitedPositions(point3d exp_node) {
  visited_positions_map_.insertPoint(pcl::PointXYZ(exp_node.x(), exp_node.y(), exp_node.z()));
}

double DarpaExplorer::computeXYAngleDifference(point3d p1, point3d p2, bool use_preferred_flight_angle) {
  double ang = fmod(atan2(p2.y() - p1.y(), p2.x() - p1.x()) + 2 * M_PI, 2 * M_PI);
  double ang_diff;
  if (use_preferred_flight_angle) {
    ang_diff = fabs(fmod(2 * M_PI + preferred_flight_angle_, 2 * M_PI) - ang);
  } else {
    ang_diff = fabs(fmod(2 * M_PI + last_flight_yaw_angle_, 2 * M_PI) - ang);
  }
  double min_ang_diff = ang_diff > M_PI ? 2 * M_PI - ang_diff : ang_diff;
  return min_ang_diff;
}

double DarpaExplorer::computeAngleDifference(double a1, double a2) {
  double ang_diff     = fabs(fmod(a1 + 2 * M_PI, 2 * M_PI) - fmod(a1 + 2 * M_PI, 2 * M_PI));
  double min_ang_diff = ang_diff > M_PI ? 2 * M_PI - ang_diff : ang_diff;
  return min_ang_diff;
}

double DarpaExplorer::computeZAngle(point3d p1, point3d p2) {
  double ang = fmod(atan2(p2.z() - p1.z(), computeXYDist(p1, p2)) + 2 * M_PI, 2 * M_PI);
  return fabs(ang);
}

double DarpaExplorer::computeZDist(point3d p1, point3d p2) {
  return fabs(p2.z() - p1.z());
}

double DarpaExplorer::computeXYDist(point3d p1, point3d p2) {
  return sqrt(pow(p2.x() - p1.x(), 2) + pow(p2.y() - p1.y(), 2));
}

/* setPlanningOctree() //{ */
void DarpaExplorer::createVisitedOctree() {
  octomap::AbstractOcTree *visited_abstract_octree = octomap::AbstractOcTree::createTree("OcTree", visited_octree_resolution_);
  visited_octree_                                  = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(visited_abstract_octree));
  double xn, yn, zn, xx, yx, zx;
  visited_octree_->getMetricMax(xx, yx, zx);
  visited_octree_->getMetricMin(xn, yn, zn);
  visited_octree_->setNodeValue(octomap::point3d(0, 0, 0), 2.0);
  ROS_INFO("[DarpaExplorer]: Visited octree min = [%.2f, %.2f, %.2f], max = [%.2f, %.2f, %.2f]", xn, yn, zn, xx, yx, zx);
}
//}

/* euclideanDist() //{ */

double DarpaExplorer::euclideanDistance(const point3d p1, const point3d p2) {
  return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
}

//}

/* getNodeState() //{ */

node_state_t DarpaExplorer::getNodeState(std::shared_ptr<octomap::OcTree> tree, const OcTreeKey &key, uint8_t depth) {
  OcTreeNode *node = tree->search(key, depth);

  if (node) {
    return tree->isNodeOccupied(node) ? occupied_state : free_state;
  }

  while (!node && depth > 0) {
    --depth;
    node = tree->search(key, depth);
    if (node) {
      if (!tree->nodeHasChildren(node))
        return tree->isNodeOccupied(node) ? occupied_state : free_state;
      return unknown_state;
    }
  }

  return unknown_state;
}

//}

/* calculateNodeStats(tree, key, level) //{ */

area_desc DarpaExplorer::calculateNodeStats(const OcTree &tree, const OcTreeKey &key, uint8_t level) {
  OcTreeNode *node = tree.search(key, tree.getTreeDepth() - level);
  area_desc   desc;

  if (node == nullptr) {
    desc.unknown = 1;
    return desc;
  }

  using stack_t = std::tuple<OcTreeNode *, int>;
  std::stack<stack_t> stack;
  stack.push(std::make_tuple(node, level));

  while (!stack.empty()) {
    stack_t top = stack.top();
    stack.pop();

    OcTreeNode *node;
    int8_t      level;
    std::tie(node, level) = top;

    if (tree.nodeHasChildren(node)) {
      for (int8_t i = 0; i < 8; ++i) {
        octomap::OcTreeNode *child = tree.getNodeChild(node, i);
        if (!child)
          continue;
        stack.push(std::make_tuple(child, level - 1));
      }
    } else {
      float value = std::pow(8, level);
      if (tree.isNodeOccupied(node)) {
        desc.occupied += value;
      } else {
        desc.free += value;
      }
    }
  }

  desc.occupied /= std::pow(8, level);
  desc.unknown /= std::pow(8, level);
  desc.free = 1 - desc.occupied - desc.unknown;
  return desc;
}

//}

/* printStatistics() //{ */

void DarpaExplorer::printStatistics(std::shared_ptr<octomap::OcTree> tree) {
  std::vector<float> values;

  for (OcTree::iterator it = tree->begin(), end = tree->end(); it != end; ++it) {
    values.push_back((*it).getLogOdds());
  }
  std::sort(values.begin(), values.end());

  std::cout << "[Darpa Explorer] Value description: " << std::endl;
  int num_quantiles = 10;
  for (int i = 0; i <= num_quantiles; ++i) {
    float value = values[values.size() * i / num_quantiles - (i == num_quantiles ? 1 : 0)];
    std::cout << "Quantile: " << i << "/" << num_quantiles;
    std::cout << "\tlogOdds: " << value << " \tprob: " << octomap::probability(value) << std::endl;
  }
}

//}

/* setVerbose() //{ */

void DarpaExplorer::setVerbose(const bool verbose) {
  verbose_ = verbose;
}

//}
