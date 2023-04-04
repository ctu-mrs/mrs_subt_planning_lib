#include "mrs_subt_planning_lib/astar_planner.h"

using namespace std;
using namespace mrs_subt_planning;

AstarPlanner::AstarPlanner(void) {
  initialized_         = false;
  verbose_             = false;
  astar_admissibility_ = 1.0;
}

AstarPlanner::~AstarPlanner() {
}

/* initialize() //{ */
void AstarPlanner::initialize(octomap::point3d start_point, octomap::point3d goal_point, std::shared_ptr<octomap::OcTree> planning_octree,
                              bool enable_planning_to_unreachable_goal, double planning_timeout, double safe_dist, double clearing_dist, double min_altitude,
                              double max_altitude, bool debug, std::shared_ptr<mrs_lib::BatchVisualizer> batch_visualizer, const bool break_at_timeout) {
  planning_octree_ = planning_octree;
  start_.pose      = start_point;
  goal_.pose       = goal_point;
  planning_octree_->getMetricSize(grid_params_.width, grid_params_.height, grid_params_.depth);
  planning_octree_->getMetricMin(grid_params_.min_x, grid_params_.min_y, grid_params_.min_z);
  planning_octree_->getMetricMax(grid_params_.max_x, grid_params_.max_y, grid_params_.max_z);
  min_altitude_ = min_altitude;
  max_altitude_ = max_altitude;
  resolution_   = planning_octree_->getResolution();
  ROS_INFO("[AstarPlanner]: Astarplanner with resolution %.2f initialized", resolution_);
  enable_planning_to_unreachable_goal_ = enable_planning_to_unreachable_goal;
  postprocessing_timeout_              = 0.2;
  planning_timeout_                    = planning_timeout - postprocessing_timeout_;
  debug_                               = debug;
  safe_dist_                           = safe_dist;
  safe_dist_prev_                      = safe_dist_;
  clearing_dist_                       = clearing_dist;
  break_at_timeout_                    = break_at_timeout;
  batch_visualizer_                    = batch_visualizer;
  map_conversion_time_                 = 0.0;

  initializeIdxsOfcellsForPruning();
  initialized_ = true;
}
//}

/* initialize() //{ */
void AstarPlanner::initialize(bool enable_planning_to_unreachable_goal, double planning_timeout, double postprocessing_timeout, double safe_dist,
                              double clearing_dist, double min_altitude, double max_altitude, bool debug,
                              std::shared_ptr<mrs_lib::BatchVisualizer> batch_visualizer, const bool break_at_timeout) {
  enable_planning_to_unreachable_goal_ = enable_planning_to_unreachable_goal;
  planning_timeout_                    = planning_timeout;
  postprocessing_timeout_              = postprocessing_timeout;
  debug_                               = debug;
  safe_dist_                           = safe_dist;
  safe_dist_prev_                      = safe_dist_;
  clearing_dist_                       = clearing_dist;
  break_at_timeout_                    = break_at_timeout;
  min_altitude_                        = min_altitude;
  max_altitude_                        = max_altitude;
  batch_visualizer_                    = batch_visualizer;
  map_conversion_time_                 = 0.0;

  initializeIdxsOfcellsForPruning();
  initialized_ = true;
}
//}

/* isNodeValid //{ */
bool AstarPlanner::isNodeValid(const Node& n) {
  // out of bounds
  octomap::point3d p = planning_octree_->keyToCoord(n.key);
  if (p.x() < grid_params_.min_x || p.x() > grid_params_.max_x || p.y() < grid_params_.min_y || p.y() > grid_params_.max_y || p.z() < grid_params_.min_z ||
      p.z() > grid_params_.max_z) {
    return false;
  }
  if (isNodeInTheNeighborhood(n.key, start_.key, clearing_dist_)) {  // unknown
    return true;
  } else if (planning_octree_->search(n.key) == NULL || planning_octree_->isNodeOccupied(planning_octree_->search(n.key))) {  // occupied
    return false;
  }
  /* else if (planning_octree_->search(n.key) != NULL && planning_octree_->isNodeOccupied(planning_octree_->search(n.key))) { */
  /*   return false; */
  /* } */

  return true;
}
//}

/* checkValidityWithNeighborhood() //{ */
bool AstarPlanner::checkValidityWithNeighborhood(const Node& n) {
  if (isNodeInTheNeighborhood(n.key, start_.key, clearing_dist_)) {  // unknown
    return true;
  } else if (planning_octree_->search(n.key) == NULL) {
    return false;
  }
  return checkValidityWithKDTree(n);
}
//}

/* checkValidityWithNeighborhood() //{ */
bool AstarPlanner::checkValidityWithNeighborhood(const octomap::OcTreeKey& k) {
  if (isNodeInTheNeighborhood(k, start_.key, clearing_dist_)) {  // unknown
    return true;
  } else if (planning_octree_->search(k) == NULL) {
    return false;
  }
  return checkValidityWithKDTree(k);
}
//}

/* checkValidityWitKDTree() //{ */
bool AstarPlanner::checkValidityWithKDTree(const Node& n) {
  pcl::PointXYZ p = octomapKeyToPclPoint(n.key);
  if (p.z < grid_params_.min_z || p.z > grid_params_.max_z || pcl_map_.getDistanceFromNearestPoint(p) < safe_dist_) {
    return false;
  }
  return true;
}
//}

/* checkValidityWitKDTree() //{ */
bool AstarPlanner::checkValidityWithKDTree(const octomap::OcTreeKey& k) {
  pcl::PointXYZ p = octomapKeyToPclPoint(k);
  if (p.z < grid_params_.min_z || p.z > grid_params_.max_z || pcl_map_.getDistanceFromNearestPoint(p) < safe_dist_) {
    return false;
  }
  return true;
}
//}

/* getValidNodeInNeighborhood() //{ */
Node AstarPlanner::getValidNodeInNeighborhood(const Node& goal) {
  std::vector<Node> neighbors = getNeighborhood26(goal);
  for (std::vector<Node>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
    if (checkValidityWithNeighborhood(*it)) {
      return *it;
    }
  }
  Node n;
  n.key.k[0] = 0;  // indicator of invalid node
  return n;
}
//}

/* findPath() //{ */

std::pair<std::vector<octomap::point3d>, bool> AstarPlanner::findPath(
    const octomap::point3d& start_point, const octomap::point3d& goal_point, std::shared_ptr<octomap::OcTree> planning_octree, bool make_path_straight,
    bool apply_postprocessing, double planning_bbx_size_h, double planning_bbx_size_v, double postprocessing_safe_dist, int postprocessing_max_iterations,
    bool postprocessing_horizontal_neighbors_only, double postprocessing_z_tolerance, double postprocessing_path_length, int shortening_window_size,
    double shortening_dist, bool apply_pruning, double pruning_dist, bool ignore_unknown_cells_near_start, double box_size_for_unknown_cells_replacement) {

  if (make_path_straight && apply_postprocessing) {
    ROS_WARN("[%s]: The path straightening cannot be applied together with the path postprocessing. ", ros::this_node::getName().c_str());
  }
  ROS_INFO("[%s]: Findpath: Planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), start_point.x(), start_point.y(), start_point.z(), goal_point.x(), goal_point.y(), goal_point.z());

  std::vector<double> bbx   = {planning_bbx_size_h, planning_bbx_size_h, planning_bbx_size_v};
  ros::Time           start = ros::Time::now();
  ROS_INFO("[%s]: Tree resampling took %.2f s.", ros::this_node::getName().c_str(), (ros::Time::now() - start).toSec());
  std::vector<Node> node_path = getNodePath(start_point, goal_point, planning_octree, ignore_unknown_cells_near_start, box_size_for_unknown_cells_replacement);
  std::vector<octomap::point3d>   waypoints;
  std::vector<octomap::OcTreeKey> waypoints_keys;

  start = ros::Time::now();
  if (apply_postprocessing) {

    int n_waypoints_max;
    if (postprocessing_path_length > 0) { 
      n_waypoints_max = static_cast<int>(ceil(postprocessing_path_length / resolution_));
    } else { 
      n_waypoints_max = node_path.size();
    }

    std::vector<Node> node_subpath =
        std::vector<Node>(node_path.begin(), node_path.begin() + std::min(n_waypoints_max, static_cast<int>(node_path.size())));
    waypoints_keys = getSafePath(getKeyPath(node_subpath), postprocessing_safe_dist, postprocessing_max_iterations, postprocessing_z_tolerance, true,
                                 postprocessing_horizontal_neighbors_only, postprocessing_timeout_);
    std::vector<octomap::OcTreeKey> safe_filtered_key_plan =
        getFilteredPlan(waypoints_keys, shortening_window_size, shortening_dist);  // FIXME: check whether there was no reason to comment this out
    waypoints = getWaypointPath(safe_filtered_key_plan);

    ROS_INFO("[%s]: Path postprocessing took %.2f ms.", ros::this_node::getName().c_str(), (ros::Time::now() - start).toSec() * 1000);

  } else if (make_path_straight) {
    waypoints = getStraightenWaypointPath(node_path, 0.2);
    ROS_INFO("[%s]: Path straightening took %.2f s.", ros::this_node::getName().c_str(), (ros::Time::now() - start).toSec());
  } else {
    waypoints = getWaypointPath(node_path);
  }

  if (apply_pruning) {
    waypoints = pruneWaypoints(waypoints, pruning_dist);
  }

  waypoints = getWaypointPathWithoutObsoletePoints(waypoints, 0.05);

  ROS_INFO("[%s]: ----------------- Init path -------------------", ros::this_node::getName().c_str());
  for (size_t k = 0; k < node_path.size(); k++) {
    octomap::point3d p = planning_octree->keyToCoord(node_path[k].key);
    ROS_INFO("[%s]: Node %lu: [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), k, p.x(), p.y(), p.z());
  }

  ROS_INFO("[%s]: ----------------- Final path -------------------", ros::this_node::getName().c_str());
  for (size_t k = 0; k < waypoints.size(); k++) {
    ROS_INFO("[%s]: Node %lu: [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), k, waypoints[k].x(), waypoints[k].y(), waypoints[k].z());
  }

  bool direct_path_to_goal_found = false;  // TODO: return this bool from getNodePathFunction
  return std::make_pair(waypoints, direct_path_to_goal_found);
}

//}

/* getNodePath() //{ */
std::vector<Node> AstarPlanner::getNodePath(const octomap::point3d& start_point, const octomap::point3d& goal_point,
                                            std::shared_ptr<octomap::OcTree> planning_octree, bool ignore_unknown_cells_near_start,
                                            double box_size_for_unknown_cells_replacement) {

  std::vector<Node> waypoints;

  if (!initialized_) {
    ROS_WARN("[AstarPlanner]: Cannot start planning, planner not initialized. Returning empty path.");
    return waypoints;
  }

  planning_octree_ = planning_octree;
  planning_octree_->getMetricSize(grid_params_.width, grid_params_.height, grid_params_.depth);
  planning_octree_->getMetricMin(grid_params_.min_x, grid_params_.min_y, grid_params_.min_z);
  planning_octree_->getMetricMax(grid_params_.max_x, grid_params_.max_y, grid_params_.max_z);
  grid_params_.max_z = max_altitude_;
  grid_params_.min_z = min_altitude_;
  resolution_        = planning_octree_->getResolution();

  start_.pose = start_point;
  goal_.pose  = goal_point;

  ROS_INFO("[AstarPlanner]: Get node path start, resolution = %.2f", resolution_);

  octomap::OcTreeKey start_key = planning_octree_->coordToKey(start_point);
  ROS_INFO("[debug]: astar planner ignore unknown cells near start = %d ", ignore_unknown_cells_near_start);
  if (ignore_unknown_cells_near_start) {
    replaceUnknownByFreeCells(start_key, box_size_for_unknown_cells_replacement);
  }

  waypoints = getNodePath();

  if (waypoints.size() > 5) {
    safe_dist_prev_ = safe_dist_;
  }

  return waypoints;
}
//}

/* getNodePath() //{ */
std::vector<Node> AstarPlanner::getNodePath(const std::vector<octomap::point3d>& initial_waypoints, std::shared_ptr<octomap::OcTree> planning_octree,
                                            bool ignore_unknown_cells_near_start, double box_size_for_unknown_cells_replacement) {

  std::vector<Node> waypoints;

  if (!initialized_) {
    ROS_WARN("[AstarPlanner]: Cannot start planning, planner not initialized. Returning empty path.");
    return waypoints;
  }

  if (initial_waypoints.size() < 2) {
    ROS_WARN("[AstarPlanner]: Cannot start planning, vector of waypoints contains only %lu waypoints, at least 2 (start and goal) expected.",
             initial_waypoints.size());
  }

  planning_octree_ = planning_octree;
  planning_octree_->getMetricSize(grid_params_.width, grid_params_.height, grid_params_.depth);
  planning_octree_->getMetricMin(grid_params_.min_x, grid_params_.min_y, grid_params_.min_z);
  planning_octree_->getMetricMax(grid_params_.max_x, grid_params_.max_y, grid_params_.max_z);
  grid_params_.max_z = max_altitude_;
  grid_params_.min_z = min_altitude_;
  resolution_        = planning_octree_->getResolution();

  if (ignore_unknown_cells_near_start) {
    replaceUnknownByFreeCells(planning_octree_->coordToKey(initial_waypoints[0]), box_size_for_unknown_cells_replacement);
  }

  double former_planning_timeout = planning_timeout_;                                         // store planning timeout for a single path
  planning_timeout_              = planning_timeout_ / double(initial_waypoints.size() - 1);  // change planning timeout according to number of waypoints
  std::vector<Node> partial_waypoints;
  ROS_INFO("[AstarPlanner]: Get node path for multiple waypoints, resolution = %.2f", resolution_);
  for (size_t k = 1; k < initial_waypoints.size(); k++) {
    start_.pose       = waypoints.size() == 0 ? initial_waypoints[0] : waypoints.back().pose;
    goal_.pose        = initial_waypoints[k];
    partial_waypoints = getNodePath();

    if (partial_waypoints.size() == 0) {
      if (waypoints.size() == 0) {
        ROS_WARN("[AstarPlanner]: Partial path to goal %lu not found proceeding to next point.", k);
        continue;
      } else {
        ROS_WARN("[AstarPlanner]: Partial path not found, returning found path.");
        return waypoints;
      }
    } else {
      waypoints.insert(waypoints.end(), partial_waypoints.begin(), partial_waypoints.end());
    }
  }

  planning_timeout_ = former_planning_timeout;  // restore former planning timeout for single path

  if (waypoints.size() > 5) {
    safe_dist_prev_ = safe_dist_;
  }

  return waypoints;
}
//}

/* replaceUnknownByFreeCells //{ */
void AstarPlanner::replaceUnknownByFreeCells(const octomap::OcTreeKey& start_key, double box_size) {

  if (planning_octree_ == NULL) {
    return;
  }

  octomap::point3d      p = planning_octree_->keyToCoord(start_key);
  octomap::point3d_list unknown_cells_centers;
  octomap::point3d      p_min = p - octomap::point3d(box_size, box_size, box_size);
  octomap::point3d      p_max = p + octomap::point3d(box_size, box_size, box_size);

  planning_octree_->getUnknownLeafCenters(unknown_cells_centers, p_min, p_max);
  ROS_INFO_COND(verbose_, "[AstarPlanner]: Replacing unknown cells by free in surrounding of point [%.2f, %.2f, %.2f].", p.x(), p.y(), p.z());
  ROS_INFO("[debug]: unknown cells centers size before = %lu", unknown_cells_centers.size());

  for (auto& n : unknown_cells_centers) {
    planning_octree_->updateNode(n, false);
  }

  octomap::point3d_list unknown_cells_centers_after;
  planning_octree_->getUnknownLeafCenters(unknown_cells_centers_after, p_min, p_max);
  ROS_INFO("[debug]: unknown cells centers size after = %lu", unknown_cells_centers_after.size());

  ROS_INFO_COND(debug_, "[AstarPlanner]: Unknown cells replaced.");
}
//}

/* getNodePath() //{ */
std::vector<Node> AstarPlanner::getNodePath() {
  ROS_INFO("[AstarPlanner]: Get node path start");
  std::vector<Node>                            waypoints;
  AstarPriorityQueue                           open_list;
  std::unordered_set<Node, NodeHasher>         closed_list;
  std::unordered_set<Node, NodeHasher>         open_set;
  std::unordered_map<Node, Node, NodeHasher>   parent_list;
  std::unordered_map<Node, double, NodeHasher> cost_so_far;

  start_.key    = planning_octree_->coordToKey(start_.pose);
  start_.f_cost = 0.0;
  goal_.key     = planning_octree_->coordToKey(goal_.pose);
  goal_.h_cost  = 0.0;

  ros::Time start_time = ros::Time::now();
  ROS_INFO_COND(debug_, "[AstarPlanner] Start octomap to pointcloud");
  std::vector<pcl::PointXYZ> pcl_points =
      octomapToPointcloud();  // TODO: replace by detection of maxmin x, maxmin y and maxmin z, for reasonable setting of are

  if (pcl_points.size() > 0) {
    ROS_INFO_COND(verbose_, "[AstarPlanner]: Start conversion");
    pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_pointcloud = PCLMap::pclVectorToPointcloud(pcl_points);
    ROS_INFO_COND(debug_, "[AstarPlanner]: init kd tree start");
    pcl_map_.initKDTreeSearch(simulated_pointcloud);
    ROS_INFO_COND(debug_, "[AstarPlanner]: init kd tree end");
  }

  if (!checkValidityWithNeighborhood(goal_)) {
    ROS_WARN_COND(debug_, "[AstarPlanner]: Goal destination unreachable.");
    Node secondary_goal_ = getValidNodeInNeighborhood(goal_);
    if (secondary_goal_.key.k[0] == 0) {
      ROS_WARN_COND(verbose_, "[AstarPlanner]: Secondary goal in the neighborhood not found. Destination unreachable.");
      if (!enable_planning_to_unreachable_goal_) {
        ROS_WARN_COND(verbose_, "[AstarPlanner]: Planning to unreachable goal not allowed. Returning empty path.");
        return waypoints;
      } else {
        ROS_INFO_COND(debug_, "[AstarPlanner]: Goal unreachable, but planning to unreachable goal allowed.");
      }
    } else {
      ROS_INFO_COND(debug_, "[AstarPlanner]: Secondary goal found. Original goal [%d, %d, %d] replaced by [%d, %d, %d].", goal_.key.k[0], goal_.key.k[1],
                    goal_.key.k[2], secondary_goal_.key.k[0], secondary_goal_.key.k[1], secondary_goal_.key.k[2]);
      goal_.pose           = planning_octree_->keyToCoord(goal_.key);
      secondary_goal_.pose = planning_octree_->keyToCoord(secondary_goal_.key);
      ROS_INFO_COND(verbose_, "[AstarPlanner]: Secondary goal found. Original goal [%.2f, %.2f, %.2f] replaced by [%.2f, %.2f, %.2f].", goal_.pose.x(),
                    goal_.pose.y(), goal_.pose.z(), secondary_goal_.pose.x(), secondary_goal_.pose.y(), secondary_goal_.pose.z());
      goal_ = secondary_goal_;
    }
  }

  if (!checkValidityWithNeighborhood(start_) && safe_dist_prev_ < safe_dist_) {  // prevents stuck due to increasing safe_dist
    safe_dist_ = safe_dist_prev_;
  }

  if (isNodeGoal(start_)) {
    ROS_WARN("[AstarPlanner]: Planner initialized at goal position. Returning empty plan.");
    waypoints.push_back(start_);
    return waypoints;
  }

  std::vector<Node> waypoints_init = getPathToNearestFeasibleNode(start_);

  if (waypoints_init.size() > 0) {
    start_ = waypoints_init.back();
    ROS_WARN("[AstarPlanner]: Start position unfeasible. Generating path to nearest feasible node.");
  }

  ROS_INFO_COND(debug_, "[AstarPlanner]: Add start into open list.");
  start_.f_cost = 0.0;
  open_list.push(start_);
  open_set.insert(start_);
  closed_list.insert(start_);
  Node current;
  Node nearest     = start_;
  nearest.h_cost   = DBL_MAX;
  int loop_counter = 1;
  int node_removed = 0;  // 0 for not present in open list, 1 for present and removed, -1 for present and not removed
  ROS_INFO_COND(debug_, "[AstarPlanner]: Start key = [%d, %d, %d]", start_.key.k[0], start_.key.k[1], start_.key.k[2]);
  ROS_INFO_COND(debug_, "[AstarPlanner]: Goal key = [%d, %d, %d]", goal_.key.k[0], goal_.key.k[1], goal_.key.k[2]);

  while (!open_set.empty()) {
    if (loop_counter % 100 == 0) {
      ROS_INFO_COND(debug_, "[AstarPlanner]: Loop counter = %d, open list size = %lu, closed_list_size = %lu", loop_counter, open_list.size(),
                    closed_list.size());
      if ((ros::Time::now() - start_time).toSec() > (planning_timeout_ - map_conversion_time_)) {
        ROS_WARN("[AstarPlanner]: Planning timeout reached.");
        break;
      }
    }
    current = open_list.top();
    open_set.erase(current);
    open_list.pop();
    closed_list.insert(current);

    if (isNodeGoal(current)) {
      ROS_INFO_COND(debug_, "[AstarPlanner]: Goal found");
      break;
    }
    std::vector<Node> neighbors;
    if (loop_counter == 1) {
      neighbors = getNeighborhood26(current);
    } else {
      neighbors = getPossibleSuccessors(current.parent_key, current.key);
      /* neighbors = getNeighborhood26(current); */
    }
    for (std::vector<Node>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {

      if (!checkValidityWithNeighborhood(*it)) {
        continue;
      }

      if (closed_list.find(*it) != closed_list.end() || open_set.find(*it) != open_set.end()) {
        continue;
      }

      double new_cost = current.f_cost + nodeDistance(current, *it);  // nodeDistance can be replaced with 1.0 for 6 neighborhood

      it->f_cost     = new_cost;
      it->h_cost     = astar_admissibility_ * euclideanCost(*it);
      it->g_cost     = it->f_cost + it->h_cost;
      it->parent_key = current.key;
      if (it->h_cost < nearest.h_cost) {
        nearest = *it;
      }
      parent_list[*it] = current;
      open_set.insert(*it);
      open_list.push(*it);
    }

    loop_counter++;
  }
  ROS_INFO("[AstarPlanner debug]: Astar ended after %d iterations", loop_counter);

  batch_visualizer_->clearVisuals();
  batch_visualizer_->clearBuffers();
  visualizeOccupiedPoints(pcl_points);
  visualizeGoal(planning_octree_->keyToCoord(goal_.key));
  ROS_INFO("[AstarPlanner debug]: Open set size %lu.", open_set.size());
  visualizeExpansions(open_set, closed_list, *planning_octree_);
  batch_visualizer_->publish();

  // path reconstruction
  if (!isNodeGoal(current)) {
    ROS_WARN("[AstarPlanner]: Path not found, goal unreachable.");

    if (break_at_timeout_) {
      return std::vector<Node>();
    }

    octomap::point3d nearest_coords = planning_octree_->keyToCoord(nearest.key);
    octomap::point3d goal_coords    = planning_octree_->keyToCoord(goal_.key);
    ROS_INFO_COND(verbose_,
                  "[AstarPlanner]: Path to nearest node to goal [%.2f, %.2f, %.2f] found. Replacing original goal [%.2f, %.2f, %.2f] by nearest node.",
                  nearest_coords.x(), nearest_coords.y(), nearest_coords.z(), goal_coords.x(), goal_coords.y(), goal_coords.z());

    if (!areKeysEqual(current.key, nearest.key)) {
      ROS_INFO_COND(debug_, "[AstarPlanner]: current and nearest are not equal");
      current = nearest;
    }
  }

  last_found_goal_ = current;
  ROS_INFO_COND(debug_, "[AstarPlanner]: start path reconstruction");
  current.pose = planning_octree_->keyToCoord(current.key);
  waypoints.push_back(current);
  int counter = 0;
  ROS_INFO_COND(debug_, "[AstarPlanner]: path reconstruction %d, waypoint key = [%d, %d, %d]", counter, waypoints[counter].key.k[0],
                waypoints[counter].key.k[1], waypoints[counter].key.k[2]);

  while (abs(waypoints[counter].f_cost) > 1e-5) {
    waypoints.push_back(parent_list[waypoints[counter]]);
    counter++;
    waypoints[counter].pose = planning_octree_->keyToCoord(waypoints[counter].key);
    ROS_INFO_COND(debug_, "[AstarPlanner]: path reconstruction %d, waypoint key = [%d, %d, %d]", counter, waypoints[counter].key.k[0],
                  waypoints[counter].key.k[1], waypoints[counter].key.k[2]);
    /* sleep(0.1); */
  }

  // reverse the path from end to beginning
  ROS_INFO_COND(debug_, "[AstarPlanner]: reversing waypoints");
  std::reverse(waypoints.begin(), waypoints.end());
  waypoints_init.insert(waypoints_init.end(), waypoints.begin(), waypoints.end());
  /* stop_index         = 15; */
  /* start_node_next    = waypoints[0]; */
  ros::Time end_time = ros::Time::now();
  ROS_INFO_COND(debug_, "[AstarPlanner]: AstarPlanner: returning path of %lu waypoints", waypoints_init.size());
  ROS_WARN_COND(verbose_, "[AstarPlanner]: Path planning took %.3f ms", (end_time - start_time).toSec() * 1000.0);
  return waypoints_init;
}
//}

/* findPathToNearestFeasibleNode() //{ */

std::vector<Node> AstarPlanner::getPathToNearestFeasibleNode(const Node& start) {

  ros::Time         start_time = ros::Time::now();
  std::vector<Node> waypoints_filtered;
  if (!checkValidityWithKDTree(start)) {  // start is not feasible, try to find closest feasible point
    ROS_INFO("[%s]: gpnfn start node is not collision free ", ros::this_node::getName().c_str());
    double global_safe_dist = safe_dist_;
    safe_dist_              = 0.0;
    std::priority_queue<Node, std::vector<Node>, NodeCompare> heap;
    std::unordered_set<Node, NodeHasher>                      closed;
    std::unordered_map<Node, Node, NodeHasher>                parents;  // first = child, second = parent

    Node start_l   = start;
    start_l.g_cost = 0.0;
    start_l.h_cost = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(start_l.key));
    start_l.f_cost = start_l.h_cost;
    heap.push(start_l);
    Node              current;
    Node              best         = start_l;
    int               loop_counter = 0;
    std::vector<Node> neighbors;
    int               iter_limits = 200;

    while (!heap.empty() && loop_counter < iter_limits) {

      current = heap.top();
      heap.pop();

      if (best.f_cost + 0.2 * best.h_cost + 0.001 * best.g_cost > current.f_cost + 0.2 * current.h_cost + 0.001 * current.g_cost) {
        best = current;
      }

      if (current.f_cost <= 0) {
        best = current;
        break;
      }

      neighbors = getNeighborhood26(current);

      for (std::vector<Node>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {

        if (planning_octree_->search(it->key) == NULL || planning_octree_->isNodeOccupied(planning_octree_->search(it->key))) {
          continue;
        }

        if (closed.find(*it) != closed.end()) {
          continue;
        }

        double obst_dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(it->key));


        it->f_cost     = global_safe_dist - obst_dist;
        it->h_cost     = fmax(current.h_cost, it->f_cost);
        it->g_cost     = current.g_cost + it->f_cost;
        it->parent_key = current.key;
        parents[*it]   = current;

        heap.push(*it);
      }


      closed.insert(current);

      loop_counter++;
    }

    safe_dist_ = global_safe_dist;  // reset  former value

    std::vector<Node> waypoints;
    if (loop_counter < iter_limits - 1 || best.h_cost <= start_.h_cost + 1.1 * resolution_) {  // solution found
      waypoints.push_back(best);

      while (abs(waypoints.back().g_cost) > 1e-5) {
        waypoints.push_back(parents[waypoints.back()]);
        waypoints.back().pose = planning_octree_->keyToCoord(waypoints.back().key);
      }

      std::reverse(waypoints.begin(), waypoints.end());

      if (waypoints.size() > 0) {

        waypoints_filtered             = getFilteredNeighborhoodPlan(waypoints);
        waypoints_filtered.back().pose = planning_octree_->keyToCoord(waypoints_filtered.back().key);
        ROS_INFO("[%s]: Replacing original start [%.2f, %.2f, %.2f] by start in the free space [%.2f, %.2f, %.2f].", ros::this_node::getName().c_str(),
                 start_.pose.x(), start_.pose.y(), start_.pose.z(), waypoints_filtered.back().pose.x(), waypoints_filtered.back().pose.y(),
                 waypoints_filtered.back().pose.z());

        for (int k = 0; k < waypoints_filtered.size(); k++) {
          ROS_INFO("[%s]: escape path node [%d] = [ %.2f, %.2f, %.2f] ", ros::this_node::getName().c_str(), k, waypoints_filtered[k].pose.x(),
                   waypoints_filtered[k].pose.y(), waypoints_filtered[k].pose.z());
        }
      }
    }
  }

  ROS_INFO("[%s]: Getting path to nearest feasible node took %.3f", ros::this_node::getName().c_str(), (ros::Time::now() - start_time).toSec());
  return waypoints_filtered;
}

//}

/* getLastFoundGoal() //{ */
octomap::point3d AstarPlanner::getLastFoundGoal() {
  if (!initialized_ || !planning_octree_) {
    octomap::point3d point = {0, 0, 0};
    return point;
  }
  return planning_octree_->keyToCoord(last_found_goal_.key);
}
//}

/* getSafePath() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getSafePath(const std::vector<octomap::OcTreeKey>& key_path, double safe_dist, int max_iteration,
                                                          double z_diff_tolerance, bool fix_goal_point, bool horizontal_neighbors_only, double timeout) {
  /* bool visualization_pause_disabled = false; */
  ros::Time start_time                       = ros::Time::now();
  double    timeout_percentage_for_filtering = 0.25;

  ROS_INFO_COND(debug_, "[AstarPlanner]: GetSafePath start");
  std::vector<octomap::OcTreeKey> local_path_keys;
  local_path_keys = key_path;

  // check for zero path length
  if (key_path.size() < 2) {
    ROS_WARN("[AstarPlanner]: getSafePath receives too short path (length = %lu)", key_path.size());
    return local_path_keys;
  }

  // TODO: generate pointcloud for reasonable surrounding
  std::vector<int> map_limits = getMapLimits(key_path, 0, key_path.size(), ceil(max_iteration * sqrt(2)), ceil(max_iteration * sqrt(2)));
  ROS_INFO_COND(debug_, "[AstarPlanner]: octomap to pointcloud start");
  std::vector<pcl::PointXYZ> pcl_points =
      octomapToPointcloud(map_limits);  // TODO: replace by detection of maxmin x, maxmin y and maxmin z, for reasonable setting of area
  ROS_INFO_COND(debug_, "[AstarPlanner]: octomap to pointcloud ends");
  ros::Time end_time = ros::Time::now();
  ROS_WARN_COND(verbose_, "Octomap to pointcloud took %.2f ms", (end_time - start_time).toSec() * 1000.0);
  if (pcl_points.size() > 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_pointcloud = PCLMap::pclVectorToPointcloud(pcl_points);
    ROS_INFO_COND(debug_, "[AstarPlanner]: Map limits: x = [%d, %d], y = [%d, %d], z = [%d, %d]", map_limits[0], map_limits[1], map_limits[2], map_limits[3],
                  map_limits[4], map_limits[5]);
    ROS_INFO_COND(debug_, "[AstarPlanner]: init kd tree start");
    pcl_map_.initKDTreeSearch(simulated_pointcloud);
    ROS_INFO_COND(debug_, "[AstarPlanner]: init kd tree end");
  } else {
    return local_path_keys;
  }

  bool has_changed                    = false;
  bool is_current_key_already_in_plan = false;
  // initialize vector of keys
  /* std::vector<octomap::OcTreeKey> added_waypoints; */
  /* octomap::OcTreeKey              last_waypoint; */
  ROS_INFO_COND(debug_, "[AstarPlanner]: local_path_keys initialized");
  ros::Duration last_iter_duration = ros::Duration(0.0);
  for (int it = 0; it < max_iteration; it++) {

    if ((1 - timeout_percentage_for_filtering) * timeout - (ros::Time::now() - start_time).toSec() < last_iter_duration.toSec()) {
      ROS_INFO("[AstarPlanner]: Path postprocessing timeout reached at iteration %d", it);
      break;
    }

    ros::Time iter_start = ros::Time::now();

    ROS_INFO_COND(debug_, "[AstarPlanner]: Start iteration %d", it);
    /* visualization_pause_disabled = false; */
    std::vector<octomap::OcTreeKey> local_path_keys_next;
    local_path_keys_next.clear();
    local_path_keys_next.push_back(local_path_keys[0]);
    has_changed = false;

    for (uint k = 1; k < local_path_keys.size(); k++) {
      ROS_INFO_COND(debug_, "[AstarPlanner]: Local path key %d: [%d, %d, %d]", k, local_path_keys[k].k[0], local_path_keys[k].k[1], local_path_keys[k].k[2]);
      /* last_waypoint                  = local_path_keys_next.back(); */
      is_current_key_already_in_plan = false;

      for (int j = local_path_keys_next.size() - 1; j > fmax(0, local_path_keys_next.size() - ceil(1.0 / resolution_)); j--) {  // detection of similar nodes
        if (areKeysEqual(local_path_keys_next[j], local_path_keys[k])) {
          is_current_key_already_in_plan = true;
          break;
        }
      }

      if (is_current_key_already_in_plan) {
        ROS_INFO_COND(debug_, "[AstarPlanner]: Current key already in plan, continue to next key");
        continue;
      }

      // node in safe distance from obstacles:
      if (pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(local_path_keys[k])) > safe_dist) {

        if (areKeysEqual(local_path_keys_next.back(), local_path_keys[k])) {
          has_changed = true;
          ROS_INFO_COND(debug_, "[AstarPlanner]: Distance is safe and previous node is equal to current node. Nothing to add.");
        } else if (areKeysInNeighborhood(local_path_keys_next.back(), local_path_keys[k])) {
          local_path_keys_next.push_back(local_path_keys[k]);
          ROS_INFO_COND(debug_, "[AstarPlanner]: Distance is safe. add [%d, %d, %d] to next path keys", local_path_keys[k].k[0], local_path_keys[k].k[1],
                        local_path_keys[k].k[2]);
        } else {
          local_path_keys_next.push_back(getConnectionNode3d(local_path_keys_next.back(), local_path_keys[k]));
          local_path_keys_next.push_back(local_path_keys[k]);
          /* added_waypoints.clear(); */
          /* added_waypoints.push_back(getConnectionNode3d(local_path_keys_next.back(), local_path_keys[k])); */
          has_changed = true;
          ROS_INFO_COND(debug_, "[AstarPlanner]: Distance is safe, but nodes are not in the neighborhood, add [%d, %d, %d], [%d, %d, %d] to next path keys",
                        local_path_keys_next.back().k[0], local_path_keys_next.back().k[1], local_path_keys_next.back().k[2], local_path_keys[k].k[0],
                        local_path_keys[k].k[1], local_path_keys[k].k[2]);
        }
        continue;
      }

      /* octomap::OcTreeKey tmp_key = getBestNeighborEscape(local_path_keys[k], local_path_keys_next.back()); */
      octomap::OcTreeKey tmp_key = getBestNeighbor(local_path_keys[k], horizontal_neighbors_only);
      ROS_INFO_COND(debug_, "[AstarPlanner]: Dist is not safe, solving connection for [%d, %d, %d] and current [%d, %d, %d] with best neighbor [%d, %d, %d]",
                    local_path_keys_next.back().k[0], local_path_keys_next.back().k[1], local_path_keys_next.back().k[2], local_path_keys[k].k[0],
                    local_path_keys[k].k[1], local_path_keys[k].k[2], tmp_key.k[0], tmp_key.k[1], tmp_key.k[2]);
      if (areKeysEqual(tmp_key, local_path_keys_next.back())) {
        ROS_INFO_COND(debug_, "[AstarPlanner]: Current node is equal to previous node. Nothing to add.");
        continue;  // check correctness
      } else if (areKeysInNeighborhood(tmp_key, local_path_keys_next.back())) {
        local_path_keys_next.push_back(tmp_key);
        has_changed = true;
        ROS_INFO_COND(debug_, "[AstarPlanner]: Current node is in neighborhood of previous node. Adding the current node to local path.");

      } else {
        std::vector<octomap::OcTreeKey> additional_waypoints = getAdditionalWaypoints3d(local_path_keys_next.back(), tmp_key);
        /* added_waypoints                                      = additional_waypoints; */
        for (uint i = 0; i < additional_waypoints.size(); i++) {
          local_path_keys_next.push_back(additional_waypoints[i]);
        }
        local_path_keys_next.push_back(tmp_key);
        has_changed = true;
        ROS_INFO_COND(debug_ && (additional_waypoints.size() == 2),
                      "[AstarPlanner]: Nodes are unconnected, adding nodes [%d, %d, %d], [%d, %d, %d] and [%d, %d, %d] to local path",
                      additional_waypoints[0].k[0], additional_waypoints[0].k[1], additional_waypoints[0].k[2], additional_waypoints[1].k[0],
                      additional_waypoints[1].k[1], additional_waypoints[1].k[2], tmp_key.k[0], tmp_key.k[1], tmp_key.k[2]);
        ROS_INFO_COND(debug_ && (additional_waypoints.size() == 1),
                      "[AstarPlanner]: Nodes are unconnected, adding nodes [%d, %d, %d] and [%d, %d, %d] to local path", additional_waypoints[0].k[0],
                      additional_waypoints[0].k[1], additional_waypoints[0].k[2], tmp_key.k[0], tmp_key.k[1], tmp_key.k[2]);
      }
    }

    if (fix_goal_point) {
      if (!areKeysEqual(local_path_keys_next.back(), local_path_keys.back())) {
        if (areKeysInNeighborhood(local_path_keys.back(), local_path_keys_next.back())) {
          local_path_keys_next.push_back(local_path_keys.back());
          has_changed = true;
          ROS_INFO_COND(debug_, "[AstarPlanner]: Current node is in neighborhood of previous node. Adding the current node to local path.");
        } else {
          std::vector<octomap::OcTreeKey> additional_waypoints = getAdditionalWaypoints3d(local_path_keys_next.back(), local_path_keys.back());
          for (uint i = 0; i < additional_waypoints.size(); i++) {
            local_path_keys_next.push_back(additional_waypoints[i]);
          }
          local_path_keys_next.push_back(local_path_keys.back());
          has_changed = true;
        }
      }
    }

    ROS_INFO_COND(debug_, "[AstarPlanner]: First lpk size %lu = lpk next size = %lu", local_path_keys.size(), local_path_keys_next.size());
    /* local_path_keys = getFilteredNeighborhoodPlan(local_path_keys_next); */
    local_path_keys = local_path_keys_next;
    // copy local_path_keys_next
    /* local_path_keys.clear(); */
    /* for (uint k = 0; k < local_path_keys_next.size(); k++) { */
    /*   local_path_keys.push_back(local_path_keys_next[k]); */
    /* } */
    ROS_INFO_COND(debug_, "[AstarPlanner]: Second lpk size %lu = lpk next size = %lu", local_path_keys.size(), local_path_keys_next.size());
    if (!has_changed) {
      ROS_INFO_COND(verbose_, "[AstarPlanner]: No change detected -> safe path algorithm ended.");
      break;
    }
    ROS_WARN_COND(debug_, "[AstarPlanner]: ------------------------------------------------------------------------");
    for (uint i = 0; i < local_path_keys.size(); i++) {
      ROS_INFO_COND(debug_, "[AstarPlanner]: Safe path key %02d: [%d, %d, %d] ", i, local_path_keys[i].k[0], local_path_keys[i].k[1], local_path_keys[i].k[2]);
    }
    ROS_WARN_COND(debug_, "[AstarPlanner]: ------------------------------------------------------------------------");
    last_iter_duration = (ros::Time::now() - iter_start);
  }

  ROS_WARN_COND(debug_, "Get safe path took %.2f ms", (ros::Time::now() - start_time).toSec() * 1000.0);
  start_time      = ros::Time::now();
  local_path_keys = getFilteredNeighborhoodPlan(local_path_keys);
  local_path_keys = getStraightenKeyPath(local_path_keys);
  local_path_keys = getZzFilteredPlan(local_path_keys, z_diff_tolerance);
  ROS_WARN_COND(debug_, "Safe path filtering took %.2f ms", (ros::Time::now() - start_time).toSec() * 1000.0);

  return local_path_keys;
}
//}

/* getFilteredNeighborhoodPlan() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getFilteredNeighborhoodPlan(const std::vector<octomap::OcTreeKey>& original_path) {
  std::vector<octomap::OcTreeKey> new_path;
  bool                            point_added = false;
  int                             window_size = 14;
  new_path.push_back(original_path[0]);
  for (uint k = 0; k < original_path.size(); k++) {
    point_added = false;
    for (int m = fmin(window_size, original_path.size() - 1 - k); m >= 0; m--) {
      if (areKeysInNeighborhood(new_path.back(), original_path[k + m])) {
        new_path.push_back(original_path[k + m]);
        k           = k + m;  // FIxME: check whether it works as intended
        point_added = true;
        break;
      }
    }
    if (!point_added) {
      new_path.push_back(original_path[k]);
    }
  }
  return new_path;
}
//}

/* getFilteredNeighborhoodPlan() //{ */
std::vector<Node> AstarPlanner::getFilteredNeighborhoodPlan(const std::vector<Node>& original_path) {
  std::vector<Node> new_path;
  bool              point_added = false;
  int               window_size = 14;
  new_path.push_back(original_path[0]);
  for (uint k = 0; k < original_path.size(); k++) {
    point_added = false;
    for (int m = fmin(window_size, original_path.size() - 1 - k); m >= 0; m--) {
      if (areKeysInNeighborhood(new_path.back().key, original_path[k + m].key)) {
        new_path.push_back(original_path[k + m]);
        k           = k + m;  // FIxME: check whether it works as intended
        point_added = true;
        break;
      }
    }
    if (!point_added) {
      new_path.push_back(original_path[k]);
    }
  }
  return new_path;
}
//}

/* getZzFilteredPlan() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getZzFilteredPlan(const std::vector<octomap::OcTreeKey>& original_path, double tolerance) {
  std::vector<octomap::OcTreeKey> new_path;
  bool                            point_added = false;
  bool                            is_new_path_feasible;
  int                             window_size = 6;
  new_path.push_back(original_path[0]);
  std::vector<octomap::OcTreeKey> partial_result;
  double                          orig_dist, current_dist;
  octomap::OcTreeKey              tmp_key;
  for (size_t k = 0; k < original_path.size(); k++) {
    partial_result.clear();
    point_added = false;
    for (int m = fmin(window_size, original_path.size() - 1 - k); m >= 2; m--) {
      if (new_path.back().k[2] == original_path[k + m].k[2]) {  // FIXME: can be rewritten in more effective way
        partial_result.clear();
        is_new_path_feasible = true;
        for (int p = 0; p < m; p++) {
          tmp_key = original_path[k + p];
          if (original_path[k + p].k[2] != original_path[k + m].k[2]) {
            orig_dist    = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(tmp_key));
            tmp_key.k[2] = original_path[k + m].k[2];
            current_dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(tmp_key));
            if ((orig_dist - current_dist) > tolerance) {
              is_new_path_feasible = false;
              break;
            }
            partial_result.push_back(tmp_key);
          } else {
            partial_result.push_back(tmp_key);
          }
        }
        if (is_new_path_feasible) {
          for (size_t f = 0; f < partial_result.size(); f++) {
            new_path.push_back(partial_result[f]);
          }
          new_path.push_back(original_path[k + m]);
          k           = k + m;  // FIxME: check whether it works as intended
          point_added = true;
          break;
        }
      }
    }
    if (!point_added) {
      new_path.push_back(original_path[k]);
    }
  }
  return new_path;
}
//}

/* getStraightenKeyPath() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getStraightenKeyPath(const std::vector<octomap::OcTreeKey>& key_path) {
  std::vector<octomap::OcTreeKey> straighten_path;
  ROS_INFO_COND(debug_, "[AstarPlanner]: Start key path straightening.");
  if (key_path.size() < 3) {
    straighten_path = key_path;
    ROS_INFO_COND(debug_, "[AstarPlanner]: Path too short to straighten, returning original path.");
  } else {
    straighten_path.push_back(key_path[0]);
    for (uint i = 0; i < key_path.size() - 2; i++) {
      if (areKeysDiagonalNeighbors(key_path[i], key_path[i + 2])) {
        ROS_INFO_COND(debug_, "[AstarPlanner]: Keys [%d, %d, %d] and [%d, %d, %d] are diagonal neighbors.", key_path[i].k[0], key_path[i].k[1],
                      key_path[i].k[2], key_path[i + 2].k[0], key_path[i + 2].k[1], key_path[i + 2].k[2]);
        straighten_path.push_back(key_path[i + 2]);
        i++;  // skip i+1 key
      } else {
        ROS_INFO_COND(debug_, "[AstarPlanner]: Keys [%d, %d, %d] and [%d, %d, %d] are not diagonal neighbors.", key_path[i].k[0], key_path[i].k[1],
                      key_path[i].k[2], key_path[i + 2].k[0], key_path[i + 2].k[1], key_path[i + 2].k[2]);
        straighten_path.push_back(key_path[i + 1]);
      }
    }
  }
  return straighten_path;
}
//}

/* pruneWaypoints() //{ */
std::vector<octomap::point3d> AstarPlanner::pruneWaypoints(std::vector<octomap::point3d>& waypoint_path, double pruning_dist) {

  if (waypoint_path.size() < 3) {
    ROS_WARN("[AstarPlanner]: Nothing to prune. Returning original path.");
    return waypoint_path;
  }

  std::vector<octomap::point3d> pruned_path;
  pruned_path.push_back(waypoint_path[0]);
  for (size_t k = 1; k < waypoint_path.size() - 1; k++) {
    if (sqrt(pow(waypoint_path[k].x() - pruned_path.back().x(), 2) + pow(waypoint_path[k].y() - pruned_path.back().y(), 2) +
             pow(waypoint_path[k].z() - pruned_path.back().z(), 2)) > pruning_dist) {
      pruned_path.push_back(waypoint_path[k]);
    }
  }

  pruned_path.push_back(waypoint_path.back());

  return pruned_path;
}
//}

/* getWaypointPathWithoutObsoletePoints() //{ */
std::vector<octomap::point3d> AstarPlanner::getWaypointPathWithoutObsoletePoints(std::vector<octomap::point3d>& waypoint_path, double tolerance) {

  if (waypoint_path.size() < 3) {
    ROS_WARN("[AstarPlanner]: No obsolete points. Returning original path.");
    return waypoint_path;
  }

  std::vector<octomap::point3d> simplified_path;
  simplified_path.push_back(waypoint_path[0]);
  for (size_t k = 1; k < waypoint_path.size() - 1; k++) {
    if (pointLineDist(waypoint_path[k - 1], waypoint_path[k + 1], waypoint_path[k]) > tolerance) {
      simplified_path.push_back(waypoint_path[k]);
    }
  }

  simplified_path.push_back(waypoint_path.back());

  return simplified_path;
}
//}

/* pointLineDist //{ */
double AstarPlanner::pointLineDist(octomap::point3d lb, octomap::point3d le, octomap::point3d p) {
  // implemented based on https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
  octomap::point3d v_line(le.x() - lb.x(), le.y() - lb.y(), le.z() - lb.z());
  octomap::point3d v_point(lb.x() - p.x(), lb.y() - p.y(), lb.z() - p.z());
  double           line_norm_sq  = pow(v_line.x(), 2) + pow(v_line.y(), 2) + pow(v_line.z(), 2);
  double           point_norm_sq = pow(v_point.x(), 2) + pow(v_point.y(), 2) + pow(v_point.z(), 2);
  double dist_sq = (line_norm_sq * point_norm_sq - pow(v_line.x() * v_point.x() + v_line.y() * v_point.y() + v_line.z() * v_point.z(), 2)) / line_norm_sq;
  return sqrt(dist_sq);
}
//}

/* getStraightenWaypointPath() //{ */
std::vector<octomap::point3d> AstarPlanner::getStraightenWaypointPath(std::vector<Node>& node_path, double dist_step) {
  std::vector<octomap::point3d> waypoints;
  if (node_path.size() < 2) {
    ROS_WARN("[AstarPlanner]: AstarPlanner: Empty node path received. Returning empty plan.");
    return waypoints;
  }

  double dx, dy, dz, dl;
  size_t s, e;
  int    steps;
  bool   is_collision_free;
  waypoints.push_back(node_path.front().pose);

  for (s = 0; s < node_path.size(); s++) {
    for (e = node_path.size() - 1; e > s; e--) {
      dx                = node_path[e].pose.x() - node_path[s].pose.x();
      dy                = node_path[e].pose.y() - node_path[s].pose.y();
      dz                = node_path[e].pose.z() - node_path[s].pose.z();
      dl                = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
      steps             = ceil(dl / dist_step);
      is_collision_free = true;

      for (int k = 0; k < steps; k++) {
        octomap::point3d point;
        if (!checkValidityWithNeighborhood(planning_octree_->coordToKey(node_path[s].pose.x() + k * dx / steps, node_path[s].pose.y() + k * dy / steps,
                                                                        node_path[s].pose.z() + k * dz / steps))) {
          is_collision_free = false;
          break;
        }
      }

      if (is_collision_free) {
        break;
      }
    }
    waypoints.push_back(node_path[e].pose);
    s = e;
  }

  return waypoints;
}
//}

/* getFilteredPlan() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getFilteredPlan(const std::vector<octomap::OcTreeKey>& original_path, int window_size,
                                                              double enabled_filtering_dist) {
  std::vector<octomap::OcTreeKey> new_path;
  if (original_path.size() == 0) {
    ROS_WARN("[AstarPlanner]: Empty path received, returning empty filtered path.");
    return new_path;
  }
  for (uint k = 0; k < original_path.size(); k++) {
    ROS_INFO_COND(debug_, "[AstarPlanner]: Filtered path [%d] = [%d, %d, %d]", k, original_path[k].k[0], original_path[k].k[1], original_path[k].k[2]);
  }
  ROS_INFO_COND(debug_, "[AstarPlanner]: Filtering: Last point of original_path = [%d, %d, %d] ", original_path[original_path.size() - 1].k[0],
                original_path[original_path.size() - 1].k[1], original_path[original_path.size() - 1].k[2]);
  new_path.push_back(original_path[0]);
  bool point_added = false;
  for (uint k = 0; k < original_path.size(); k++) {
    point_added = false;
    for (int m = fmin(window_size, original_path.size() - 1 - k); m >= 0; m--) {
      if (keyEuclideanDist(new_path[new_path.size() - 1], original_path[k + m]) < enabled_filtering_dist) {
        new_path.push_back(original_path[k + m]);
        k           = k + m;  // FIxME: check whether it works as intended
        point_added = true;
        break;
      }
    }
    if (!point_added) {
      new_path.push_back(original_path[k]);
    }
  }
  for (uint k = 0; k < new_path.size(); k++) {
    ROS_INFO_COND(debug_, "[AstarPlanner]: Filtered path [%d] = [%d, %d, %d]", k, new_path[k].k[0], new_path[k].k[1], new_path[k].k[2]);
  }
  ROS_INFO_COND(debug_, "[AstarPlanner]: Filtering: Last point of new_path = [%d, %d, %d] ", new_path[new_path.size() - 1].k[0],
                new_path[new_path.size() - 1].k[1], new_path[new_path.size() - 1].k[2]);
  return new_path;
}
//}

/* getWaypointPath() //{ */
std::vector<octomap::point3d> AstarPlanner::getWaypointPath(const std::vector<Node>& node_path) {
  std::vector<octomap::point3d> waypoints;
  if (node_path.empty()) {
    ROS_WARN("[AstarPlanner]: AstarPlanner: Empty node path received. Returning empty plan.");
    return waypoints;
  }
  for (auto node : node_path) {
    waypoints.push_back(node.pose);
  }
  return waypoints;
}
//}

/* getWaypointPath() //{ */
std::vector<octomap::point3d> AstarPlanner::getWaypointPath(const std::vector<octomap::OcTreeKey>& key_path) {
  std::vector<octomap::point3d> waypoints;
  if (key_path.empty()) {
    ROS_WARN("[AstarPlanner]: AstarPlanner: Empty key path received. Returning empty plan.");
    return waypoints;
  }
  for (auto key : key_path) {
    waypoints.push_back(planning_octree_->keyToCoord(key));
  }
  return waypoints;
}
//}

/* getKeyVectorFromCoordinates() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getKeyVectorFromCoordinates(const std::vector<geometry_msgs::Point>& pose_array) {
  std::vector<octomap::OcTreeKey> key_path;
  for (uint k = 0; k < pose_array.size(); k++) {
    key_path.push_back(octomap::OcTreeKey(pose_array[k].x, pose_array[k].y, pose_array[k].z));
  }
  return key_path;
}
//}

/* firstUnfeasibleNodeInPath() //{ */
std::pair<int, int> AstarPlanner::firstUnfeasibleNodeInPath(const std::vector<octomap::OcTreeKey>&   key_waypoints,
                                                            const std::vector<geometry_msgs::Point>& pose_array, int n_points_forward,
                                                            const octomap::point3d& current_pose, double safe_dist_for_replanning,
                                                            double critical_dist_for_replanning) {
  ROS_INFO_COND(debug_, "[AstarPlanner]: First unfeasible node in path start.");
  std::pair<int, int> result;
  result.first         = -1;
  result.second        = -1;
  int current_pose_idx = 0;
  for (uint k = 0; k < key_waypoints.size(); k++) {  // FIXME add saving previously found starting index to avoid iterating over whole path
    octomap::point3d point = planning_octree_->keyToCoord(key_waypoints[k]);
    if (sqrt(pow(point.x() - current_pose.x(), 2) + pow(point.y() - current_pose.y(), 2) + pow(point.z() - current_pose.z(), 2)) < 0.8) {
      current_pose_idx = k;
      break;
    }
  }
  ROS_INFO_COND(debug_, "[AstarPlanner]: Current pose idx found.");
  uint                       end_index  = fmin(current_pose_idx + n_points_forward, key_waypoints.size());
  std::vector<int>           map_limits = getMapLimits(key_waypoints, current_pose_idx, end_index, ceil(2.0 / resolution_), ceil(2.0 / resolution_));
  std::vector<pcl::PointXYZ> pcl_points =
      octomapToPointcloud(map_limits);  // TODO: replace by detection of maxmin x, maxmin y and maxmin z, for reasonable setting of are
  if (pcl_points.size() > 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr simulated_pointcloud = PCLMap::pclVectorToPointcloud(pcl_points);

    ROS_INFO_COND(true, "[AstarPlanner]: Map limits: x = [%d, %d], y = [%d, %d], z = [%d, %d]", map_limits[0], map_limits[1], map_limits[2], map_limits[3],
                  map_limits[4], map_limits[5]);
    ROS_INFO_COND(debug_, "[AstarPlanner]: init kd tree start");
    pcl_map_.initKDTreeSearch(simulated_pointcloud);
    ROS_INFO_COND(debug_, "[AstarPlanner]: init kd tree end");

    /* for (uint k = current_pose_idx; k < end_index; k++) { */
    /*   Node n; */
    /*   n.key = key_waypoints[k]; */
    /*   if (pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(key_waypoints[k])) < safe_dist_for_replanning) { */
    /*     result.second = k; */
    /*     break; */
    /*   } */
    /* } */

    std::vector<octomap::OcTreeKey> predicted_trajectory_keys = getKeyVectorFromCoordinates(pose_array);
    double                          dist;
    bool                            first_key_set = false;
    for (uint k = 0; k < predicted_trajectory_keys.size(); k++) {
      dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(predicted_trajectory_keys[k]));
      if (!first_key_set && dist < safe_dist_for_replanning) {
        result.first  = k;
        first_key_set = true;
      }
      if (first_key_set && dist < critical_dist_for_replanning) {
        result.second = k;
        break;
      }
    }
  }

  return result;
}
//}

/* octomapToPointcloud() //{ */
std::vector<pcl::PointXYZ> AstarPlanner::octomapToPointcloud(const std::vector<int>& map_limits) {
  std::vector<pcl::PointXYZ> output_pcl;
  ROS_INFO_COND(debug_, "[AstarPlanner]: octomap to pointcloud start, x = [%d, %d], y = [%d, %d], z = [%d, %d]", map_limits[0], map_limits[1], map_limits[2],
                map_limits[3], map_limits[4], map_limits[5]);
  for (int x = map_limits[0]; x <= map_limits[1]; x++) {
    for (int y = map_limits[2]; y <= map_limits[3]; y++) {
      for (int z = map_limits[4]; z <= map_limits[5]; z++) {
        /* ROS_INFO("[AstarPlanner]: x = %d, y = %d, z = %d", x, y, z); */
        pcl::PointXYZ      point;
        octomap::OcTreeKey tmp_key;
        tmp_key.k[0] = x;
        tmp_key.k[1] = y;
        tmp_key.k[2] = z;
        if (planning_octree_->search(tmp_key) == NULL ||
            (planning_octree_->search(tmp_key) != NULL && planning_octree_->isNodeOccupied(planning_octree_->search(tmp_key)))) {
          octomap::point3d octomap_point = planning_octree_->keyToCoord(tmp_key);
          point.x                        = octomap_point.x();
          point.y                        = octomap_point.y();
          point.z                        = octomap_point.z();
          output_pcl.push_back(point);
        }
      }
    }
  }
  ROS_INFO_COND(debug_, "[AstarPlanner]: octomap to pointcloud end");
  return output_pcl;
}
//}

/* octomapToPointcloud() //{ */
std::vector<pcl::PointXYZ> AstarPlanner::octomapToPointcloud() {
  std::vector<pcl::PointXYZ> output_pcl;

  for (octomap::OcTree::leaf_iterator it = planning_octree_->begin_leafs(), end = planning_octree_->end_leafs(); it != end; ++it) {

    if (planning_octree_->search(it.getKey()) == NULL ||
        (planning_octree_->search(it.getKey()) != NULL && planning_octree_->isNodeOccupied(planning_octree_->search(it.getKey())))) {

      if (it.getDepth() == planning_octree_->getTreeDepth()) {

        pcl::PointXYZ    point;
        octomap::point3d octomap_point = planning_octree_->keyToCoord(it.getKey());
        point.x                        = octomap_point.x();
        point.y                        = octomap_point.y();
        point.z                        = octomap_point.z();
        output_pcl.push_back(point);

      } else {

        // solution for leafs with non-maximum depth
        double           sd            = it.getSize() / 2.0;
        double           ed            = it.getSize() / 2.0;
        octomap::point3d octomap_point = planning_octree_->keyToCoord(it.getKey());
        for (double x = -sd; x < ed; x += resolution_) {
          for (double y = -sd; y < ed; y += resolution_) {
            for (double z = -sd; z < ed; z += resolution_) {
              pcl::PointXYZ point;
              point.x = octomap_point.x() + x;
              point.y = octomap_point.y() + y;
              point.z = octomap_point.z() + z;
              output_pcl.push_back(point);
            }
          }
        }
      }
    }
  }

  ROS_INFO_COND(debug_, "[AstarPlanner]: octomap to pointcloud end");
  return output_pcl;
}
//}

/* getKeyPath() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getKeyPath(const std::vector<Node>& plan) {
  std::vector<octomap::OcTreeKey> key_path;
  for (uint k = 0; k < plan.size(); k++) {
    key_path.push_back(plan[k].key);
  }
  return key_path;
}
//}

/* getMapLimits() //{ */
std::vector<int> AstarPlanner::getMapLimits(const std::vector<octomap::OcTreeKey>& plan, int start_idx, int end_idx, int xy_reserve, int z_reserve) {
  std::vector<int>   map_limits(6);
  int                max_abs = INT_MAX;  // FIXME: replace by detection of maximum index in octomap
  int                min_x   = INT_MAX;
  int                max_x   = INT_MIN;
  int                min_y   = INT_MAX;
  int                max_y   = INT_MIN;
  int                min_z   = INT_MAX;
  int                max_z   = INT_MIN;
  octomap::OcTreeKey key;
  for (int k = start_idx; k < end_idx; k++) {
    key = plan[k];
    if (key[0] < min_x) {
      min_x = key[0];
    }
    if (key[0] > max_x) {
      max_x = key[0];
    }
    if (key[1] < min_y) {
      min_y = key[1];
    }
    if (key[1] > max_y) {
      max_y = key[1];
    }
    if (key[2] < min_z) {
      min_z = key[2];
    }
    if (key[2] > max_z) {
      max_z = key[2];
    }
  }
  map_limits[0] = fmax(min_x - xy_reserve, 0);
  map_limits[1] = fmin(max_x + xy_reserve, max_abs);
  map_limits[2] = fmax(min_y - xy_reserve, 0);
  map_limits[3] = fmin(max_y + xy_reserve, max_abs);
  map_limits[4] = fmax(min_z - z_reserve, 0);
  map_limits[5] = fmin(max_z + z_reserve, max_abs);
  return map_limits;
}
//}

/* getAdditionalWaypoints() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getAdditionalWaypoints(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  int                             nof_diff_coords = nofDifferentKeyCoordinates(k1, k2);
  std::vector<octomap::OcTreeKey> additional_waypoints;
  std::vector<int>                diff_coords    = getDifferenceInCoordinates(k1, k2);
  int                             manhattan_dist = 0;
  octomap::OcTreeKey              res1, res2;
  switch (nof_diff_coords) {
    case 1: {
      for (int i = 0; i < 3; i++) {
        if (diff_coords[i] != 0) {
          res1.k[i]      = k1.k[i] + ((k1.k[i] < k2.k[i]) - (k1.k[i] > k2.k[i]));
          res2.k[i]      = k1.k[i] + 2 * ((k1.k[i] < k2.k[i]) - (k1.k[i] > k2.k[i]));
          manhattan_dist = abs(k1.k[i] - k2.k[i]);
        } else {
          res1.k[i] = k1.k[i];
          res2.k[i] = k1.k[i];
        }
      }
      additional_waypoints.push_back(res1);
      if (manhattan_dist == 3) {
        additional_waypoints.push_back(res2);
      }
    } break;
    case 2: {
      additional_waypoints = getSafestWaypointsBetweenKeys(getPossibleWaypointsForTwoDiffCoord(k1, k2));
    } break;
    case 3: {
      additional_waypoints = getSafestWaypointsBetweenKeys(getPossibleWaypointsForThreeDiffCoord(k1, k2));
    } break;
    default: {
      ROS_ERROR("[AstarPlanner]: Number of different coords = %d outside expected range. ", nof_diff_coords);
    }
  }
  return additional_waypoints;
}
//}

/* getAdditionalWaypoints3d() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getAdditionalWaypoints3d(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  // k1 = from, k2 = to
  octomap::OcTreeKey                           dir, diff, r1, r2;
  std::vector<octomap::OcTreeKey>              partial_results;
  std::vector<std::vector<octomap::OcTreeKey>> results;
  diff.k[0] = abs(k2.k[0] - k1.k[0]);
  diff.k[1] = abs(k2.k[1] - k1.k[1]);
  diff.k[2] = abs(k2.k[2] - k1.k[2]);
  /* if (diff.k[0] < 2 && diff.k[1] < 2 && diff.k[2] < 2) { */
  /*   return getConnectionNode3d(k1, k2); */
  /* } */
  int                           dir_0 = k2.k[0] - k1.k[0] == 0 ? 0 : k2.k[0] - k1.k[0] > 0 ? 1 : -1;
  int                           dir_1 = k2.k[1] - k1.k[1] == 0 ? 0 : k2.k[1] - k1.k[1] > 0 ? 1 : -1;
  int                           dir_2 = k2.k[2] - k1.k[2] == 0 ? 0 : k2.k[2] - k1.k[2] > 0 ? 1 : -1;
  std::vector<std::vector<int>> i0    = diff.k[0] == 0 ? idxs4 : diff.k[0] == 1 ? idxs3 : diff.k[0] == 2 ? idxs2 : idxs1;
  std::vector<std::vector<int>> i1    = diff.k[1] == 0 ? idxs4 : diff.k[1] == 1 ? idxs3 : diff.k[1] == 2 ? idxs2 : idxs1;
  std::vector<std::vector<int>> i2    = diff.k[2] == 0 ? idxs4 : diff.k[2] == 1 ? idxs3 : diff.k[2] == 2 ? idxs2 : idxs1;
  for (size_t j = 0; j < i0.size(); j++) {
    r1.k[0] = k2.k[0] + dir_0 * i0[j][0];
    r2.k[0] = r1.k[0] + dir_0 * i0[j][1];
    for (size_t k = 0; k < i1.size(); k++) {
      r1.k[1] = k2.k[1] + dir_1 * i1[k][0];
      r2.k[1] = r1.k[1] + dir_1 * i1[k][1];
      for (size_t l = 0; l < i2.size(); l++) {
        r1.k[2] = k2.k[2] + dir_2 * i2[l][0];
        r2.k[2] = r1.k[2] + dir_2 * i2[l][1];
        partial_results.clear();
        partial_results.push_back(r2);
        partial_results.push_back(r1);
        results.push_back(partial_results);
      }
    }
  }
  /* partial_results = getSafestWaypointsBetweenKeys(results); */
  return getSafestWaypointsBetweenKeys(results);
}
//}

/* getSafestWaypointsBetweenKeys() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getSafestWaypointsBetweenKeys(const std::vector<std::vector<octomap::OcTreeKey>>& possible_waypoints) {
  int    best_idx = 0;
  double max_dist = -1.0;
  double dist;
  for (uint i = 0; i < possible_waypoints.size(); i++) {
    dist = DBL_MAX;
    for (uint j = 0; j < possible_waypoints[i].size(); j++) {
      dist = fmin(pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(possible_waypoints[i][j])), dist);
    }
    if (dist > max_dist) {
      max_dist = dist;
      best_idx = i;
    }
  }
  return possible_waypoints[best_idx];
}
//}

/* getSafestWaypointsBetweenKeys() //{ */
octomap::OcTreeKey AstarPlanner::getSafestWaypointsBetweenKeys(const std::vector<octomap::OcTreeKey>& possible_waypoints) {
  int    best_idx = 0;
  double max_dist = -1.0;
  double dist     = DBL_MAX;
  for (uint i = 0; i < possible_waypoints.size(); i++) {
    dist = fmin(pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(possible_waypoints[i])), dist);
    if (dist > max_dist) {
      max_dist = dist;
      best_idx = i;
    }
  }
  return possible_waypoints[best_idx];
}
//}

/* getPossibleWaypointsForTwoDiffCoord() //{ */
std::vector<std::vector<octomap::OcTreeKey>> AstarPlanner::getPossibleWaypointsForTwoDiffCoord(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  std::vector<std::vector<octomap::OcTreeKey>> result;
  std::vector<int>                             diff_in_coords = getDifferenceInCoordinates(k1, k2);
  // add solution for diagonaly connected nodes with manhattan distance equal to two
  if ((abs(k1.k[0] - k2.k[0]) + abs(k1.k[1] - k2.k[1]) + abs(k1.k[2] - k2.k[2])) == 2) {
    result.resize(2);
    for (uint i = 0; i < result.size(); i++) {
      result[i].resize(1);
    }
    bool switch_keys = false;
    for (int m = 0; m < 3; m++) {
      if (diff_in_coords[m] == 0) {
        for (uint i = 0; i < result.size(); i++) {
          for (uint j = 0; j < result[i].size(); j++) {
            result[i][j].k[m] = k1.k[m];
          }
        }
      } else if (diff_in_coords[m] == 1) {
        if (switch_keys) {
          result[0][0].k[m] = k1.k[m];
          result[1][0].k[m] = k2.k[m];
          switch_keys       = false;
        } else {
          result[0][0].k[m] = k2.k[m];
          result[1][0].k[m] = k1.k[m];
          switch_keys       = true;
        }
      } else {
        ROS_WARN("[AstarPlanner]: Something is crazy: diff in coords %d = %d", m, diff_in_coords[m]);
      }
    }
  } else {
    result.resize(3);
    for (uint i = 0; i < result.size(); i++) {
      result[i].resize(2);
    }
    for (int m = 0; m < 3; m++) {
      if (diff_in_coords[m] == 0) {
        for (uint i = 0; i < result.size(); i++) {
          for (uint j = 0; j < 2; j++) {
            result[i][j].k[m] = k1.k[m];
          }
        }
      } else if (diff_in_coords[m] == 1) {
        result[0][0].k[m] = k1.k[m];
        result[0][1].k[m] = k1.k[m];
        result[1][0].k[m] = k1.k[m];
        result[1][1].k[m] = k1.k[m] + (k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]);
        result[2][0].k[m] = k1.k[m] + (k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]);
        result[2][1].k[m] = k1.k[m] + (k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]);
      } else {
        result[0][0].k[m] = k1.k[m] + (k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]);
        result[0][1].k[m] = k1.k[m] + 2 * ((k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]));
        result[1][0].k[m] = k1.k[m] + (k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]);
        result[1][1].k[m] = k1.k[m] + (k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]);
        result[2][0].k[m] = k1.k[m];
        result[2][1].k[m] = k1.k[m] + (k2.k[m] > k1.k[m]) - (k2.k[m] < k1.k[m]);
      }
    }
  }
  return result;
}
//}

/* getPossibleWaypointsForThreeDiffCoord() //{ */
std::vector<std::vector<octomap::OcTreeKey>> AstarPlanner::getPossibleWaypointsForThreeDiffCoord(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  std::vector<std::vector<octomap::OcTreeKey>> result;
  result.resize(6);
  for (uint i = 0; i < result.size(); i++) {
    result[i].resize(2);
  }
  result[0][0].k[0] = k1.k[0] + (k2.k[0] > k1.k[0]) - (k2.k[0] < k1.k[0]);
  result[0][0].k[1] = k1.k[1];
  result[0][0].k[2] = k1.k[2];
  result[0][1].k[0] = k1.k[0] + (k2.k[0] > k1.k[0]) - (k2.k[0] < k1.k[0]);
  result[0][1].k[1] = k1.k[1] + (k2.k[1] > k1.k[1]) - (k2.k[1] < k1.k[1]);
  result[0][1].k[2] = k1.k[2];
  result[1][0]      = result[0][0];
  result[1][1].k[0] = k1.k[0] + (k2.k[0] > k1.k[0]) - (k2.k[0] < k1.k[0]);
  result[1][1].k[1] = k1.k[1];
  result[1][1].k[2] = k1.k[2] + (k2.k[2] > k1.k[2]) - (k2.k[2] < k1.k[2]);
  result[2][0].k[0] = k1.k[0];
  result[2][0].k[1] = k1.k[1] + (k2.k[1] > k1.k[1]) - (k2.k[1] < k1.k[1]);
  result[2][0].k[2] = k1.k[2];
  result[2][1]      = result[0][1];
  result[3][0]      = result[2][0];
  result[3][1].k[0] = k1.k[0];
  result[3][1].k[1] = k1.k[1] + (k2.k[1] > k1.k[1]) - (k2.k[1] < k1.k[1]);
  result[3][1].k[2] = k1.k[2] + (k2.k[2] > k1.k[2]) - (k2.k[2] < k1.k[2]);
  result[4][0].k[0] = k1.k[0];
  result[4][0].k[1] = k1.k[1];
  result[4][0].k[2] = k1.k[2] + (k2.k[2] > k1.k[2]) - (k2.k[2] < k1.k[2]);
  result[4][1]      = result[1][1];
  result[5][0]      = result[4][0];
  result[5][1]      = result[3][1];
  return result;
}
//}

/* getConnectionNode() //{ */
octomap::OcTreeKey AstarPlanner::getConnectionNode(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  // returns best connection node for nodes with manhattan distance = 1
  octomap::OcTreeKey result_1;
  octomap::OcTreeKey result_2;
  if (k1.k[0] == k2.k[0]) {
    result_1.k[0] = k1.k[0];
    result_2.k[0] = k1.k[0];
    result_1.k[1] = k1.k[1];
    result_1.k[2] = k2.k[2];
    result_2.k[1] = k2.k[1];
    result_2.k[2] = k1.k[2];
  } else if (k1.k[1] == k2.k[1]) {
    result_1.k[1] = k1.k[1];
    result_2.k[1] = k1.k[1];
    result_1.k[0] = k1.k[0];
    result_1.k[2] = k2.k[2];
    result_2.k[0] = k2.k[0];
    result_2.k[2] = k1.k[2];
  } else {
    result_1.k[2] = k1.k[2];
    result_2.k[2] = k1.k[2];
    result_1.k[0] = k1.k[0];
    result_1.k[1] = k2.k[1];
    result_2.k[0] = k2.k[0];
    result_2.k[1] = k1.k[1];
  }
  double obs_dist_1 = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(result_1));
  double obs_dist_2 = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(result_2));
  return obs_dist_1 > obs_dist_2 ? result_1 : result_2;
}
//}

/* getConnectionNode3d() //{ */
octomap::OcTreeKey AstarPlanner::getConnectionNode3d(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  // returns best connection node for nodes with manhattan distance = 1
  octomap::OcTreeKey              dir, diff, r1;
  std::vector<octomap::OcTreeKey> results;
  diff.k[0]              = abs(k2.k[0] - k1.k[0]);
  diff.k[1]              = abs(k2.k[1] - k1.k[1]);
  diff.k[2]              = abs(k2.k[2] - k1.k[2]);
  int              dir_0 = k2.k[0] - k1.k[0] == 0 ? 0 : k2.k[0] - k1.k[0] > 0 ? 1 : -1;
  int              dir_1 = k2.k[1] - k1.k[1] == 0 ? 0 : k2.k[1] - k1.k[1] > 0 ? 1 : -1;
  int              dir_2 = k2.k[2] - k1.k[2] == 0 ? 0 : k2.k[2] - k1.k[2] > 0 ? 1 : -1;
  std::vector<int> i0    = diff.k[0] == 0 ? idxs3d : diff.k[0] == 1 ? idxs2d : idxs1d;
  std::vector<int> i1    = diff.k[1] == 0 ? idxs3d : diff.k[1] == 1 ? idxs2d : idxs1d;
  std::vector<int> i2    = diff.k[2] == 0 ? idxs3d : diff.k[2] == 1 ? idxs2d : idxs1d;
  for (size_t j = 0; j < i0.size(); j++) {
    r1.k[0] = k2.k[0] + dir_0 * i0[j];
    for (size_t k = 0; k < i1.size(); k++) {
      r1.k[1] = k2.k[1] + dir_1 * i1[k];
      for (size_t l = 0; l < i2.size(); l++) {
        r1.k[2] = k2.k[2] + dir_2 * i2[l];
        results.push_back(r1);
      }
    }
  }
  return getSafestWaypointsBetweenKeys(results);
}
//}

/* findConnection() //{ */
octomap::OcTreeKey AstarPlanner::findConnection(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2, double original_obs_dist) {
  double             eucl_dist = sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));
  double             obs_dist;
  octomap::OcTreeKey connection_waypoint;
  connection_waypoint.k[0] = grid_params_.width + 1;  // used for detection of no better waypoint found
  if (fabs(eucl_dist - 2) < 1e-5) {
    octomap::OcTreeKey res;
    res.k[0] = (k1.k[0] + k2.k[0]) / 2;
    res.k[1] = (k1.k[1] + k2.k[1]) / 2;
    res.k[2] = (k1.k[2] + k2.k[2]) / 2;
    obs_dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(res));
    if (obs_dist >= original_obs_dist) {
      connection_waypoint = res;
    }
  } else if (fabs(eucl_dist - sqrt(5)) < 1e-5) {
    octomap::OcTreeKey res1;
    octomap::OcTreeKey res2;
    for (int i = 0; i < 3; i++) {
      switch (abs(k1.k[i] - k2.k[i])) {
        case 0:
          res1.k[i] = k1.k[i];
          res2.k[i] = k1.k[i];
          break;
        case 1:
          res1.k[i] = k1.k[i];
          res2.k[i] = k2.k[i];
          break;
        case 2:
          res1.k[i] = (k1.k[i] + k2.k[i]) / 2;
          res2.k[i] = (k1.k[i] + k2.k[i]) / 2;
          break;
        default:
          ROS_WARN("[AstarPlanner]: Unexpected number in eucliden dist of keys.");
      }
      double obs_dist_1 = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(res1));
      double obs_dist_2 = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(res2));
      if (obs_dist_1 > obs_dist_2) {
        if (obs_dist_1 > original_obs_dist) {
          connection_waypoint = res1;
        }
      } else {
        if (obs_dist_2 > original_obs_dist) {
          connection_waypoint = res2;
        }
      }
    }
  } else {
    std::vector<octomap::OcTreeKey> possible_connection_keys = generatePossibleConnectionsDiagonalKeys(k1, k2);
    double                          max_dist                 = original_obs_dist;
    for (std::vector<octomap::OcTreeKey>::iterator it = possible_connection_keys.begin(); it != possible_connection_keys.end(); ++it) {
      obs_dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(*it));
      if (obs_dist > max_dist) {
        connection_waypoint = *it;
        max_dist            = obs_dist;
      }
    }
  }
  return connection_waypoint;
}
//}

/* generatePossibleConnectionsDiagonalKeys() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::generatePossibleConnectionsDiagonalKeys(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  std::vector<octomap::OcTreeKey> keys;
  for (int i = 0; i < 3; i++) {
    octomap::OcTreeKey tmp_key = k1;
    tmp_key.k[i]               = k2.k[i];
    keys.push_back(tmp_key);
  }
  for (int i = 0; i < 3; i++) {
    octomap::OcTreeKey tmp_key = k2;
    tmp_key.k[i]               = k1.k[i];
    keys.push_back(tmp_key);
  }
  return keys;
}
//}

/* getBestNeighbor() //{ */
octomap::OcTreeKey AstarPlanner::getBestNeighbor(const octomap::OcTreeKey& c, bool horizontal_neighbors_only) {
  std::vector<octomap::OcTreeKey> neighbors = horizontal_neighbors_only ? getKeyNeighborhood8(c) : getKeyNeighborhood26(c);
  double                          obs_dist;
  double                          max_dist          = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(c));
  octomap::OcTreeKey              best_neighbor_key = c;
  for (std::vector<octomap::OcTreeKey>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
    obs_dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(*it));  // nodeDistance can be replaced with 1.0 for 6 neighborhood
    if (abs(obs_dist - max_dist) < 1e-5) {
      if (getDistFactorOfNeighbors(*it) < getDistFactorOfNeighbors(best_neighbor_key)) {
        best_neighbor_key = *it;
        max_dist          = obs_dist;
      }
    } else if ((obs_dist - max_dist) > 0) {
      best_neighbor_key = *it;
      max_dist          = obs_dist;
    }
  }
  /* ROS_INFO_COND(debug_, "[AstarPlanner]: Get best neighbor: current dist = %.2f, max_dist = %.2f", current_dist, max_dist); */
  return best_neighbor_key;
}
//}

/* getBestNeighborEscape() //{ */
octomap::OcTreeKey AstarPlanner::getBestNeighborEscape(const octomap::OcTreeKey& c, const octomap::OcTreeKey& prev) {
  std::vector<octomap::OcTreeKey> neighbors = getKeyNeighborhood26(c);
  double                          obs_dist;
  double                          max_dist          = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(c));
  octomap::OcTreeKey              best_neighbor_key = c;
  for (std::vector<octomap::OcTreeKey>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
    obs_dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(*it));  // nodeDistance can be replaced with 1.0 for 6 neighborhood
    if (abs(obs_dist - max_dist) < 1e-5) {
      if (keyEuclideanDist(prev, *it) < keyEuclideanDist(prev, c)) {
        best_neighbor_key = *it;
        max_dist          = obs_dist;
      }
      /* if (getDistFactorOfNeighbors(*it) > getDistFactorOfNeighbors(best_neighbor_key)) { */
      /*   best_neighbor_key = *it; */
      /*   max_dist          = obs_dist; */
      /* } */
    } else if ((obs_dist - max_dist) > 0) {
      best_neighbor_key = *it;
      max_dist          = obs_dist;
    }
  }
  return best_neighbor_key;
}
//}

/* getDistFactorOfNeighbors() //{ */
double AstarPlanner::getDistFactorOfNeighbors(const octomap::OcTreeKey& c) {
  std::vector<octomap::OcTreeKey> neighbors = getKeyNeighborhood6(c);
  std::vector<double>             dists;
  double                          max_dist = DBL_MIN;
  double                          dist;
  for (std::vector<octomap::OcTreeKey>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
    /* dists.push_back(pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(*it)));  // nodeDistance can be replaced with 1.0 for 6 neighborhood */
    dist = pcl_map_.getDistanceFromNearestPoint(octomapKeyToPclPoint(*it));
    if (dist > max_dist) {
      max_dist = dist;
    }
    dists.push_back(dist);
  }
  double res = 0.0;
  sort(dists.begin(), dists.end());
  for (uint k = 0; k < dists.size(); k++) {
    res += (k + 1) * (k + 1) * dists[k];
  }
  return res;
}
//}

/* getPossiblSuccessors() //{ */
std::vector<Node> AstarPlanner::getPossibleSuccessors(const octomap::OcTreeKey& parent, const octomap::OcTreeKey& current) {
  int                move_length = nofDifferentKeyCoordinates(parent, current);
  std::vector<Node>  successors;
  octomap::OcTreeKey tmp_key;
  if (move_length == 1) {  // horizontal
    /* ROS_INFO("[AstarPlanner]: Move length == 1"); */
    int c_coord  = current.k[0] != parent.k[0] ? 0 : current.k[1] != parent.k[1] ? 1 : 2;
    int dir_sign = current.k[c_coord] - parent.k[c_coord];
    int u_1, u_2;  // unchanged coords
    u_2 = c_coord == 0 ? 1 : 0;
    u_1 = c_coord == 2 ? 1 : 2;
    for (int b = -1; b < 2; b++) {
      for (int c = -1; c < 2; c++) {
        octomap::OcTreeKey tmp_key;
        Node               neighbor;
        /* tmp_key.k[c_coord] = current.k[c_coord]; */
        tmp_key.k[u_1]     = current.k[u_1] + b;
        tmp_key.k[u_2]     = current.k[u_2] + c;
        tmp_key.k[c_coord] = current.k[c_coord] + dir_sign;
        neighbor.key       = tmp_key;
        successors.push_back(neighbor);
      }
    }
  } else if (move_length == 2) {  // 2D - diagonal
    /* ROS_INFO("[AstarPlanner]: Move length == 2"); */
    int u_coord = current.k[0] == parent.k[0] ? 0 : current.k[1] == parent.k[1] ? 1 : 2;
    int c_1     = u_coord == 0 ? 1 : 0;
    int c_2     = u_coord == 2 ? 1 : 2;
    int dir_c_1 = current.k[c_1] - parent.k[c_1];
    int dir_c_2 = current.k[c_2] - parent.k[c_2];

    for (size_t k = 0; k < idxs_2d_diagonal_unconditioned_.size(); k++) {
      octomap::OcTreeKey tmp_key;
      Node               neighbor;
      tmp_key.k[u_coord] = current.k[u_coord] + neighborhood_cube_[idxs_2d_diagonal_unconditioned_[k] - 1][2];
      tmp_key.k[c_1]     = current.k[c_1] + dir_c_1 * neighborhood_cube_[idxs_2d_diagonal_unconditioned_[k] - 1][0];
      tmp_key.k[c_2]     = current.k[c_2] + dir_c_2 * neighborhood_cube_[idxs_2d_diagonal_unconditioned_[k] - 1][1];
      neighbor.key       = tmp_key;
      successors.push_back(neighbor);
    }
    for (size_t k = 0; k < idxs_2d_diagonal_conditioned_.size(); k++) {
      octomap::OcTreeKey tmp_key;
      octomap::OcTreeKey obs_key;
      bool               forced         = true;
      bool               partial_result = true;
      for (size_t m = 0; m < obs_2d_diagonal_cond_[idxs_2d_diagonal_conditioned_[k] - 1].size(); m++) {
        partial_result = true;
        for (size_t n = 0; n < obs_2d_diagonal_cond_[idxs_2d_diagonal_conditioned_[k] - 1][m].size(); n++) {
          obs_key.k[u_coord] = current.k[u_coord] + neighborhood_cube_[obs_2d_diagonal_cond_[idxs_2d_diagonal_conditioned_[k] - 1][m][n]][2];
          obs_key.k[c_1]     = current.k[c_1] + dir_c_1 * neighborhood_cube_[obs_2d_diagonal_cond_[idxs_2d_diagonal_conditioned_[k] - 1][m][n]][0];
          obs_key.k[c_2]     = current.k[c_2] + dir_c_2 * neighborhood_cube_[obs_2d_diagonal_cond_[idxs_2d_diagonal_conditioned_[k] - 1][m][n]][1];
          if (!checkValidityWithNeighborhood(obs_key)) {
            partial_result = false;
            break;
          }
        }
        if (partial_result) {
          forced = false;
          break;
        }
      }
      if (forced) {
        Node neighbor;
        tmp_key.k[u_coord] = current.k[u_coord] + neighborhood_cube_[idxs_2d_diagonal_conditioned_[k] - 1][2];
        tmp_key.k[c_1]     = current.k[c_1] + dir_c_1 * neighborhood_cube_[idxs_2d_diagonal_conditioned_[k] - 1][0];
        tmp_key.k[c_2]     = current.k[c_2] + dir_c_2 * neighborhood_cube_[idxs_2d_diagonal_conditioned_[k] - 1][1];
        neighbor.key       = tmp_key;
        successors.push_back(neighbor);
      }
    }
  } else {
    /* ROS_INFO("[AstarPlanner]: Move length == 3"); */
    // cube
    // obscond_for_diagonal_move_
    // TODO: fill implementation for 3D move
    int dir_c_1 = current.k[0] - parent.k[0];
    int dir_c_2 = current.k[1] - parent.k[1];
    int dir_c_3 = current.k[2] - parent.k[2];

    for (size_t k = 0; k < idxs_diagonal_unconditioned_.size(); k++) {
      octomap::OcTreeKey tmp_key;
      Node               neighbor;
      tmp_key.k[0] = current.k[0] + dir_c_1 * neighborhood_cube_[idxs_diagonal_unconditioned_[k] - 1][0];
      tmp_key.k[1] = current.k[1] + dir_c_2 * neighborhood_cube_[idxs_diagonal_unconditioned_[k] - 1][1];
      tmp_key.k[2] = current.k[2] + dir_c_3 * neighborhood_cube_[idxs_diagonal_unconditioned_[k] - 1][2];
      neighbor.key = tmp_key;
      successors.push_back(neighbor);
    }
    for (size_t k = 0; k < idxs_diagonal_conditioned_.size(); k++) {
      octomap::OcTreeKey tmp_key;
      octomap::OcTreeKey obs_key;
      bool               forced         = true;
      bool               partial_result = true;
      for (size_t m = 0; m < obs_diagonal_cond_[idxs_diagonal_conditioned_[k] - 1].size(); m++) {
        partial_result = true;
        for (size_t n = 0; n < obs_diagonal_cond_[idxs_diagonal_conditioned_[k] - 1][m].size(); n++) {
          obs_key.k[0] = current.k[0] + dir_c_1 * neighborhood_cube_[obs_diagonal_cond_[idxs_diagonal_conditioned_[k] - 1][m][n]][0];
          obs_key.k[1] = current.k[1] + dir_c_2 * neighborhood_cube_[obs_diagonal_cond_[idxs_diagonal_conditioned_[k] - 1][m][n]][1];
          obs_key.k[2] = current.k[2] + dir_c_3 * neighborhood_cube_[obs_diagonal_cond_[idxs_diagonal_conditioned_[k] - 1][m][n]][2];
          if (!checkValidityWithNeighborhood(obs_key)) {
            partial_result = false;
            break;
          }
        }
        if (partial_result) {
          forced = false;
          break;
        }
      }
      if (forced) {
        Node neighbor;
        tmp_key.k[0] = current.k[0] + dir_c_1 * neighborhood_cube_[idxs_diagonal_conditioned_[k] - 1][0];
        tmp_key.k[1] = current.k[1] + dir_c_2 * neighborhood_cube_[idxs_diagonal_conditioned_[k] - 1][1];
        tmp_key.k[2] = current.k[2] + dir_c_3 * neighborhood_cube_[idxs_diagonal_conditioned_[k] - 1][2];
        neighbor.key = tmp_key;
        successors.push_back(neighbor);
      }
    }
  }
  /* ROS_INFO("[AstarPlanner]: Parent = [%d, %d, %d], current = [%d, %d, %d]", parent.k[0], parent.k[1], parent.k[2], current.k[0],
   * current.k[1], current.k[2]); */
  /* ROS_WARN("[AstarPlanner]: Size of successors = %lu", successors.size()); */
  /* for (size_t k = 0; k < successors.size(); k++) { */
  /*   ROS_INFO("[AstarPlanner]: Succesor %lu: [%d, %d, %d]", k, successors[k].key.k[0], successors[k].key.k[1],
   * successors[k].key.k[2]); */
  /* } */

  return successors;
}
//}

/* setStartAndGoal() //{ */
void AstarPlanner::setStartAndGoal(octomap::point3d start_pose, octomap::point3d goal_pose) {
  start_.pose   = start_pose;
  start_.key    = planning_octree_->coordToKey(start_pose);
  start_.f_cost = 0.0;
  goal_.pose    = goal_pose;
  goal_.key     = planning_octree_->coordToKey(goal_pose);
  goal_.h_cost  = 0.0;
}
//}

/* setVerbose() //{ */
void AstarPlanner::setVerbose(const bool verbose) {
  verbose_ = verbose;
}
//}

/* setSafeDist() //{ */
void AstarPlanner::setSafeDist(const double safe_dist) {
  safe_dist_ = safe_dist;
  ROS_INFO("[AstarPlanner]: A* safe dist set to %.2f ", safe_dist_);
}
//}

/* setAstarAdmissibility() //{ */
void AstarPlanner::setAstarAdmissibility(const double astar_admissibility) {
  astar_admissibility_ = astar_admissibility;
  ROS_INFO("[AstarPlanner]: A* admissibility set to %.2f ", astar_admissibility_);
}
//}

/* SUPPORTING METHODS //{ */

/* isNodeGoal() //{ */
bool AstarPlanner::isNodeGoal(const Node& n) {
  if (n.key[0] == goal_.key.k[0] && n.key[1] == goal_.key.k[1] && n.key[2] == goal_.key.k[2]) {
    return true;
  }
  return false;
}
//}

/* isNodeGoal() //{ */
bool AstarPlanner::isNodeGoal(const Node& n, const Node& goal) {
  if (n.key[0] == goal.key.k[0] && n.key[1] == goal.key.k[1] && n.key[2] == goal.key.k[2]) {
    return true;
  }
  return false;
}
//}

/* euclideanCost() //{ */
double AstarPlanner::euclideanCost(const Node& n) {
  return sqrt(pow(n.key.k[0] - goal_.key.k[0], 2) + pow(n.key.k[1] - goal_.key.k[1], 2) + pow(n.key.k[2] - goal_.key.k[2], 2));
}
//}

/* manhattanCost() //{ */
double AstarPlanner::manhattanCost(const Node& n) {
  return fabs(n.key.k[0] - goal_.key.k[0]) + fabs(n.key.k[1] - goal_.key.k[1]) + fabs(n.key.k[2] - goal_.key.k[2]);
}
//}

/* noedDistance() //{ */
double AstarPlanner::nodeDistance(const Node& a, const Node& b) {
  return sqrt(pow(a.key.k[0] - b.key.k[0], 2) + pow(a.key.k[1] - b.key.k[1], 2) + pow(a.key.k[2] - b.key.k[2], 2));
}
//}

/* isNodeInTheNeighborhood() //{ */
bool AstarPlanner::isNodeInTheNeighborhood(const octomap::OcTreeKey& n, const octomap::OcTreeKey& center, double dist) {
  double voxel_dist = sqrt(pow(n.k[0] - center.k[0], 2) + pow(n.k[1] - center.k[1], 2) + pow(n.k[2] - center.k[2], 2));
  /* ROS_INFO("[AstarPlanner]: isNodeInTheNeighborhood: returning: %.2f ", (voxel_dist * resolution_) ); */
  return (voxel_dist * resolution_) < dist;
}
//}

/* getCubeForMoves() //{ */
std::vector<std::vector<int>> AstarPlanner::getCubeForMoves() {
  std::vector<std::vector<int>> cube;
  for (int i3 = -1; i3 < 2; i3++) {
    for (int i1 = 1; i1 > -2; i1--) {
      for (int i2 = -1; i2 < 2; i2++) {
        cube.push_back({i1, i2, i3});
      }
    }
  }
  return cube;
}
//}

/* initializeIdxsOfcellsForPruning() //{ */
void AstarPlanner::initializeIdxsOfcellsForPruning() {
  /* idxs_diagonal_conditioned_      = {1, 2, 3, 6, 9, 10, 18, 19, 22, 25, 26, 27}; */
  idxs_diagonal_conditioned_ = {27, 26, 25, 22, 19, 18, 10, 9, 6, 3, 2, 1};
  /* idxs_diagonal_unconditioned_    = {11, 12, 15, 20, 21, 23, 24}; */
  idxs_diagonal_unconditioned_ = {24, 23, 21, 20, 15, 12, 11};
  /* idxs_2d_diagonal_conditioned_   = {1, 2, 6, 9, 10, 18, 19, 20, 24, 27}; */
  idxs_2d_diagonal_conditioned_ = {27, 24, 20, 19, 18, 10, 9, 6, 2, 1};
  /* idxs_2d_diagonal_unconditioned_ = {3, 11, 12, 15, 21}; */
  idxs_2d_diagonal_unconditioned_ = {21, 15, 12, 11, 3};
  obs_diagonal_cond_              = getObstacleConditionsForDiagonalMove();
  obs_2d_diagonal_cond_           = getObstacleConditionsFor2dDiagonalMove();
  neighborhood_cube_              = getCubeForMoves();
  idxs1                           = {{-1, -1}};
  idxs2                           = {{-1, 0}, {0, -1}};
  idxs3                           = {{-1, 0}, {0, -1}, {0, 0}, {-1, -1}};
  idxs4                           = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}, {0, 0}};
  idxs1d                          = {-1};
  idxs2d                          = {-1, 0};
  idxs3d                          = {-1, 0, 1};
}
//}

/* getObstacleConditionsForDiagonalMove() //{ */
std::vector<std::vector<std::vector<int>>> AstarPlanner::getObstacleConditionsForDiagonalMove() {
  std::vector<std::vector<std::vector<int>>> obs_cond;
  std::vector<std::vector<int>>              p_cond;
  p_cond.clear();
  p_cond.push_back({4});
  p_cond.push_back({13});
  obs_cond.push_back(p_cond);  // 1
  p_cond.clear();
  p_cond.push_back({4});
  p_cond.push_back({5});
  obs_cond.push_back(p_cond);  // 2
  p_cond.clear();
  p_cond.push_back({5});
  obs_cond.push_back(p_cond);  // 3
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 4
  obs_cond.push_back(p_cond);  // 5
  p_cond.push_back({5});
  p_cond.push_back({8});
  obs_cond.push_back(p_cond);  // 6
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 7
  obs_cond.push_back(p_cond);  // 8
  p_cond.push_back({8});
  obs_cond.push_back(p_cond);  // 9
  p_cond.clear();
  p_cond.push_back({4});
  p_cond.push_back({13});
  obs_cond.push_back(p_cond);  // 10
  p_cond.clear();
  for (int k = 11; k < 18; k++) {
    obs_cond.push_back(p_cond);  // 11 - 17
  }
  p_cond.push_back({8});
  p_cond.push_back({17});
  obs_cond.push_back(p_cond);  // 18
  p_cond.clear();
  p_cond.push_back({13});
  p_cond.push_back({4, 10});
  p_cond.push_back({16, 22});
  obs_cond.push_back(p_cond);  // 19
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 20
  obs_cond.push_back(p_cond);  // 21
  p_cond.push_back({16});
  p_cond.push_back({13});
  obs_cond.push_back(p_cond);  // 22
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 23
  obs_cond.push_back(p_cond);  // 24
  p_cond.clear();
  p_cond.push_back({16});
  obs_cond.push_back(p_cond);  // 25
  p_cond.clear();
  p_cond.push_back({16});
  p_cond.push_back({17});
  obs_cond.push_back(p_cond);  // 26
  p_cond.clear();
  p_cond.push_back({17});
  p_cond.push_back({8, 18});
  p_cond.push_back({16, 26});
  obs_cond.push_back(p_cond);  // 27

  return obs_cond;
}
//}

/* getObstacleConditionsFor2dDiagonalMove() //{ */
std::vector<std::vector<std::vector<int>>> AstarPlanner::getObstacleConditionsFor2dDiagonalMove() {
  std::vector<std::vector<std::vector<int>>> obs_cond;
  std::vector<std::vector<int>>              p_cond;
  p_cond.clear();
  p_cond.push_back({4});
  p_cond.push_back({13});
  obs_cond.push_back(p_cond);  // 1
  p_cond.clear();
  p_cond.push_back({5});
  obs_cond.push_back(p_cond);  // 2
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 3
  obs_cond.push_back(p_cond);  // 4
  obs_cond.push_back(p_cond);  // 5
  p_cond.push_back({5});
  obs_cond.push_back(p_cond);  // 6
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 7
  obs_cond.push_back(p_cond);  // 8
  p_cond.push_back({8});
  p_cond.push_back({17});
  obs_cond.push_back(p_cond);  // 9
  p_cond.clear();
  p_cond.push_back({13});
  obs_cond.push_back(p_cond);  // 10
  p_cond.clear();
  for (int k = 11; k < 18; k++) {
    obs_cond.push_back(p_cond);  // 11 - 17
  }
  p_cond.push_back({17});
  obs_cond.push_back(p_cond);  // 18
  p_cond.clear();
  p_cond.push_back({13});
  p_cond.push_back({22});
  obs_cond.push_back(p_cond);  // 19
  p_cond.clear();
  p_cond.push_back({23});
  obs_cond.push_back(p_cond);  // 20
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 21
  obs_cond.push_back(p_cond);  // 22
  obs_cond.push_back(p_cond);  // 23
  p_cond.push_back({23});
  obs_cond.push_back(p_cond);  // 24
  p_cond.clear();
  obs_cond.push_back(p_cond);  // 25
  obs_cond.push_back(p_cond);  // 26
  p_cond.push_back({17});
  p_cond.push_back({26});
  obs_cond.push_back(p_cond);  // 27
  return obs_cond;
}
//}

/* getNeighborhood26() //{ */
std::vector<Node> AstarPlanner::getNeighborhood26(const Node& n) {
  std::vector<Node> neighbors;
  for (int a = -1; a < 2; a++) {
    for (int b = -1; b < 2; b++) {
      for (int c = -1; c < 2; c++) {
        if (a == 0 && b == 0 && c == 0) {
          continue;
        }
        octomap::OcTreeKey tmp_key;
        Node               neighbor;
        tmp_key.k[0] = n.key.k[0] + a;
        tmp_key.k[1] = n.key.k[1] + b;
        tmp_key.k[2] = n.key.k[2] + c;
        neighbor.key = tmp_key;
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
}
//}

/* getKeyNeighborhood26() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getKeyNeighborhood26(const octomap::OcTreeKey& key_a) {
  std::vector<octomap::OcTreeKey> neighbors;
  for (int a = -1; a < 2; a++) {
    for (int b = -1; b < 2; b++) {
      for (int c = -1; c < 2; c++) {
        if (a == 0 && b == 0 && c == 0) {
          continue;
        }
        octomap::OcTreeKey tmp_key;
        Node               neighbor;
        tmp_key.k[0] = key_a.k[0] + a;
        tmp_key.k[1] = key_a.k[1] + b;
        tmp_key.k[2] = key_a.k[2] + c;
        neighbors.push_back(tmp_key);
      }
    }
  }
  return neighbors;
}
//}

/* getKeyNeighborhood8() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getKeyNeighborhood8(const octomap::OcTreeKey& key_a) {
  std::vector<octomap::OcTreeKey> neighbors;
  for (int a = -1; a < 2; a++) {
    for (int b = -1; b < 2; b++) {
      if (a == 0 && b == 0) {
        continue;
      }
      octomap::OcTreeKey tmp_key;
      Node               neighbor;
      tmp_key.k[0] = key_a.k[0] + a;
      tmp_key.k[1] = key_a.k[1] + b;
      tmp_key.k[2] = key_a.k[2];
      neighbors.push_back(tmp_key);
    }
  }
  return neighbors;
}
//}

/* getNeighborhood6() //{ */
std::vector<Node> AstarPlanner::getNeighborhood6(const Node& n) {
  std::vector<Node>             neighbors;
  std::vector<std::vector<int>> shifts;
  shifts.push_back({1, 0, 0});
  shifts.push_back({-1, 0, 0});
  shifts.push_back({0, 1, 0});
  shifts.push_back({0, -1, 0});
  shifts.push_back({0, 0, 1});
  shifts.push_back({0, 0, -1});
  for (uint i = 0; i < shifts.size(); i++) {
    octomap::OcTreeKey tmp_key;
    Node               neighbor;
    tmp_key.k[0] = n.key.k[0] + shifts[i][0];
    tmp_key.k[1] = n.key.k[1] + shifts[i][1];
    tmp_key.k[2] = n.key.k[2] + shifts[i][2];
    neighbor.key = tmp_key;
    neighbors.push_back(neighbor);
  }
  return neighbors;
}
//}

/* getKeyNeighborhood6() //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getKeyNeighborhood6(const octomap::OcTreeKey& key) {
  std::vector<octomap::OcTreeKey> neighbors;
  std::vector<std::vector<int>>   shifts;
  shifts.push_back({1, 0, 0});
  shifts.push_back({-1, 0, 0});
  shifts.push_back({0, 1, 0});
  shifts.push_back({0, -1, 0});
  shifts.push_back({0, 0, 1});
  shifts.push_back({0, 0, -1});
  for (uint i = 0; i < shifts.size(); i++) {
    octomap::OcTreeKey tmp_key;
    Node               neighbor;
    tmp_key.k[0] = key.k[0] + shifts[i][0];
    tmp_key.k[1] = key.k[1] + shifts[i][1];
    tmp_key.k[2] = key.k[2] + shifts[i][2];
    neighbors.push_back(tmp_key);
  }
  return neighbors;
}
//}

/* checkLimits() //{ */
bool AstarPlanner::checkLimits(const std::vector<int>& map_limits, const Node& n) {
  return !(n.key[0] < map_limits[0] || n.key[0] > map_limits[1] || n.key[1] < map_limits[2] || n.key[1] > map_limits[3] || n.key[2] < map_limits[4] ||
           n.key[2] > map_limits[5]);
}
//}

/* areKeysDiagonalNeighbors() //{ */
bool AstarPlanner::areKeysDiagonalNeighbors(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  return (keyManhattanDist(k1, k2) == 2 && nofDifferentKeyCoordinates(k1, k2) == 2) ||
         (keyManhattanDist(k1, k2) == 3 && nofDifferentKeyCoordinates(k1, k2) == 3);
}
//}

/* getDifferenceInCoordinates() //{ */
std::vector<int> AstarPlanner::getDifferenceInCoordinates(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  std::vector<int> diffs;
  for (uint i = 0; i < 3; i++) {
    diffs.push_back(abs(k1.k[i] - k2.k[i]));
  }
  return diffs;
}
//}

/* areKeysInNeighborhood() //{ */
bool AstarPlanner::areKeysInNeighborhood(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  return abs(k1.k[0] - k2.k[0]) <= 1 && abs(k1.k[1] - k2.k[1]) <= 1 && abs(k1.k[2] - k2.k[2]) <= 1;  // 26N version
  /* return (abs(k1.k[0] - k2.k[0]) + abs(k1.k[1] - k2.k[1]) + abs(k1.k[2] - k2.k[2])) == 1; // 6N version */
}
//}

/* areKeysInNeighborhood() //{ */
bool AstarPlanner::areKeysInTwoStepsDistance(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  return abs(k1.k[0] - k2.k[0]) <= 2 && abs(k1.k[1] - k2.k[1]) <= 2 && abs(k1.k[2] - k2.k[2]) <= 2;  // 26N version
  /* return (abs(k1.k[0] - k2.k[0]) + abs(k1.k[1] - k2.k[1]) + abs(k1.k[2] - k2.k[2])) == 1; // 6N version */
}
//}

/* areKeysEqual() //{ */
bool AstarPlanner::areKeysEqual(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  return (k1.k[0] == k2.k[0]) && (k1.k[1] == k2.k[1]) && (k1.k[2] == k2.k[2]);
}
//}

/* keyManhattanDist() //{ */
int AstarPlanner::keyManhattanDist(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  return abs(k1.k[0] - k2.k[0]) + abs(k1.k[1] - k2.k[1]) + abs(k1.k[2] - k2.k[2]);
}
//}

/* keyManhattanDist() //{ */
double AstarPlanner::keyEuclideanDist(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  return sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));
}
//}

/* nofDifferentKeyCoordinates() //{ */
int AstarPlanner::nofDifferentKeyCoordinates(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  int result = k1.k[0] == k2.k[0] ? 0 : 1;
  if (k1.k[1] != k2.k[1]) {
    result++;
  }
  if (k1.k[2] != k2.k[2]) {
    result++;
  }
  return result;
}
//}

/*  octomapKeyToPclPoint()//{ */
pcl::PointXYZ AstarPlanner::octomapKeyToPclPoint(const octomap::OcTreeKey& octo_key) {
  octomap::point3d point3d = planning_octree_->keyToCoord(octo_key);
  pcl::PointXYZ    pcl_point;
  pcl_point.x = point3d.x();
  pcl_point.y = point3d.y();
  pcl_point.z = point3d.z();
  return pcl_point;
}
//}

/* isConnectivityMaintained() //{ */
int AstarPlanner::isConnectivityMaintained(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2) {
  // returns 1 for exactly the same keys, 2 for fully connected keys and three for disconnected
  double euclidean_dist = sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));
  if (euclidean_dist == 0) {
    return 1;
  } else if (euclidean_dist <= 1.5) {  // <= sqrt(2)
    return 2;
  } else {
    return 3;
  }
}
//}

/* setPlanningOctree() //{ */
void AstarPlanner::setPlanningOctree(std::shared_ptr<octomap::OcTree> new_map) {
  planning_octree_ = new_map;
}
//}

/* visualizeOccupiedPoints() //{ */
void AstarPlanner::visualizeOccupiedPoints(std::vector<pcl::PointXYZ>& pcl_points) {

  for (auto& point : pcl_points) {
    Eigen::Vector3d p(point.x, point.y, point.z);
    batch_visualizer_->addPoint(p, 0.2, 0.2, 1.0, 0.3);
  }
}
//}

/* visualizeExpansions() //{ */

void AstarPlanner::visualizeExpansions(const std::unordered_set<Node, NodeHasher>& open, const std::unordered_set<Node, NodeHasher>& closed,
                                       octomap::OcTree& tree) {

  for (auto& n : open) {
    auto            coord = tree.keyToCoord(n.key);
    Eigen::Vector3d p(coord.x(), coord.y(), coord.z());
    batch_visualizer_->addPoint(p, 0.2, 1.0, 0.2, 0.3);
  }

  for (auto& n : closed) {
    auto            coord = tree.keyToCoord(n.key);
    Eigen::Vector3d p(coord.x(), coord.y(), coord.z());
    batch_visualizer_->addPoint(p, 1.0, 0.2, 0.2, 0.3);
  }
}

//}

/* visualizeGoal() //{ */

void AstarPlanner::visualizeGoal(const octomap::point3d& goal) {

  Eigen::Vector3d           center(goal.x(), goal.y(), goal.z());
  double                    cube_scale  = 0.5;
  Eigen::Vector3d           size        = Eigen::Vector3d(1, 1, 1) * cube_scale;
  Eigen::Quaterniond        orientation = Eigen::Quaterniond::Identity();
  mrs_lib::geometry::Cuboid c(center, size, orientation);

  batch_visualizer_->addCuboid(c, 1.0, 0.0, 1.0, 1.0, true);
}

//}
