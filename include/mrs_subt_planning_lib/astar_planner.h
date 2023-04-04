#ifndef __ASTAR_PLANNER_H__
#define __ASTAR_PLANNER_H__

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/batch_visualizer.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include "mrs_subt_planning_lib/pcl_map.h"


namespace mrs_subt_planning
{

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

struct GridParams
{
  double width;   // numel x
  double height;  // numel y
  double depth;   // numel z
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
};

struct Node
{
  octomap::OcTreeKey key;
  octomap::point3d   pose = octomap::point3d(0.0, 0.0, 0.0);
  octomap::OcTreeKey parent_key;
  double             g_cost      = 0;  // overall node cost
  double             h_cost      = 0;  // heuristic cost
  double             f_cost      = 0;  // cost from start
  double             unnorm_cost = 0;  // cost unnormalized
  double             obs_cost    = 0;  // cost unnormalized
  double             path_cost   = 0;  // cost unnormalized
  int                n_nodes     = 0;  // number of nodes visited from start

  bool operator==(Node b) {
    return key == b.key;
  }


  bool operator!=(const Node b) {
    return key != b.key;
  }

  bool operator<(Node b) {
    return ((key.k[0] + key.k[1] + key.k[2]) < (b.key.k[0] + b.key.k[1] + b.key.k[2])) &&
           ((key.k[0] != b.key.k[0]) || (key.k[1] != b.key.k[1]) || (key.k[2] != b.key.k[2]));
  }

  bool operator>(Node b) {
    return ((key.k[0] + key.k[1] + key.k[2]) > (b.key.k[0] + b.key.k[1] + b.key.k[2])) &&
           ((key.k[0] != b.key.k[0]) || (key.k[1] != b.key.k[1]) || (key.k[2] != b.key.k[2]));
  }
};

struct NodeCompare
{
  bool operator()(const Node& lhs, const Node& rhs) {
    return lhs.g_cost > rhs.g_cost;
  }
};

inline bool operator<(const Node& lhs, const Node& rhs) {
  return ((lhs.key.k[0] + lhs.key.k[1] + lhs.key.k[2]) < (rhs.key.k[0] + rhs.key.k[1] + rhs.key.k[2])) &&
         ((lhs.key.k[0] != rhs.key.k[0]) || (lhs.key.k[1] != rhs.key.k[1]) || (lhs.key.k[2] != rhs.key.k[2]));
}

inline bool operator==(const Node& lhs, const Node& rhs) {
  return lhs.key == rhs.key;
}

struct NodeHasher
{
  std::size_t operator()(const Node& n) const {
    using std::hash;
    return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
  }
};

class AstarPriorityQueue : public std::priority_queue<Node, std::vector<Node>, NodeCompare> {
public:
  int conditional_remove(const Node& value, double cost_value) {
    auto it = std::find_if(this->c.begin(), this->c.end(), node_equal(value));
    if (it != this->c.end()) {
      if (it->f_cost > cost_value) {
        this->c.erase(it);
        std::make_heap(this->c.begin(), this->c.end(), this->comp);
        return 1;  // node removed
      } else {
        return -1;  // node present but not removed
      }
    } else {
      return 0;  // node not present
    }
  }

  std::vector<octomap::point3d> getAllNodes() {
    std::vector<octomap::point3d> nodes;
    for (auto p : this->c) {
      nodes.push_back(p.pose);
    }
    return nodes;
  }

  struct node_equal : std::unary_function<Node, bool>
  {
    node_equal(const Node& n_a) : n_a_(n_a) {
    }
    bool operator()(const Node& n_b) const {
      return (n_b.key.k[0] == n_a_.key.k[0]) && (n_b.key.k[1] == n_a_.key.k[1]) && (n_b.key.k[2] == n_a_.key.k[2]);
    }
    const Node& n_a_;
  };
};

/**
 * @brief Class planner represents the main class for path planning
 */

class AstarPlanner {
public:
  /**
   * @brief constructor
   */
  AstarPlanner();

  /**
   * @brief destructor
   */
  virtual ~AstarPlanner();

  void initialize(octomap::point3d start_point, octomap::point3d goal_point, std::shared_ptr<octomap::OcTree> planning_octree,
                  bool enable_planning_to_unreachable_goal, double planning_timeout_, double safe_dist, double clearing_dist, double min_altitude,
                  double max_altitude, bool debug, std::shared_ptr<mrs_lib::BatchVisualizer> batch_visualizer,
                  const bool break_at_timeout = false);  // for backward compatibility only

  void initialize(bool enable_planning_to_unreachable_goal, double planning_timeout_, double postprocessing_timeout_, double safe_dist, double clearing_dist,
                  double min_altitude, double max_altitude, bool debug, std::shared_ptr<mrs_lib::BatchVisualizer> batch_visualizer,
                  const bool break_at_timeout = false);

  std::vector<Node> getNodePath();  // for backward compatibility only
  std::vector<Node> getNodePath(const octomap::point3d& start_point, const octomap::point3d& goal_point, std::shared_ptr<octomap::OcTree> planning_octree,
                                bool ignore_unknown_cells_near_start = false, double box_size_for_unknown_cells_replacement = 2.0);
  std::vector<Node> getNodePath(const std::vector<octomap::point3d>& initial_waypoints, std::shared_ptr<octomap::OcTree> planning_octree,
                                bool ignore_unknown_cells_near_start = false, double box_size_for_unknown_cells_replacement = 2.0);

  std::vector<octomap::point3d>   getWaypointPath(const std::vector<Node>& node_path);
  std::vector<octomap::point3d>   getWaypointPath(const std::vector<octomap::OcTreeKey>& key_path);
  std::vector<octomap::point3d>   getLocalPath(const std::vector<Node>& node_path);
  std::vector<octomap::OcTreeKey> getSafePath(const std::vector<octomap::OcTreeKey>& key_path, double safe_dist, int max_iteration, double z_diff_tolerance,
                                              bool fix_goal_point, bool horizontal_neighbors_only, double timeout);
  std::vector<octomap::point3d>   getStraightenWaypointPath(std::vector<Node>& node_path, double dist_step);
  std::vector<octomap::OcTreeKey> getFilteredPlan(const std::vector<octomap::OcTreeKey>& original_path, int size_of_window, double enabled_filtering_dist);

  std::pair<int, int> firstUnfeasibleNodeInPath(const std::vector<octomap::OcTreeKey>& key_waypoints, const std::vector<geometry_msgs::Point>& pose_array,
                                                int n_points_forward, const octomap::point3d& current_pose, double safe_dist_for_replanning_,
                                                double critical_dist_for_replanning);
  octomap::point3d    getLastFoundGoal();
  std::vector<octomap::OcTreeKey> getKeyPath(const std::vector<Node>& plan);
  void                            setStartAndGoal(octomap::point3d start_pose, octomap::point3d goal_pose);
  void                            setVerbose(const bool verbose);
  void                            setSafeDist(const double safe_dist);
  void                            setAstarAdmissibility(const double astar_admissibility);
  void                            setPlanningOctree(std::shared_ptr<octomap::OcTree> new_map);

  std::pair<std::vector<octomap::point3d>, bool> findPath(const octomap::point3d& start_point, const octomap::point3d& goal_point,
                                                          std::shared_ptr<octomap::OcTree> planning_octree, bool make_path_straight, bool apply_postprocessing,
                                                          double planning_bbx_size_h, double planning_bbx_size_v, double postprocessing_safe_dist,
                                                          int postprocessing_max_iterations, bool postprocessing_horizontal_neighbors_only,
                                                          double postprocessing_z_tolerance, double postprocessing_path_length, int shortening_window_size,
                                                          double shortening_dist, bool apply_pruning, double pruning_dist,
                                                          bool ignore_unknown_cells_near_start = false, double box_size_for_unknown_cells_replacement = 2.0);

  octomap::OcTreeNode* touchNode(std::shared_ptr<octomap::OcTree>& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<octomap::OcTree>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth = 0);

  std::shared_ptr<octomap::OcTree> createPlanningTree(std::shared_ptr<octomap::OcTree> tree, const octomap::point3d& start, double resolution,
                                                      std::vector<double> bbx);

protected:
  PCLMap pcl_map_;

  double max_planning_time;

  double astar_admissibility_ = 1.5;

  std::shared_ptr<octomap::OcTree> planning_octree_;
  GridParams                       grid_params_;
  octomap::point3d                 goal_coords_;
  octomap::OcTreeKey               goal_key_;
  octomap::point3d                 start_coords_;
  octomap::OcTreeKey               start_key_;

  Node goal_;
  Node start_;
  Node start_node_next;
  Node last_found_goal_;
  int  stop_index;

  std::shared_ptr<mrs_lib::BatchVisualizer> batch_visualizer_;

  bool                            isNodeValid(const Node& n);
  bool                            isNodeGoal(const Node& n);
  bool                            isNodeGoal(const Node& n, const Node& goal);
  double                          euclideanCost(const Node& n);
  double                          manhattanCost(const Node& n);
  std::vector<Node>               getNeighborhood6(const Node& n);
  std::vector<Node>               getNeighborhood26(const Node& n);
  std::vector<octomap::OcTreeKey> getKeyNeighborhood6(const octomap::OcTreeKey& k);
  std::vector<octomap::OcTreeKey> getKeyNeighborhood8(const octomap::OcTreeKey& k);
  std::vector<octomap::OcTreeKey> getKeyNeighborhood26(const octomap::OcTreeKey& k);
  double                          nodeDistance(const Node& a, const Node& b);
  bool                            checkValidityWithNeighborhood(const Node& a);
  bool                            checkValidityWithNeighborhood(const octomap::OcTreeKey& k);
  std::vector<pcl::PointXYZ>      octomapToPointcloud(const std::vector<int>& map_limits);
  std::vector<pcl::PointXYZ>      octomapToPointcloud();
  std::vector<int>                getMapLimits(const std::vector<octomap::OcTreeKey>& plan, int start_index, int end_index, int xy_reserve, int z_reserve);
  bool                            checkLimits(const std::vector<int>& map_limits, const Node& n);
  bool                            isNodeInTheNeighborhood(const octomap::OcTreeKey& n, const octomap::OcTreeKey& center, double dist);
  octomap::OcTreeKey              getBestNeighbor(const octomap::OcTreeKey& k, bool horizontal_neighbors_only);
  int                             isConnectivityMaintained(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  octomap::OcTreeKey              findConnection(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2, double original_obs_dist);
  pcl::PointXYZ                   octomapKeyToPclPoint(const octomap::OcTreeKey& octo_key);
  std::vector<octomap::OcTreeKey> generatePossibleConnectionsDiagonalKeys(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  bool                            areKeysInNeighborhood(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  bool                            areKeysEqual(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  octomap::OcTreeKey              getConnectionNode(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  int                             keyManhattanDist(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  double                          keyEuclideanDist(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  int                             nofDifferentKeyCoordinates(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<octomap::OcTreeKey> getAdditionalWaypoints(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<int>                getDifferenceInCoordinates(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<std::vector<octomap::OcTreeKey>> getPossibleWaypointsForTwoDiffCoord(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<std::vector<octomap::OcTreeKey>> getPossibleWaypointsForThreeDiffCoord(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<octomap::OcTreeKey>              getSafestWaypointsBetweenKeys(const std::vector<std::vector<octomap::OcTreeKey>>& possible_waypoints);
  std::vector<octomap::OcTreeKey>              getStraightenKeyPath(const std::vector<octomap::OcTreeKey>& key_path);
  bool                                         areKeysDiagonalNeighbors(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<octomap::OcTreeKey>              getSmoothPath(const std::vector<octomap::OcTreeKey>& path);
  Node                                         getValidNodeInNeighborhood(const Node& goal);
  bool                                         checkValidityWithKDTree(const Node& n);
  bool                                         checkValidityWithKDTree(const octomap::OcTreeKey& k);
  std::vector<octomap::OcTreeKey>              getKeyVectorFromCoordinates(const std::vector<geometry_msgs::Point>& pose_array);
  double                                       getDistFactorOfNeighbors(const octomap::OcTreeKey& c);
  void                                         replaceUnknownByFreeCells(const octomap::OcTreeKey& start_key, double box_size);
  std::vector<Node>                            getPathToNearestFeasibleNode(const Node& start);
  double                                       pointLineDist(octomap::point3d lb, octomap::point3d le, octomap::point3d point);
  std::vector<octomap::point3d>                getWaypointPathWithoutObsoletePoints(std::vector<octomap::point3d>& waypoint_path, double tolerance);
  std::vector<octomap::point3d>                pruneWaypoints(std::vector<octomap::point3d>& waypoint_path, double pruning_dist);

  void visualizeOccupiedPoints(std::vector<pcl::PointXYZ>& pcl_points);
  void visualizeExpansions(const std::unordered_set<Node, NodeHasher>& open, const std::unordered_set<Node, NodeHasher>& closed, octomap::OcTree& tree);
  void visualizeGoal(const octomap::point3d& goal);

  // params
  bool   use_neighborhood_6_;
  double planning_timeout_;
  double postprocessing_timeout_;
  double resolution_;
  bool   enable_planning_to_unreachable_goal_;
  bool   debug_;
  bool   verbose_;
  bool   initialized_;
  double safe_dist_;
  double safe_dist_prev_;
  double clearing_dist_;
  double min_altitude_;
  double max_altitude_;
  bool   break_at_timeout_;

  // pruning
  std::vector<int>                           idxs_diagonal_conditioned_;
  std::vector<int>                           idxs_diagonal_unconditioned_;
  std::vector<int>                           idxs_2d_diagonal_conditioned_;
  std::vector<int>                           idxs_2d_diagonal_unconditioned_;
  std::vector<std::vector<std::vector<int>>> obs_diagonal_cond_;
  std::vector<std::vector<std::vector<int>>> obs_2d_diagonal_cond_;
  std::vector<std::vector<int>>              neighborhood_cube_;
  std::vector<int>                           idxs1d;
  std::vector<int>                           idxs2d;
  std::vector<int>                           idxs3d;
  std::vector<std::vector<int>>              idxs1;
  std::vector<std::vector<int>>              idxs2;
  std::vector<std::vector<int>>              idxs3;
  std::vector<std::vector<int>>              idxs4;

  void                                       initializeIdxsOfcellsForPruning();
  std::vector<std::vector<std::vector<int>>> getObstacleConditionsForDiagonalMove();
  std::vector<std::vector<std::vector<int>>> getObstacleConditionsFor2dDiagonalMove();
  std::vector<std::vector<int>>              getCubeForMoves();
  std::vector<Node>                          getPossibleSuccessors(const octomap::OcTreeKey& parent, const octomap::OcTreeKey& current);
  octomap::OcTreeKey                         getSafestWaypointsBetweenKeys(const std::vector<octomap::OcTreeKey>& possible_waypoints);
  octomap::OcTreeKey                         getConnectionNode3d(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<octomap::OcTreeKey>            getAdditionalWaypoints3d(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  octomap::OcTreeKey                         getBestNeighborEscape(const octomap::OcTreeKey& c, const octomap::OcTreeKey& prev);
  std::vector<octomap::OcTreeKey>            getFilteredNeighborhoodPlan(const std::vector<octomap::OcTreeKey>& plan);
  bool                                       areKeysInTwoStepsDistance(const octomap::OcTreeKey& k1, const octomap::OcTreeKey& k2);
  std::vector<octomap::OcTreeKey>            getFilteredNeighborhoodPlan2(const std::vector<octomap::OcTreeKey>& original_path);
  std::vector<Node>                          getFilteredNeighborhoodPlan(const std::vector<Node>& original_path);
  std::vector<octomap::OcTreeKey>            getZzFilteredPlan(const std::vector<octomap::OcTreeKey>& original_path, double tolerance);

  double map_conversion_time_;
};
}  // namespace mrs_subt_planning

#endif
