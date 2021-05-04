#ifndef __ASTAR_PLANNER_H__
#define __ASTAR_PLANNER_H__

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <mrs_lib/param_loader.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/MarkerArray.h>
#include "darpa_planning_lib/pcl_map.h"


namespace darpa_planning
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
    return (key.k[0] == b.key.k[0]) && (key.k[1] == b.key.k[1]) && (key.k[2] == b.key.k[2]);
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
  return (lhs.key.k[0] == rhs.key.k[0]) && (lhs.key.k[1] == rhs.key.k[1]) && (lhs.key.k[2] == rhs.key.k[2]);
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
                  bool enable_planning_to_unreachable_goal, double planning_timeout_, double safe_dist, double clearing_dist, bool debug,
                  bool resolution_increased, const bool break_at_timeout = false); // for backward compatibility only 

  void initialize(bool enable_planning_to_unreachable_goal, double planning_timeout_, double safe_dist, double clearing_dist, bool debug,
                  const bool break_at_timeout = false);

  std::vector<Node>               getNodePath(); // for backward compatibility only
  std::vector<Node>               getNodePath(octomap::point3d start_point, octomap::point3d goal_point, std::shared_ptr<octomap::OcTree> planning_octree, bool resolution_increased = false);
  std::vector<Node>               getNodePath(std::vector<octomap::point3d>, std::shared_ptr<octomap::OcTree> planning_octree, bool resolution_increased = false);
  std::vector<octomap::point3d>   getWaypointPath(std::vector<Node> node_path);
  std::vector<octomap::point3d>   getWaypointPath(std::vector<octomap::OcTreeKey> key_path);
  std::vector<octomap::point3d>   getLocalPath(std::vector<Node> node_path);
  std::vector<octomap::OcTreeKey> getSafePath(std::vector<octomap::OcTreeKey> key_path, double safe_dist, int max_iteration, double z_diff_tolerance,
                                              bool fix_goal_point);
  std::vector<octomap::OcTreeKey> getFilteredPlan(std::vector<octomap::OcTreeKey> original_path, int size_of_window, double enabled_filtering_dist);
  std::pair<int, int> firstUnfeasibleNodeInPath(std::vector<octomap::OcTreeKey> key_waypoints, geometry_msgs::PoseArray pose_array, int n_points_forward,
                                                octomap::point3d current_pose, double safe_dist_for_replanning_, double critical_dist_for_replanning);
  void                setPlanningOctree(std::shared_ptr<octomap::OcTree> new_map);
  octomap::point3d    getLastFoundGoal();
  std::vector<octomap::OcTreeKey> getKeyPath(std::vector<Node> plan);
  void                            setStartAndGoal(octomap::point3d start_pose, octomap::point3d goal_pose);
  void                            setVerbose(const bool verbose);
  void                            setSafeDist(const double safe_dist);

protected:
  PCLMap pcl_map_;

  double max_planning_time;

  ros::Publisher pub_pc;
  ros::Publisher pub_octree;
  ros::Publisher pub_debug;

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

  bool                            isNodeValid(Node n);
  bool                            isNodeGoal(Node n);
  bool                            isNodeGoal(Node n, Node goal);
  double                          euclideanCost(Node n);
  double                          manhattanCost(Node n);
  std::vector<Node>               getNeighborhood6(Node n);
  std::vector<Node>               getNeighborhood26(Node n);
  std::vector<octomap::OcTreeKey> getKeyNeighborhood6(octomap::OcTreeKey k);
  std::vector<octomap::OcTreeKey> getKeyNeighborhood26(octomap::OcTreeKey k);
  double                          nodeDistance(Node a, Node b);
  bool                            checkValidityWithNeighborhood(Node a);
  bool                            checkValidityWithNeighborhood(octomap::OcTreeKey k);
  std::vector<pcl::PointXYZ>      octomapToPointcloud(std::vector<int> map_limits);
  std::vector<pcl::PointXYZ>      octomapToPointcloud();
  std::vector<int>                getMapLimits(std::vector<octomap::OcTreeKey> plan, int start_index, int end_index, int xy_reserve, int z_reserve);
  bool                            checkLimits(std::vector<int> map_limits, Node n);
  double                          generateObstacleCost(double dist);
  bool                            isNodeInTheNeighborhood(octomap::OcTreeKey n, octomap::OcTreeKey center, double dist);
  octomap::OcTreeKey              getBestNeighbor(octomap::OcTreeKey k);
  int                             isConnectivityMaintained(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  octomap::OcTreeKey              findConnection(octomap::OcTreeKey k1, octomap::OcTreeKey k2, double original_obs_dist);
  pcl::PointXYZ                   octomapKeyToPclPoint(octomap::OcTreeKey octo_key);
  std::vector<octomap::OcTreeKey> generatePossibleConnectionsDiagonalKeys(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  bool                            areKeysInNeighborhood(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  bool                            areKeysEqual(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  octomap::OcTreeKey              getConnectionNode(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  int                             keyManhattanDist(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  double                          keyEuclideanDist(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  int                             nofDifferentKeyCoordinates(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<octomap::OcTreeKey> getAdditionalWaypoints(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<int>                getDifferenceInCoordinates(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<std::vector<octomap::OcTreeKey>> getPossibleWaypointsForTwoDiffCoord(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<std::vector<octomap::OcTreeKey>> getPossibleWaypointsForThreeDiffCoord(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<octomap::OcTreeKey>              getSafestWaypointsBetweenKeys(std::vector<std::vector<octomap::OcTreeKey>> possible_waypoints);
  std::vector<octomap::OcTreeKey>              getStraightenKeyPath(std::vector<octomap::OcTreeKey> key_path);
  bool                                         areKeysDiagonalNeighbors(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<octomap::OcTreeKey>              getSmoothPath(std::vector<octomap::OcTreeKey> path);
  Node                                         getValidNodeInNeighborhood(Node goal);
  bool                                         checkValidityWithKDTree(Node n);
  bool                                         checkValidityWithKDTree(octomap::OcTreeKey k);
  std::vector<octomap::OcTreeKey>              getKeyVectorFromCoordinates(geometry_msgs::PoseArray pose_array);
  double                                       getDistFactorOfNeighbors(octomap::OcTreeKey c);

  // params
  bool   use_neighborhood_6_;
  double planning_timeout_;
  double resolution_;
  bool   enable_planning_to_unreachable_goal_;
  bool   debug_;
  bool   verbose_;
  bool   initialized_;
  double safe_dist_;
  double clearing_dist_;
  bool   resolution_increased_;
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
  std::vector<Node>                          getPossibleSuccessors(octomap::OcTreeKey parent, octomap::OcTreeKey current);
  octomap::OcTreeKey                         getSafestWaypointsBetweenKeys(std::vector<octomap::OcTreeKey> possible_waypoints);
  octomap::OcTreeKey                         getConnectionNode3d(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<octomap::OcTreeKey>            getAdditionalWaypoints3d(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  octomap::OcTreeKey                         getBestNeighborEscape(octomap::OcTreeKey c, octomap::OcTreeKey prev);
  void                                       publishPoints(octomap::OcTreeKey current, octomap::OcTreeKey best_neigh, octomap::OcTreeKey last_added,
                                                           std::vector<octomap::OcTreeKey> additional_waypoints);
  std::vector<octomap::OcTreeKey>            getFilteredNeighborhoodPlan(std::vector<octomap::OcTreeKey> plan);
  bool                                       areKeysInTwoStepsDistance(octomap::OcTreeKey k1, octomap::OcTreeKey k2);
  std::vector<octomap::OcTreeKey>            getFilteredNeighborhoodPlan2(std::vector<octomap::OcTreeKey> original_path);
  std::vector<octomap::OcTreeKey>            getZzFilteredPlan(std::vector<octomap::OcTreeKey> original_path, double tolerance);
};
}  // namespace darpa_planning

#endif
