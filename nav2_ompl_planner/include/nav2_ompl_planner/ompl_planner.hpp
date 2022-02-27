#ifndef NAV2_OMPL_PLANNER__OMPL_PLANNER_HPP_
#define NAV2_OMPL_PLANNER__OMPL_PLANNER_HPP_

#include <string>
#include <memory>
#include <type_traits>

#include "ompl/base/State.h"
#include "ompl/base/StateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace nav2_ompl_planner
{

class OMPLPlanner : public nav2_core::GlobalPlanner
{
public:
  OMPLPlanner();
  ~OMPLPlanner() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Point conversion
  geometry_msgs::msg::PoseStamped convertToPoseStamped(const ompl::base::State * state);

  // State validity checker
  bool isStateValid(const ompl::base::State * state);

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string name_;

  // Setup Ptr
  ompl::geometric::SimpleSetupPtr ss_;

  // State Space Ptr
  ompl::base::StateSpacePtr ompl_state_space_;

  //
  bool allow_unknown_;

  double solve_time_;

  double collision_checking_resolution_;

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker_;
};

}  // namespace nav2_ompl_planner

#endif  // NAV2_OMPL_PLANNER__OMPL_PLANNER_HPP_
