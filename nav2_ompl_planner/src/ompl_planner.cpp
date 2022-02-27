#include <string>
#include <memory>
#include <cmath>
#include <fstream>
#include <chrono>

#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/OptimizationObjective.h"

#include "tf2/utils.h"
#include "nav2_ompl_planner/ompl_planner.hpp"
#include "nav2_util/node_utils.hpp"


namespace nav2_ompl_planner
{
// TODO Maybe my core assumption has been wrong? -- Ompl does do this fine?
// TODO LATER clean up headers / class definitions
// TODO LATER store this code? stevemacenski? test in smac? Is there a reason in nav2? (speed, quality, niche, ?)
// move things that can be initialized into configure / setup TODO LATER
// TODO LATER maybe this is a nice student project for those asking
// support local trajectory planning using OMPL
// TODO LATER store weak ptr / logger so no keeping locked
// TODO LATER unused parameters; parameterize planner
 //TODO LATER parameterize reeds shepp turning radius
// Optimal planning objectives -- cost map or penalizing near obstacles?
  // https://ompl.kavrakilab.org/optimalPlanningTutorial.html
  // https://ompl.kavrakilab.org/optimizationObjectivesTutorial.html
  // http://ompl.kavrakilab.org/2013/10/03/extending-ompl-support-for-optimal-path-planning-2.html
  // https://ompl.kavrakilab.org/namespaceompl_1_1base.html "Objective"s
// Build a framework on top of it with plugin-based objectives to optimize for
// parameter for planners that take objectives into account
// Other sampling space types too?

// TODO STEVE penalty for reversing + tune
class PathReversePenaltyCost : public ompl::base::PathLengthOptimizationObjective
{
public:
  PathReversePenaltyCost(const ompl::base::SpaceInformationPtr &si) : PathLengthOptimizationObjective(si)
  {
  }

  ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
  {
    auto path = si_->getStateSpace()->as<ompl::base::ReedsSheppStateSpace>()->reedsShepp(s1, s2);
    double cost = 0.0;
    // cost for reversing is 10 * length of reverse segment squared.
    // cost for going forward is simply the length
    // TODO is this the right formulation to balance?
    for (auto& l : path.length_) {          
      cost += l < 0.0 ? 10.0 * l * l : l;
    }
    return ompl::base::Cost(cost);
  }
};

// TODO STEVE penalty for costmap + tune (normalize?)
class CostMapObjective : public ompl::base::StateCostIntegralObjective {
  public:

  CostMapObjective(const ompl::base::SpaceInformationPtr & si, const nav2_costmap_2d::Costmap2D * costmap)
  : ompl::base::StateCostIntegralObjective(si, true), costmap_(costmap)
  {
  }

  ompl::base::Cost stateCost(const ompl::base::State * s) const override
  {
    const double wx(s->as<ompl::base::SE2StateSpace::StateType>()->getX());
    const double wy(s->as<ompl::base::SE2StateSpace::StateType>()->getY());
    int mx, my;
    costmap_->worldToMapEnforceBounds(wx, wy, mx, my);
    const double cost = static_cast<double>(costmap_->getCost(mx, my));
    return ompl::base::Cost(cost);
  }

  protected:
    const nav2_costmap_2d::Costmap2D * costmap_;
};

OMPLPlanner::OMPLPlanner()
: node_(nullptr), costmap_(nullptr), collision_checker_(nullptr) {}

void OMPLPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  collision_checker_ =
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>(costmap_);

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".solve_time",
    rclcpp::ParameterValue(10.0));
  node_->get_parameter(name_ + ".solve_time", solve_time_); 
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".collision_checking_resolution", rclcpp::ParameterValue(
      0.005));
  node_->get_parameter(name_ + ".collision_checking_resolution", collision_checking_resolution_);
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(
      true));
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  RCLCPP_INFO(
    node_->get_logger(), "Configuring plugin %s of type OMPLPlanner",
    name_.c_str());
}

void OMPLPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type OMPLPlanner",
    name_.c_str());
}

void OMPLPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type OMPLPlanner",
    name_.c_str());
}

void OMPLPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type OMPLPlanner",
    name_.c_str());
}

nav_msgs::msg::Path OMPLPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(node_->get_logger(), "Passed init of type OMPLPlanner");

  // Creating the Ackermann state space
  ompl_state_space_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(0.4);

  // Setting valid sampling bounds on the costmap
  auto bounds = ompl::base::RealVectorBounds(2);
  bounds.setLow(0, costmap_->getOriginX());
  bounds.setHigh(0, costmap_->getOriginX() + costmap_->getSizeInMetersX());
  bounds.setLow(1, costmap_->getOriginY());
  bounds.setHigh(1, costmap_->getOriginY() + costmap_->getSizeInMetersY());
  ompl_state_space_->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

  // Creating the problem and setting up the collision checker
  ss_.reset(new ompl::geometric::SimpleSetup(ompl_state_space_));
  ss_->setStateValidityChecker(
    [this](const ompl::base::State * state) -> bool {
      return this->isStateValid(state);
    });
  ss_->getSpaceInformation()->setStateValidityCheckingResolution(collision_checking_resolution_);

  // Initializing start, goal, and path
  nav_msgs::msg::Path path;

  ompl::base::ScopedState<> ompl_start(ompl_state_space_);
  ompl::base::ScopedState<> ompl_goal(ompl_state_space_);

  ompl_start[0] = start.pose.position.x;
  ompl_start[1] = start.pose.position.y;
  // ompl_start[2] = tf2::getYaw(start.pose.orientation); //TODO

  ompl_goal[0] = goal.pose.position.x;
  ompl_goal[1] = goal.pose.position.y;
  // ompl_goal[2] = tf2::getYaw(goal.pose.orientation); //TODO

  ss_->setStartAndGoalStates(ompl_start, ompl_goal);

  // Creating objective functions to help minimize cost, path length, and reversing maneuvers
  // TODO why don't these appear to be being followed by the planner? Just not enough compute time even with 5 seconds? weights? Something else?
    // getting plans STILL with reversing segments, and awfully close to obstacles. I cant even tell if the length objective is making an impact
    // lower values seem to be working better?
  ompl::base::OptimizationObjectivePtr cost_objective(
      new CostMapObjective(ss_->getSpaceInformation(), costmap_));
  ompl::base::OptimizationObjectivePtr length_objective(
      new ompl::base::PathLengthOptimizationObjective(ss_->getSpaceInformation()));
  ompl::base::OptimizationObjectivePtr reverse_objective(new PathReversePenaltyCost(ss_->getSpaceInformation()));

  std::shared_ptr<ompl::base::MultiOptimizationObjective> opt(new ompl::base::MultiOptimizationObjective(ss_->getSpaceInformation()));
  opt->addObjective(cost_objective, 1e-3); // TODO tune penalties
  opt->addObjective(length_objective, 1.0);
  opt->addObjective(reverse_objective, 100.0);
  ss_->setOptimizationObjective(opt);

  // Setting optimal planner
  // using RRT* will use the optimization objective, KPIECE will not, but also not really working; TODO other options?
  ss_->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss_->getSpaceInformation()));

  // Solving the planning problem
  auto problemDef = ss_->getProblemDefinition();
  auto cct = ompl::base::CostConvergenceTerminationCondition(problemDef, 1, 1); // TODO tune, termination conditions just going rampedly
  ss_->setup();
  if (ss_->solve(cct)) {
    RCLCPP_INFO(node_->get_logger(), "Path found!");
    ss_->simplifySolution(); // TODO termination args / time
    auto solution_path = ss_->getSolutionPath();
    path.header.stamp = node_->now();
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    int min_num_states = round(solution_path.length() / costmap_->getResolution());
    solution_path.interpolate(min_num_states);

    path.poses.reserve(solution_path.getStates().size());
    for (const auto ptr : solution_path.getStates()) {
      path.poses.push_back(convertToPoseStamped(ptr));
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Path not found!");
  }

  auto stop_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
  std::cout << duration.count() << " milliseconds to plan path."  << std::endl;

  return path;
}

geometry_msgs::msg::PoseStamped OMPLPlanner::convertToPoseStamped(const ompl::base::State * state)
{
  ompl::base::ScopedState<> ss(ompl_state_space_);
  ss = *state;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = ss[0];
  pose.pose.position.y = ss[1];
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = sin(0.5 * ss[2]);
  pose.pose.orientation.w = cos(0.5 * ss[2]);
  return pose;
}

bool OMPLPlanner::isStateValid(const ompl::base::State * state)
{
  if (!ss_->getSpaceInformation()->satisfiesBounds(state)) {
    return false;
  }

  ompl::base::ScopedState<> ss(ompl_state_space_);
  ss = state;
  auto cost = collision_checker_.footprintCostAtPose(ss[0], ss[1], ss[2], costmap_ros_->getRobotFootprint());
  if (cost > 252) {
    return false;
  }

  return true;
}
}  // namespace nav2_ompl_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_ompl_planner::OMPLPlanner, nav2_core::GlobalPlanner)
