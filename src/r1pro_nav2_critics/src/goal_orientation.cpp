#include <cmath>
#include <dwb_core/trajectory_critic.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace r1pro_nav2_critics
{
class GoalOrientationCritic : public dwb_core::TrajectoryCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D &,
    const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D &) override
  {
    // Use the true transformed goal. The cropped path may end before it.
    goal_yaw_ = goal.theta;
    goal_x_ = goal.x;
    goal_y_ = goal.y;
    return std::isfinite(goal_yaw_);
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (trajectory.poses.empty()) {
      return 0.0;
    }
    const double error = goal_yaw_ - trajectory.poses.back().theta;
    // Wrap across +/- pi so a 179 -> -179 degree goal takes the short turn.
    const auto & end = trajectory.poses.back();
    // Resolve equal grid scores within the final 5 cm costmap cells so the
    // robot continues approaching the requested pose at low speed.
    const double distance = std::hypot(end.x - goal_x_, end.y - goal_y_);
    return std::abs(std::atan2(std::sin(error), std::cos(error))) + .5 * distance;
  }

private:
  double goal_yaw_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
};
}

PLUGINLIB_EXPORT_CLASS(r1pro_nav2_critics::GoalOrientationCritic, dwb_core::TrajectoryCritic)
