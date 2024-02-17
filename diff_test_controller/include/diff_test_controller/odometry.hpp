#ifndef __ODOMETRY_HPP_
#define __ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace diff_test_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool update(double left_vel, double right_vel, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  void integrate(double linear, double angular, double dt);
  void resetAccumulators();

  // 当前时间戳:
  rclcpp::Time timestamp_;

  // 当前位置:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  //当前速度:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_;
  double left_wheel_radius_;
  double right_wheel_radius_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace diff_test_controller

#endif  // __ODOMETRY_HPP_