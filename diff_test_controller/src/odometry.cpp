#include "diff_test_controller/odometry.hpp"

namespace diff_test_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  left_wheel_radius_(0.0),
  right_wheel_radius_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  //复位滤波器和时间戳
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double left_vel, double right_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // 运动学正解计算速度和角速度
  const double linear = (left_vel + right_vel) / 2;
  const double angular = (right_vel - left_vel) / wheel_separation_;

  // 里程计积分
  integrate(linear , angular, dt);

  timestamp_ = time;

  //速度值滤波
  linear_accumulator_.accumulate(linear / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation, double left_wheel_radius, double right_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}


void Odometry::integrate(double linear, double angular, double dt)
{
  if (fabs(angular) < 1e-6)
  {
    //若角速度过小，认为基本在走直线，采用二阶龙格-库塔法积分
    const double direction = heading_ + angular * 0.5;
    x_ += linear * cos(direction) * dt;
    y_ += linear * sin(direction) * dt;
    heading_ += angular * dt;
  }
  else
  {
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular * dt;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace diff_drive_controller