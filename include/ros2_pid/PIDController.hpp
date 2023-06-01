// Copyright (c) 2023 StressOverflow
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_PID__PIDCONTROLLER_HPP_
#define ROS2_PID__PIDCONTROLLER_HPP_

#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"

namespace ros2_pid
{

class PIDController
{
public:
  PIDController();
  explicit PIDController(rclcpp::Node::SharedPtr node);

  enum class DerivativeMode
  {
    ON_ERROR,
    ON_MEASUREMENT
  };

  enum class ControlMode
  {
    MANUAL,
    AUTOMATIC
  };

  enum class Direction
  {
    DIRECT,
    REVERSE
  };

  enum class TimeMode
  {
    SYNCRONOUS,
    ASYNCRONOUS
  };

  void init(rclcpp::Node::SharedPtr node);
  void set_p(double n_KP);
  void set_i(double n_KI);
  void set_d(double n_KD);
  void set_pid(double n_KP, double n_KI, double n_KD);
  void set_setpoint(double new_setpoint);
  void set_reference(double new_reference);
  void set_min_err(double new_min_err);
  void set_abs_min_output(double new_abs_min_output);
  void set_min_output(double new_min_output);
  void set_max_output(double new_max_output);
  void set_symmetric_output_limits(double new_output_limit);
  void set_derivative_mode(DerivativeMode new_derivative_mode);
  void set_control_mode(ControlMode new_control_mode);
  void set_direction(Direction new_direction);
  void set_time_mode(TimeMode new_time_mode);

  double get_p();
  double get_i();
  double get_d();
  double get_setpoint();
  double get_min_err();
  double get_abs_min_output();
  double get_min_output();
  double get_max_output();
  double get_output();
  DerivativeMode get_derivative_mode();
  ControlMode get_control_mode();
  Direction get_direction();
  TimeMode get_time_mode();

  void reset();
  void reset_integral();
  void compute();

  void log_settings();

private:
  rclcpp::Node::SharedPtr node_ = nullptr;

  double KP_ = 0.0;
  double KI_ = 0.0;
  double KD_ = 0.0;

  double setpoint_ = std::numeric_limits<double>::quiet_NaN();

  double min_err_ = 0.0;
  double abs_min_output_ = 0.0;
  double min_output_ = std::numeric_limits<double>::lowest();
  double max_output_ = std::numeric_limits<double>::max();

  DerivativeMode derivative_mode_ = DerivativeMode::ON_ERROR;
  ControlMode control_mode_ = ControlMode::AUTOMATIC;
  Direction controller_direction_ = Direction::DIRECT;
  TimeMode time_mode_ = TimeMode::ASYNCRONOUS;

  double curr_error_ = 0.0;
  double prev_error_ = std::numeric_limits<double>::quiet_NaN();
  double sum_error_ = 0.0;
  double proportional_term_ = 0.0;
  double integral_term_ = 0.0;
  double derivative_term_ = 0.0;
  double derivative_error_ = 0.0;
  double new_reference_ = 0.0;
  double prev_reference_ = std::numeric_limits<double>::quiet_NaN();
  double delta_reference_ = 0.0;

  double output_ = std::numeric_limits<double>::quiet_NaN();

  rclcpp::Time curr_time_, prev_time_;
  rclcpp::Duration delta_time_ = curr_time_ - prev_time_;
  double delta_time_seconds_ = 0.0;

  void compute_();

  double get_derivative_term_();

  void clamp_sum_error_();
  void clamp_integral_term_();
  void reset_integral_();
};

}  // namespace ros2_pid

#endif  // ROS2_PID__PIDCONTROLLER_HPP_
