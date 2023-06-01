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

#include <cmath>
#include <algorithm>
#include <limits>

#include "ros2_pid/PIDController.hpp"

#include "rclcpp/rclcpp.hpp"

namespace ros2_pid
{

PIDController::PIDController()
{
}

PIDController::PIDController(rclcpp::Node::SharedPtr node)
{
  init(node);
}

void
PIDController::init(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  curr_time_ = prev_time_ = node_->now();
}

void
PIDController::set_p(double n_KP)
{
  if (n_KP < 0.0) {
    return;
  }

  KP_ = n_KP;
}

void
PIDController::set_i(double n_KI)
{
  if (n_KI < 0.0) {
    return;
  }

  KI_ = n_KI;
}

void
PIDController::set_d(double n_KD)
{
  if (n_KD < 0.0) {
    return;
  }

  KD_ = n_KD;
}

void
PIDController::set_pid(double n_KP, double n_KI, double n_KD)
{
  if (n_KP < 0.0 || n_KI < 0.0 || n_KD < 0.0) {
    return;
  }

  KP_ = n_KP;
  KI_ = n_KI;
  KD_ = n_KD;
}

void
PIDController::set_setpoint(double new_setpoint)
{
  setpoint_ = new_setpoint;
}

void
PIDController::set_reference(double new_reference)
{
  new_reference_ = new_reference;
}

void
PIDController::set_min_err(double new_min_err)
{
  min_err_ = fabs(new_min_err);
}

void
PIDController::set_abs_min_output(double new_abs_min_output)
{
  abs_min_output_ = fabs(new_abs_min_output);
}

void
PIDController::set_min_output(double new_min_output)
{
  min_output_ = new_min_output;
}

void
PIDController::set_max_output(double new_max_output)
{
  max_output_ = new_max_output;
}

void
PIDController::set_symmetric_output_limits(double new_output_limit)
{
  min_output_ = -new_output_limit;
  max_output_ = new_output_limit;
}

void
PIDController::set_derivative_mode(DerivativeMode new_derivative_mode)
{
  derivative_mode_ = new_derivative_mode;
}

void
PIDController::set_control_mode(ControlMode new_control_mode)
{
  if (new_control_mode == ControlMode::AUTOMATIC) {
    prev_reference_ = new_reference_;
    integral_term_ = output_;
    clamp_integral_term_();
  }
  control_mode_ = new_control_mode;
}

void
PIDController::set_direction(Direction new_direction)
{
  if (new_direction != controller_direction_) {
    KP_ = KP_ * -1.0;
    KI_ = KI_ * -1.0;
    KD_ = KD_ * -1.0;
  }
  controller_direction_ = new_direction;
}

void
PIDController::set_time_mode(TimeMode new_time_mode)
{
  time_mode_ = new_time_mode;
}

double
PIDController::get_p()
{
  return KP_;
}

double
PIDController::get_i()
{
  return KI_;
}

double
PIDController::get_d()
{
  return KD_;
}

double
PIDController::get_setpoint()
{
  return setpoint_;
}

double
PIDController::get_min_err()
{
  return min_err_;
}

double
PIDController::get_abs_min_output()
{
  return abs_min_output_;
}

double
PIDController::get_min_output()
{
  return min_output_;
}

double
PIDController::get_max_output()
{
  return max_output_;
}

double
PIDController::get_output()
{
  return output_;
}

PIDController::DerivativeMode
PIDController::get_derivative_mode()
{
  return derivative_mode_;
}

PIDController::ControlMode
PIDController::get_control_mode()
{
  return control_mode_;
}

PIDController::Direction
PIDController::get_direction()
{
  return controller_direction_;
}

PIDController::TimeMode
PIDController::get_time_mode()
{
  return time_mode_;
}

void
PIDController::reset()
{
  curr_error_ = 0.0;
  prev_error_ = std::numeric_limits<double>::quiet_NaN();
  sum_error_ = 0.0;
  integral_term_ = 0.0;
  proportional_term_ = 0.0;
  derivative_term_ = 0.0;
  derivative_error_ = 0.0;
  new_reference_ = 0.0;
  prev_reference_ = std::numeric_limits<double>::quiet_NaN();
  delta_reference_ = 0.0;

  output_ = std::numeric_limits<double>::quiet_NaN();
}

void
PIDController::reset_integral()
{
  reset_integral_();
}

void
PIDController::compute()
{
  if (node_ == nullptr) {
    output_ = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  if (control_mode_ == ControlMode::MANUAL) {
    output_ = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  if (setpoint_ == std::numeric_limits<double>::quiet_NaN()) {
    output_ = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  compute_();
}

void
PIDController::compute_()
{
  curr_time_ = node_->now();
  delta_time_ = curr_time_ - prev_time_;
  delta_time_seconds_ = delta_time_.seconds();

  curr_error_ = new_reference_ - setpoint_;

  double curr_error_sign = std::signbit(curr_error_) ? -1.0 : 1.0;
  double prev_error_sign = std::signbit(prev_error_) ? -1.0 : 1.0;

  if (curr_error_sign != prev_error_sign) {
    reset_integral_();
  }

  if (fabs(curr_error_) < min_err_) {
    curr_error_ = 0.0;
  }

  proportional_term_ = KP_ * curr_error_;

  sum_error_ += curr_error_ * delta_time_seconds_;

  clamp_sum_error_();

  integral_term_ += KI_ * sum_error_;

  clamp_integral_term_();

  derivative_term_ = get_derivative_term_();

  output_ = proportional_term_ + integral_term_ + derivative_term_;

  output_ = std::clamp(output_, min_output_, max_output_);

  if (fabs(output_) < abs_min_output_) {
    output_ = 0.0;
  }

  prev_time_ = curr_time_;
  prev_error_ = curr_error_;
  prev_reference_ = new_reference_;
}

double
PIDController::get_derivative_term_()
{
  switch (derivative_mode_) {
    case DerivativeMode::ON_ERROR:
      if (std::isnan(prev_error_)) {
        return 0.0;
      }
      derivative_error_ = (curr_error_ - prev_error_) / delta_time_seconds_;
      return KD_ * derivative_error_;
    case DerivativeMode::ON_MEASUREMENT:
      if (std::isnan(prev_reference_)) {
        return 0.0;
      }
      delta_reference_ = new_reference_ - prev_reference_;
      return KD_ * delta_reference_ * -1.0;
    default:
      return 0.0;
  }
}

void
PIDController::clamp_sum_error_()
{
  sum_error_ = std::clamp(sum_error_, min_output_, max_output_);
}

void
PIDController::clamp_integral_term_()
{
  integral_term_ = std::clamp(integral_term_, min_output_, max_output_);
}

void
PIDController::reset_integral_()
{
  sum_error_ = 0.0;
  integral_term_ = 0.0;
}

void
PIDController::log_settings()
{
  rclcpp::Logger logger = node_->get_logger().get_child("PIDController");

  if (node_ == nullptr) {
    RCLCPP_ERROR(logger, "PID is not initialized!");
    return;
  }

  RCLCPP_INFO(logger, "PID settings:");
  RCLCPP_INFO(logger, "  KP: %f", KP_);
  RCLCPP_INFO(logger, "  KI: %f", KI_);
  RCLCPP_INFO(logger, "  KD: %f", KD_);
  RCLCPP_INFO(logger, "  Setpoint: %f", setpoint_);
  RCLCPP_INFO(logger, "  Min error: %f", min_err_);
  RCLCPP_INFO(logger, "  Abs min output: %f", abs_min_output_);
  RCLCPP_INFO(logger, "  Min output: %f", min_output_);
  RCLCPP_INFO(logger, "  Max output: %f", max_output_);

  switch (derivative_mode_) {
    case DerivativeMode::ON_ERROR:
      RCLCPP_INFO(logger, "  Derivative mode: ON_ERROR");
      break;
    case DerivativeMode::ON_MEASUREMENT:
      RCLCPP_INFO(logger, "  Derivative mode: ON_MEASUREMENT");
      break;
    default:
      RCLCPP_INFO(logger, "  Derivative mode: UNKNOWN");
      break;
  }

  switch (control_mode_) {
    case ControlMode::MANUAL:
      RCLCPP_INFO(logger, "  Control mode: MANUAL");
      break;
    case ControlMode::AUTOMATIC:
      RCLCPP_INFO(logger, "  Control mode: AUTOMATIC");
      break;
    default:
      RCLCPP_INFO(logger, "  Control mode: UNKNOWN");
      break;
  }

  switch (controller_direction_) {
    case Direction::DIRECT:
      RCLCPP_INFO(logger, "  Direction: DIRECT");
      break;
    case Direction::REVERSE:
      RCLCPP_INFO(logger, "  Direction: REVERSE");
      break;
    default:
      RCLCPP_INFO(logger, "  Direction: UNKNOWN");
      break;
  }

  switch (time_mode_) {
    case TimeMode::SYNCRONOUS:
      RCLCPP_INFO(logger, "  Time mode: SYNCRONOUS");
      break;
    case TimeMode::ASYNCRONOUS:
      RCLCPP_INFO(logger, "  Time mode: ASYNCRONOUS");
      break;
    default:
      RCLCPP_INFO(logger, "  Time mode: UNKNOWN");
      break;
  }

  RCLCPP_INFO(logger, "PID state:");
  RCLCPP_INFO(logger, "  Current error: %f", curr_error_);
  RCLCPP_INFO(logger, "  Proportional term: %f", proportional_term_);
  RCLCPP_INFO(logger, "  Integral term: %f", integral_term_);
  RCLCPP_INFO(logger, "  Derivative term: %f", derivative_term_);

  RCLCPP_INFO(logger, "PID I/O:");
  RCLCPP_INFO(logger, "  Input: %f", new_reference_);
  RCLCPP_INFO(logger, "  Output: %f", output_);
}

}  // namespace ros2_pid
