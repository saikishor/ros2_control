// Copyright 2024 PAL Robotics S.L.
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

/// \author Sai Kishor Kothakota

#ifndef JOINT_LIMITS__JOINT_LIMITER_STRUCT_HPP_
#define JOINT_LIMITS__JOINT_LIMITER_STRUCT_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"

namespace joint_limits
{

struct JointControlInterfacesData
{
  std::string joint_name;
  std::optional<double> position = std::nullopt;
  std::optional<double> velocity = std::nullopt;
  std::optional<double> effort = std::nullopt;
  std::optional<double> acceleration = std::nullopt;
  std::optional<double> jerk = std::nullopt;

  bool has_data() const
  {
    return has_position() || has_velocity() || has_effort() || has_acceleration() || has_jerk();
  }

  bool has_position() const { return position.has_value(); }

  bool has_velocity() const { return velocity.has_value(); }

  bool has_effort() const { return effort.has_value(); }

  bool has_acceleration() const { return acceleration.has_value(); }

  bool has_jerk() const { return jerk.has_value(); }
};

struct JointInterfacesCommandLimiterData
{
  std::string joint_name;
  JointControlInterfacesData actual;
  JointControlInterfacesData command;
  JointControlInterfacesData prev_command;
  JointControlInterfacesData limited;

  bool has_actual_data() const { return actual.has_data(); }

  bool has_command_data() const { return command.has_data(); }
};

template <typename JointLimitsStateDataType>
class JointLimiterInterface
{
public:
  JointLimiterInterface() = default;

  virtual ~JointLimiterInterface() = default;

  /**
   * Wrapper init method that accepts the joint names and their limits directly
   */
  virtual bool init(
    const std::vector<std::string> & joint_names,
    const std::vector<joint_limits::JointLimits> & joint_limits,
    const std::vector<joint_limits::SoftJointLimits> & soft_limits,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf)
  {
    return on_init();
  }

  /** \brief Initialize joint limits limiter.
   *
   * Generic initialization method that calls implementation-specific `on_init` method.
   * \returns true if initialization was successful, otherwise false.
   */
  virtual bool configure(const JointLimitsStateDataType & current_joint_states)
  {
    return on_configure(current_joint_states);
  }

  /** \brief Enforce joint limits to desired joint state for multiple physical quantities.
   *
   * Generic enforce method that calls implementation-specific `on_enforce` method.
   *
   * \param[in] current_joint_states current joint states a robot is in.
   * \param[in,out] desired_joint_states joint state that should be adjusted to obey the limits.
   * \param[in] dt time delta to calculate missing integrals and derivation in joint limits.
   * \returns true if limits are enforced, otherwise false.
   */
  virtual bool enforce(
    JointLimitsStateDataType & current_joint_states,
    JointLimitsStateDataType & desired_joint_states, const rclcpp::Duration & dt)
  {
    return on_enforce(current_joint_states, desired_joint_states, dt);
  }

protected:
  /** \brief Method is realized by an implementation.
   *
   * Implementation-specific initialization of limiter's internal states and libraries.
   * \returns true if initialization was successful, otherwise false.
   */
  virtual bool on_init() { return true; }

  /** \brief Method is realized by an implementation.
   *
   * Implementation-specific configuration of limiter's internal states and libraries.
   * \returns true if initialization was successful, otherwise false.
   */
  virtual bool on_configure(const JointLimitsStateDataType & current_joint_states) { return true; }

  /** \brief Method is realized by an implementation.
   *
   * Filter-specific implementation of the joint limits enforce algorithm for multiple dependent
   * physical quantities.
   *
   * \param[in] current_joint_states current joint states a robot is in.
   * \param[in,out] desired_joint_states joint state that should be adjusted to obey the limits.
   * \param[in] dt time delta to calculate missing integrals and derivation in joint limits.
   * \returns true if limits are enforced, otherwise false.
   */
  virtual bool on_enforce(
    JointLimitsStateDataType & current_joint_states,
    JointLimitsStateDataType & desired_joint_states, const rclcpp::Duration & dt)
  {
    return false;
  }
};

}  // namespace joint_limits
#endif  // JOINT_LIMITS__JOINT_LIMITER_STRUCT_HPP_
