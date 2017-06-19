/***
 *  Software License Agreement: BSD 3-Clause License
 *  
 *  Copyright (c) 2016-2017, qbrobotics®
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *  
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *  
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *  
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QB_MOVE_TRANSMISSION_H
#define QB_MOVE_TRANSMISSION_H

#include <transmission_interface/transmission.h>
#include <control_toolbox/filters.h>
//TODO: switch to a more complete filter library

namespace qb_move_transmission_interface {
/**
 * The qbrobotics \em qbmove Transmission interface implements the specific \p transmission_interface::Transmission to
 * convert from \em qbmove motors state to its equivalent joint state representation, and vice versa.
 * \sa qb_device_transmission_interface::qbDeviceTransmissionResources
 */
class qbMoveTransmission : public transmission_interface::Transmission {
 public:
  /**
   * Build the \em qbmove transmission with default velocity and effort scale factors (respectively \p 0.2 and \p 0.001),
   * and retrieve the other scale factors from the Parameter Server. The position factor uses the private \p
   * "~encoder_resolutions" param, following the formula \f$f_{pos} = \frac{\pi}{2^(2-encoder\_resolutions)}\f$ (if not
   * found it uses the default resolution, which is \p 1); the preset factor uses the private \p "~preset_ticks_limit"
   * param, following the formula \f$f_{pres} = \frac{1}{preset\_ticks\_limit}\f$ (if not found it uses the default
   * limit, which is \p 3000 \em ticks). The control mode can be set to work either with motor positions or with shaft
   * position and stiffness preset using the private parameter \p "~command_with_position_and_preset" (by default it
   * is \p true).
   */
  qbMoveTransmission()
      : qbMoveTransmission(ros::param::param<bool>("~command_with_position_and_preset", true),
                           M_PI/std::pow(2, 15-ros::param::param<int>("~encoder_resolutions", 1)),
                           1./ros::param::param<int>("~preset_ticks_limit", 3000)) {}

  /**
   * Build the \em qbmove transmission with the given scale factors and the default velocity and effort scale factors
   * (respectively \p 0.2 and \p 0.001).
   * \param command_with_position_and_preset If \p true the controller exploits shaft position and stiffness preset.
   * \param position_factor Motor position \em ticks to joint position \em radians \em scale factor.
   * \param preset_factor Preset value \em ticks to preset percent value [\p 0, \p 1] scale factor.
   */
  qbMoveTransmission(const bool &command_with_position_and_preset, const double &position_factor, const double &preset_factor)
      : qbMoveTransmission(command_with_position_and_preset, position_factor, preset_factor, 0.2, 0.001) {}

  /**
   *
   * Build the \em qbmove transmission with the given scale factors.
   * \param command_with_position_and_preset If \p true the controller exploits shaft position and stiffness preset.
   * \param position_factor Motor position \em ticks to joint position \em radians \em scale factor.
   * \param preset_factor Preset value \em ticks to preset percent value [\p 0, \p 1] scale factor.
   * \param velocity_factor Exponential filter smoothing factor.
   * \param effort_factor Motor current \em mA to joint effort \em A scale factor.
   */
  qbMoveTransmission(const bool &command_with_position_and_preset, const double &position_factor, const double &preset_factor, const double &velocity_factor, const double &effort_factor)
      : Transmission(),
        command_with_position_and_preset_(command_with_position_and_preset),
        position_factor_(position_factor),
        preset_factor_(preset_factor),
        velocity_factor_(velocity_factor),
        effort_factor_(effort_factor) {}

  /**
   * Transform \em effort variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointEffort(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && actuator.effort[1] && joint.effort[0] && joint.effort[1] && joint.effort[2] && joint.effort[3]);

    *joint.effort[0] = *actuator.effort[0] * effort_factor_;  // motor_1_joint [A]
    *joint.effort[1] = *actuator.effort[1] * effort_factor_;  // motor_2_joint [A]
    *joint.effort[2] = (*actuator.effort[0] + *actuator.effort[1]) * effort_factor_ / 2;  // sort of current average [A]
    *joint.effort[3] = 0.0;  // meaningless
  }

  /**
   * Transform \em velocity variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointVelocity(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && actuator.velocity[1] && joint.velocity[0] && joint.velocity[1] && joint.velocity[2] && joint.velocity[3]);

    // note: be aware that this method _misuses_ actuator.velocity to store the current measured velocity
    //   - *actuator.velocity[i] is the current measured velocity of the motor i in [ticks/s]
    //   - *joint.velocity[i] is the previous step velocity in [radians/s] or [percent/s]
    *joint.velocity[0] = filters::exponentialSmoothing(*actuator.velocity[0] * position_factor_, *joint.velocity[0], velocity_factor_);  // motor_1_joint [radians/s]
    *joint.velocity[1] = filters::exponentialSmoothing(*actuator.velocity[1] * position_factor_, *joint.velocity[1], velocity_factor_);  // motor_2_joint [radians/s]
    *joint.velocity[2] = filters::exponentialSmoothing(*actuator.velocity[2] * position_factor_, *joint.velocity[2], velocity_factor_);  // shaft_joint [radians/s]
    *joint.velocity[3] = (*joint.velocity[0] - *joint.velocity[1]) / 2;  // stiffness preset [percent/s]
  }

  /**
   * Transform \em position variables from actuator to joint space.
   * \param actuator Actuator-space variables.
   * \param[out] joint Joint-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void actuatorToJointPosition(const transmission_interface::ActuatorData& actuator, transmission_interface::JointData& joint) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && actuator.position[1] && joint.position[0] && joint.position[1] && joint.position[2] && joint.position[3]);

    *joint.position[0] = *actuator.position[0] * position_factor_;  // motor_1_joint [radians]
    *joint.position[1] = *actuator.position[1] * position_factor_;  // motor_2_joint [radians]
    *joint.position[2] = *actuator.position[2] * position_factor_;  // shaft_joint [radians]
    *joint.position[3] = std::abs(*actuator.position[0] - *actuator.position[1]) * preset_factor_ / 2;  // stiffness_preset = abs(motor_1 - motor_2)/2 [percent 0,1]
  }

  /**
   * \return \p true if the controller exploits shaft position and stiffness preset.
   */
  inline const double getCommandWithPoistionAndPreset() const { return command_with_position_and_preset_; }

  /**
   * \return The current position scale factor.
   */
  inline const double getPositionFactor() const { return position_factor_; }

  /**
   * \return The current preset scale factor.
   */
  inline const double getPresetFactor() const { return preset_factor_; }

  /**
   * \return The current velocity scale factor.
   */
  inline const double getVelocityFactor() const { return velocity_factor_; }

  /**
   * \return The current effort scale factor.
   */
  inline const double getEffortFactor() const { return effort_factor_; }

  /**
   * Transform \em effort variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint effort vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorEffort(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.effort.size() && numJoints() == joint.effort.size());
    ROS_ASSERT(actuator.effort[0] && actuator.effort[1] && joint.effort[0] && joint.effort[1] && joint.effort[2] && joint.effort[3]);

    if (command_with_position_and_preset_) {
      // the qbmove cannot be controlled in current from shaft and preset info
      *actuator.effort[0] = 0.0;
      *actuator.effort[1] = 0.0;
      *actuator.effort[2] = 0.0;
    }
    else {  // direct motors control
      // not used yet, but this could help in the near future
      *actuator.effort[0] = *joint.effort[0] / effort_factor_;
      *actuator.effort[1] = *joint.effort[1] / effort_factor_;
      *actuator.effort[2] = 0.0;
    }
  }

  /**
   * Transform \em velocity variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint velocity vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorVelocity(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.velocity.size() && numJoints() == joint.velocity.size());
    ROS_ASSERT(actuator.velocity[0] && actuator.velocity[1] && joint.velocity[0] && joint.velocity[1] && joint.velocity[2] && joint.velocity[3]);

    // the qbmove cannot be controlled in velocity
    *actuator.velocity[0] = 0.0;
    *actuator.velocity[1] = 0.0;
    *actuator.velocity[2] = 0.0;
  }

  /**
   * Transform \em position variables from joint to actuator space.
   * \param joint Joint-space variables.
   * \param[out] actuator Actuator-space variables.
   * \pre Actuator and joint position vectors must have size according respectively to \p numActuators and \p numJoints
   * and must point to valid data, at least for the used fields, i.e. it is not required that all other data vectors
   * contain valid data; they can even remain empty.
   */
  inline void jointToActuatorPosition(const transmission_interface::JointData& joint, transmission_interface::ActuatorData& actuator) {
    ROS_ASSERT(numActuators() == actuator.position.size() && numJoints() == joint.position.size());
    ROS_ASSERT(actuator.position[0] && actuator.position[1] && joint.position[0] && joint.position[1] && joint.position[2] && joint.position[3]);

    if (command_with_position_and_preset_) {
      *actuator.position[0] = *joint.position[2] / position_factor_ + *joint.position[3] / preset_factor_;  // motor_1 = shaft + preset [ticks]
      *actuator.position[1] = *joint.position[2] / position_factor_ - *joint.position[3] / preset_factor_;  // motor_2 = shaft - preset [ticks]
      *actuator.position[2] = 0.0;
    }
    else {  // direct motors control
      *actuator.position[0] = *joint.position[0] / position_factor_;  // motor_1 = motor_1_command [ticks]
      *actuator.position[1] = *joint.position[1] / position_factor_;  // motor_2 = motor_2_command [ticks]
      *actuator.position[2] = 0.0;
    }
  }

  /**
   * \return The number of actuators of this transmission, i.e always 2 for the \em qbmove.
   */
  inline std::size_t numActuators() const { return 3; }

  /**
   * \return The number of joints of this transmission, i.e always 4 for the \em qbmove.
   */
  inline std::size_t numJoints() const { return 4; }

 private:
  bool command_with_position_and_preset_;
  double position_factor_;
  double preset_factor_;
  double velocity_factor_;
  double effort_factor_;
};
}  // namespace qb_move_transmission_interface

#endif // QB_MOVE_TRANSMISSION_H