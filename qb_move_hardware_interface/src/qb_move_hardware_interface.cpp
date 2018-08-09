/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2018, qbrobotics®
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

#include <qb_move_hardware_interface/qb_move_hardware_interface.h>

using namespace qb_move_hardware_interface;

qbMoveHW::qbMoveHW()
    : qbDeviceHW(std::make_shared<qb_move_transmission_interface::qbMoveTransmission>(), {"motor_1_joint", "motor_2_joint", "shaft_joint"}, {"motor_1_joint", "motor_2_joint", "shaft_joint", "stiffness_preset_virtual_joint"}) {

}

qbMoveHW::~qbMoveHW() {

}

std::vector<std::string> qbMoveHW::getJoints() {
  if (command_with_position_and_preset_) {
    return {joints_.names.at(2), joints_.names.at(3)};
  }
  return {joints_.names.at(0), joints_.names.at(1)};
}

int qbMoveHW::getMaxStiffness() {
  std::smatch match_row, match_value;
  std::string device_info(getInfo());
  if (std::regex_search(device_info, match_row, std::regex("Max stiffness: [[:digit:]]+"))) {
    std::string max_stiffness(match_row[0]);
    std::regex_search(max_stiffness, match_value, std::regex("[[:digit:]]+"));
    return std::stoi(match_value[0]);
  }
  return 3000;  // default limit of stiffness value
}

bool qbMoveHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh) {
  if (!qb_device_hardware_interface::qbDeviceHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // if the device interface initialization has succeed the device info have been retrieved
  std::static_pointer_cast<qb_move_transmission_interface::qbMoveTransmission>(transmission_.getTransmission())->setPositionFactor(device_.encoder_resolutions);
  std::static_pointer_cast<qb_move_transmission_interface::qbMoveTransmission>(transmission_.getTransmission())->setPresetFactor(getMaxStiffness());
  command_with_position_and_preset_ = std::static_pointer_cast<qb_move_transmission_interface::qbMoveTransmission>(transmission_.getTransmission())->getCommandWithPoistionAndPreset();
  return true;
}

void qbMoveHW::read(const ros::Time& time, const ros::Duration& period) {
  // read actuator state from the hardware (convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::read(time, period);
}

void qbMoveHW::updateShaftPositionLimits() {
  double preset_percent_to_radians(std::static_pointer_cast<qb_move_transmission_interface::qbMoveTransmission>(transmission_.getTransmission())->getPresetPercentToRadians());
  // the shaft limits [radians] depend on fixed motor limits [radians] and variable stiffness preset [0,1]
  joints_.limits.at(2).min_position = joints_.limits.at(0).min_position + std::abs(joints_.commands.at(3)*preset_percent_to_radians);
  joints_.limits.at(2).max_position = joints_.limits.at(0).max_position - std::abs(joints_.commands.at(3)*preset_percent_to_radians);
}

void qbMoveHW::write(const ros::Time& time, const ros::Duration& period) {
  // the variable stiffness decreases the shaft position limits (all the other limits are fixed)
  updateShaftPositionLimits();

  // send actuator command to the hardware (saturate and convert to proper measurement units)
  qb_device_hardware_interface::qbDeviceHW::write(time, period);
}

PLUGINLIB_EXPORT_CLASS(qb_move_hardware_interface::qbMoveHW, hardware_interface::RobotHW)