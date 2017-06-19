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

#ifndef QB_MOVE_HARDWARE_INTERFACE_H
#define QB_MOVE_HARDWARE_INTERFACE_H

// internal libraries
#include <qb_device_hardware_interface/qb_device_hardware_interface.h>
#include <qb_move_hardware_interface/qb_move_transmission_interface.h>

namespace qb_move_hardware_interface {
/**
 * The qbrobotics \em qbmove HardWare interface implements the specific structures to manage the communication with the
 * \em qbmove device. It exploits the features provided by the base device-independent hardware interface and the
 * specific transmission interface.
 * \sa qb_device_hardware_interface::qbDeviceHW, qb_move_transmission_interface::qbMoveTransmission
 */
class qbMoveHW : public qb_device_hardware_interface::qbDeviceHW {
 public:
  /**
   * Initialize the \p qb_device_hardware_interface::qbDeviceHW with the specific transmission interface and actuator
   * and joint names.
   * \sa qb_device_hardware_interface::qbDeviceHW(), qb_move_transmission_interface::qbMoveTransmission
   */
  qbMoveHW();

  /**
   * Do nothing.
   */
  virtual ~qbMoveHW();

  /**
   * Return the shaft position and stiffness preset joints whether \p command_with_position_and_preset_ is \p true; the
   * motors joints otherwise.
   * \return The vector of controller joint names.
   */
  std::vector<std::string> getJoints();

  /**
   * Call the base \p qb_device_hardware_interface::read().
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   * \sa qb_device_hardware_interface::read()
   */
  void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Update the shaft joint limits and call the base \p qb_device_hardware_interface::write(). The update is necessary
   * since an increase of the variable stiffness decreases the shaft position limits, and vice versa.
   * \param time The current time.
   * \param period The time passed since the last call to this method, i.e. the control period.
   * \sa qb_device_hardware_interface::write(), updateShaftPositionLimits()
   */
  void write(const ros::Time& time, const ros::Duration& period);

 private:
  bool command_with_position_and_preset_;
  double position_ticks_to_radians_;
  double preset_ticks_to_percent_;

  /**
   * Update the shaft joint limits since they depend on the fixed motors limits and on the variable stiffness preset.
   * What comes out is that an increase of the variable stiffness decreases the absolute value of the shaft position
   * limit, follwing the formula \f$JL = \pm\mathopen|JL_{MAX} - S\mathclose|\f$, where \p S is the stifness preset
   * (all the values are in \em ticks).
   * \sa write()
   */
  void updateShaftPositionLimits();
};
typedef std::shared_ptr<qbMoveHW> qbMoveHWPtr;
}  // namespace qb_move_hardware_interface

#endif // QB_MOVE_HARDWARE_INTERFACE_H