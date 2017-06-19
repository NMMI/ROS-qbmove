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

#ifndef QB_MOVE_CONTROL_H
#define QB_MOVE_CONTROL_H

// internal libraries
#include <qb_device_control/qb_device_control.h>
#include <qb_move_hardware_interface/qb_move_hardware_interface.h>

namespace qb_move_control {
/**
 * The qbrobotics \em qbmove Control interface implements the specific structures to control the \em qbmove device. It
 * exploits the features provided by the base device-independent control class and the specific hardware interface.
 * \sa qb_device_control::qbDeviceControl, qb_move_hardware_interface::qbMoveHW
 */
class qbMoveControl : public qb_device_control::qbDeviceControl {
 public:
  /**
   * Initialize the \p qb_device_control::qbDeviceControl with the specific \p qb_move_hardware_interface::qbMoveHW
   * hardware interface. It exploits all the features provided by the base class and nothing more.
   * \sa qb_device_control::qbDeviceControl(), qb_move_hardware_interface::qbMoveHW
   */
  qbMoveControl();

  /**
   * Do nothing.
   */
  virtual ~qbMoveControl();
};
}  // namespace qb_move_control

#endif // QB_MOVE_CONTROL_H