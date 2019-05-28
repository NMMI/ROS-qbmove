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

#ifndef QB_MOVE_INTERACTIVE_INTERFACE_H
#define QB_MOVE_INTERACTIVE_INTERFACE_H

// ROS libraries
#include <interactive_markers/interactive_marker_server.h>
#include <tf2/utils.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace qb_move_interactive_interface {
/**
 * A simple interface that adds an interactive marker (to be used in \p rviz) with two controls to change respectively
 * the shaft position and the stiffness of a \em qbmove device.
 * The core features are called in the server callback where feedback from the marker are handled.
 * \sa interactiveMarkerCallback()
 */
class qbMoveInteractive {
 public:
  /**
   * Start the async spinner and do nothing else. The real initialization is done later by \p initMarkers().
   * \sa initMarkers()
   */
  qbMoveInteractive()
      : spinner_(1) {
    spinner_.start();
  }

  /**
   * Stop the async spinner.
   */
  virtual ~qbMoveInteractive() {
    spinner_.stop();
  }

  /**
   * Initialize the \p interactive_markers::InteractiveMarkerServer and the marker displayed in \p rviz to control the
   * shaft position of the \em qbmove device and its stiffness. This method is called by the \p qbMoveHW during its
   * initialization.
   * \param root_nh A NodeHandle in the root of the caller namespace (depends from the one in the hardware interface).
   * \param device_name The unique device name.
   * \param joint_names The vector of the actuated joint names.
   * \sa buildCylinder(), buildMotorControl()
   */
  void initMarkers(ros::NodeHandle &root_nh, const std::string &device_name, const std::vector<std::string> &joint_names) {
    device_name_ = device_name;
    joint_names_ = joint_names;
    interactive_commands_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(device_name + "_interactive_commands");

    cylinder_.header.frame_id = device_name + "_shaft_link";
    cylinder_.name = device_name + "_position_reference_cylinder";
    cylinder_.description = "motor position reference cylinder.";
    cylinder_.scale = 0.05;
    controls_.pose.position.y = 0.02;  // others are 0
    buildCylinder(cylinder_);

    controls_.header.frame_id = device_name + "_shaft_link";
    controls_.name = device_name + "_position_reference_controls";
    controls_.description = "motor position reference controls.";
    controls_.scale = 0.05;
    controls_.pose.position.y = 0.02;  // others are 0
    buildMotorControl(controls_);

    interactive_commands_server_->insert(cylinder_);
    interactive_commands_server_->insert(controls_, std::bind(&qbMoveInteractive::interactiveMarkerCallback, this, std::placeholders::_1));
    interactive_commands_server_->applyChanges();

    // set initial pose to avoid NaN in computation
    controls_position_orig_.orientation.w = 1;  // others are 0
    controls_position_orig_.position.y = 0.02;  // others are 0
    controls_position_old_ = controls_position_orig_;

    joint_command_pub_ = root_nh.advertise<trajectory_msgs::JointTrajectory>("control/" + device_name + "_position_and_preset_trajectory_controller/command", 1);
  }

  /**
   * Update the stored joint positions with the given values. It is aimed to be called when reading HW.
   * \param joint_positions The vector of the actuated joint positions, basically [shaft_position, stiffness].
   */
  void setMarkerState(const std::vector<double> &joint_positions) {
    joint_positions_ = joint_positions;
    setScaleFromStiffness(joint_positions_.at(1));
  }

 protected:
  ros::AsyncSpinner spinner_;
  ros::Publisher joint_command_pub_;
  trajectory_msgs::JointTrajectory joint_command_trajectories_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::string device_name_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_commands_server_;
  visualization_msgs::InteractiveMarker cylinder_;
  visualization_msgs::InteractiveMarker controls_;
  geometry_msgs::Pose controls_position_old_;
  geometry_msgs::Pose controls_position_orig_;

  /**
   * Create a cylinder marker representing the qbmove shaft axis and attach it to the given interactive marker.
   * \param interactive_marker The Interactive Marker structure to be filled.
   */
  void buildCylinder(visualization_msgs::InteractiveMarker &interactive_marker) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.y = 0.02;  // others are 0
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 1;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.scale.x = interactive_marker.scale * 0.4;
    marker.scale.y = interactive_marker.scale * 0.4;
    marker.scale.z = interactive_marker.scale * 0.4;
    marker.color.r = 0.35;
    marker.color.g = 0.35;
    marker.color.b = 0.35;
    marker.color.a = 0.75;

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = static_cast<unsigned char>(true);
    control.markers.push_back(marker);
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    interactive_marker.controls.push_back(control);
  }

  /**
   * Add two interactive controls to the given interactive marker. The first let to rotate the shaft, while the second
   * allow to change the device stiffness. The change in shaft position is clear from the orientation of the flange
   * attached to the qbmove; on the other hand a change in stiffness is represented graphically with a bigger or
   * smaller cylinder, respectively for more and less stiffness.
   * \param interactive_marker The Interactive Marker structure to be filled.
   */
  void buildMotorControl(visualization_msgs::InteractiveMarker &interactive_marker) {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "position";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    interactive_marker.controls.push_back(control);
    control.name = "stiffness";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    interactive_marker.controls.push_back(control);
  }

  /**
   * Compute the angular distance between current and last orientations.
   * \param to_pose The current orientation of the marker.
   * \param from_pose The last known orientation of the marker.
   * \return The computed angular distance.
   */
  double computeAngularDistance(const geometry_msgs::Quaternion &to_pose, const geometry_msgs::Quaternion &from_pose) {
    tf2::Quaternion to_q, from_q;
    tf2::fromMsg(to_pose, to_q);
//    tf2::fromMsg(from_pose, from_q);
    double to_roll = 0.0;
//    double from_roll = 0.0;
    double not_used_1, not_used_2;
    tf2::Matrix3x3(to_q).getRPY(not_used_1, to_roll, not_used_2);
//    tf2::Matrix3x3(from_q).getRPY(not_used_1, from_roll, not_used_2);
    return to_roll;  // actually this works better than the difference in orientation
  }

  /**
   * Compute the distance along the marker axis between current and last positions.
   * \param to_pose The current position of the marker.
   * \param from_pose The last known position of the marker.
   * \return The computed distance.
   */
  double computeLinearDistance(const geometry_msgs::Point &to_pose, const geometry_msgs::Point &from_pose) {
    return to_pose.y - from_pose.y;  // the marker can only move in the y direction
  }

  /**
   * Compute a simple joint trajectory from the given displacements in position and orientation.
   * \param displacement_position The displacement in angular position of the shaft, expressed in radians.
   * \param displacement_stiffness The displacement in stiffness of the device, expressed in [0,1];
   * \return The joint trajectory filled with the proper commands.
   */
  trajectory_msgs::JointTrajectory computeJointTrajectories(const double &displacement_position, const double &displacement_stiffness) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {joint_positions_.at(0) + displacement_position, getStiffnessFromScale()};
    point.velocities.resize(2);
    point.accelerations.resize(2);
    point.time_from_start = ros::Duration(1.0);  //TODO: add a parameter to set the trajectory velocity

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = device_name_ + "_interactive";
    trajectory.joint_names.push_back(joint_names_.at(0));
    trajectory.joint_names.push_back(joint_names_.at(1));
    trajectory.points.push_back(point);
    return trajectory;
  }

  /**
   * \return The stiffness value computed by the diameter scale of the marker.
   * \sa setScaleFromStiffness()
   */
  double getStiffnessFromScale() {
    return cylinder_.controls.at(0).markers.at(0).scale.x/0.05 - 0.2;
  }

  /**
   * This is the core method of the class, where commands are computed w.r.t. the previous state of the interactive
   * marker and the computed joint trajectory is sent to the low level controller, e.g. the joint trajectory controller
   * of the \p qbMoveHW.
   * \param feedback The feedback state of the interactive marker, provided by rviz.
   */
  void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == feedback->POSE_UPDATE) {
      // auto clamp = [](auto &val, const auto &min, const auto &max) { val = std::min(std::max(val, min), max); };
      double clamped_scale = std::min(std::max(cylinder_.controls.at(0).markers.at(0).scale.x + 2*(feedback->pose.position.y - controls_position_old_.position.y), 0.01), 0.05);
      cylinder_.controls.at(0).markers.at(0).scale.x = clamped_scale;
      cylinder_.controls.at(0).markers.at(0).scale.y = clamped_scale;
      interactive_commands_server_->insert(cylinder_);
      interactive_commands_server_->applyChanges();

      double displacement_position = computeAngularDistance(feedback->pose.orientation, controls_position_old_.orientation);
      //TODO: find a better workaround to avoid discontinuities
      if (std::abs(displacement_position) > 1.5) {
        interactive_commands_server_->erase(controls_.name);
        interactive_commands_server_->applyChanges();
        interactive_commands_server_->insert(controls_, std::bind(&qbMoveInteractive::interactiveMarkerCallback, this, std::placeholders::_1));
        interactive_commands_server_->applyChanges();
      }

      double displacement_stiffness = computeLinearDistance(feedback->pose.position, controls_position_old_.position);
      joint_command_trajectories_ = computeJointTrajectories(displacement_position, displacement_stiffness);
      joint_command_pub_.publish(joint_command_trajectories_);

      controls_position_old_ = feedback->pose;
      return;
    }

    if (feedback->event_type == feedback->MOUSE_UP) {
      interactive_commands_server_->setPose(controls_.name, controls_position_orig_);
      interactive_commands_server_->applyChanges();
      controls_position_old_ = controls_position_orig_;
      return;
    }
  }

  /**
   * Set the diameter scale of the marker from the given stiffness value.
   * \param stiffness_value The \em qbmove stiffness value.
   * \sa getStiffnessFromScale()
   */
  void setScaleFromStiffness(const double &stiffness_value) {
    double scaled_stiffness_value = 0.01 + stiffness_value*0.05;
    cylinder_.controls.at(0).markers.at(0).scale.x = scaled_stiffness_value;
    cylinder_.controls.at(0).markers.at(0).scale.y = scaled_stiffness_value;
    interactive_commands_server_->insert(cylinder_);
    interactive_commands_server_->applyChanges();
  }
};
}  // namespace qb_move_interactive_interface

#endif // QB_MOVE_INTERACTIVE_INTERFACE_H