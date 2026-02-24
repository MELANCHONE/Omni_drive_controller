// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "omni_drive_controller/odometry.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace omni_drive_controller
{
Odometry::Odometry()
: timestamp_(0.0),
  position_x_in_base_frame_(0.0),
  position_y_in_base_frame_(0.0),
  orientation_z_in_base_frame_(0.0),
  velocity_in_base_frame_linear_x(0.0),
  velocity_in_base_frame_linear_y(0.0),
  velocity_in_base_frame_angular_z(0.0),
  wheel_to_center_distance_(0.0),
  wheels_radius_(0.0)
{
}

void Odometry::init(
  const rclcpp::Time & time, std::array<double, PLANAR_POINT_DIM> base_frame_offset)
{
  // Reset timestamp:
  timestamp_ = time;

  // Base frame offset (wrt to center frame).
  base_frame_offset_[0] = base_frame_offset[0];
  base_frame_offset_[1] = base_frame_offset[1];
  base_frame_offset_[2] = base_frame_offset[2];
}

bool Odometry::update(
  const double wheel_front_left_vel, const double wheel_rear_left_vel,
  const double wheel_rear_right_vel, const double wheel_front_right_vel, const double dt)
{
  /// We cannot estimate the speed with very small time intervals:
  // const double dt = (time - timestamp_).toSec();
  if (dt < 0.0001) return false;  // Interval too small to integrate with

  /// Compute FK (i.e. compute mobile robot's body twist out of its wheels velocities):
  /// NOTE: the mecanum IK gives the body speed at the center frame, we then offset this velocity
  ///       at the base frame.
  /// NOTE: in the diff drive the velocity is filtered out, but we prefer to return it raw and
  ///       let the user perform post-processing at will.
  ///       We prefer this way of doing as filtering introduces delay (which makes it difficult
  ///       to interpret and compare behavior curves).

  /// \note The variables meaning:
  /// angular_transformation_from_center_2_base: Rotation transformation matrix, to transform
  /// from center frame to base frame
  /// linear_transformation_from_center_2_base: offset/linear transformation matrix,
  /// to transform from center frame to base frame

  double v_fl = wheel_front_left_vel;
  double v_fr = wheel_front_right_vel;
  double v_rr = wheel_rear_right_vel;  // 应该等于 v_fl
  double v_rl = wheel_rear_left_vel;
  

  
  // 正运动学
  const double sqrt2_2 = 0.7071067811865476;
  
  double velocity_in_center_frame_linear_x =
    wheels_radius_ * sqrt2_2 / 4.0 *
    (-v_fl + v_fr - v_rl + v_rr);
         
  double velocity_in_center_frame_linear_y =
    wheels_radius_ * sqrt2_2 / 4.0 *
    (v_fl + v_fr - v_rl - v_rr);
    
  double velocity_in_center_frame_angular_z =
    wheels_radius_ / (4.0 * wheel_to_center_distance_) *
    (v_fl + v_fr + v_rl + v_rr);

  tf2::Quaternion orientation_R_c_b;
  orientation_R_c_b.setRPY(0.0, 0.0, -base_frame_offset_[2]);

  tf2::Matrix3x3 angular_transformation_from_center_2_base = tf2::Matrix3x3((orientation_R_c_b));
  tf2::Vector3 velocity_in_center_frame_w_r_t_base_frame_ =
    angular_transformation_from_center_2_base *
    tf2::Vector3(velocity_in_center_frame_linear_x, velocity_in_center_frame_linear_y, 0.0);
  tf2::Vector3 linear_transformation_from_center_2_base =
    angular_transformation_from_center_2_base *
    tf2::Vector3(-base_frame_offset_[0], -base_frame_offset_[1], 0.0);

  velocity_in_base_frame_linear_x =
    velocity_in_center_frame_w_r_t_base_frame_.x() +
    linear_transformation_from_center_2_base.y() * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_linear_y =
    velocity_in_center_frame_w_r_t_base_frame_.y() -
    linear_transformation_from_center_2_base.x() * velocity_in_center_frame_angular_z;
  velocity_in_base_frame_angular_z = velocity_in_center_frame_angular_z;

  /// Integration.
  /// NOTE: the position is expressed in the odometry frame , unlike the twist which is
  ///       expressed in the body frame.
  orientation_z_in_base_frame_ += velocity_in_base_frame_angular_z * dt;

  tf2::Quaternion orientation_R_b_odom;
  orientation_R_b_odom.setRPY(0.0, 0.0, orientation_z_in_base_frame_);

  tf2::Matrix3x3 angular_transformation_from_base_2_odom = tf2::Matrix3x3((orientation_R_b_odom));
  tf2::Vector3 velocity_in_base_frame_w_r_t_odom_frame_ =
    angular_transformation_from_base_2_odom *
    tf2::Vector3(velocity_in_base_frame_linear_x, velocity_in_base_frame_linear_y, 0.0);

  position_x_in_base_frame_ += velocity_in_base_frame_w_r_t_odom_frame_.x() * dt;
  position_y_in_base_frame_ += velocity_in_base_frame_w_r_t_odom_frame_.y() * dt;

  return true;
}

void Odometry::setWheelsParams(
  const double wheel_to_center_distance, const double wheels_radius)
{
  wheel_to_center_distance_ = wheel_to_center_distance;
  wheels_radius_ = wheels_radius;
}

}  // namespace omni_drive_controller
