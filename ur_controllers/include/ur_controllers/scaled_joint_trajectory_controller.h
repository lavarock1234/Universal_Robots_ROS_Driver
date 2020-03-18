// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-18
 *
 */
//----------------------------------------------------------------------
#ifndef UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED
#define UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED

#include "ur_controllers/hardware_interface_adapter.h"
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/pos_vel_acc_state.h>

namespace ur_controllers {

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

double interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel) {
  using std::pow;
  double a = p0_pos;
  double b = p0_vel;
  double c = (-3 * a + 3 * p1_pos - 2 * T * b - T * p1_vel) / pow(T, 2);
  double d = (2 * a - 2 * p1_pos + T * b + T * p1_vel) / pow(T, 3);
  return a + b * t + c * pow(t, 2) + d * pow(t, 3);
}
template<class T>
inline std::string ToString(const T& obj) {
    std::stringstream ss;
    ss << obj;
    return ss.str();
}
template<class T>
inline std::string ToString(const T& obj, unsigned precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << obj;
    return ss.str();
}
template<class T>
inline std::string ToString(const std::vector<T>& vec, unsigned precision = 6, double multiplier = 1.0, char delimiter = ',', std::string braces = "[]") {
    if(braces.size() != 2) {
        braces = "[]";
    }

    if(vec.empty()) {
        return std::string("[]");
    }
    std::stringstream ss;
    ss << braces[0];
    for(unsigned i = 0; i < vec.size() - 1; i++) {
        ss << ToString(multiplier * vec[i], precision) << delimiter;
    }
    ss << ToString(multiplier * vec[vec.size()-1], precision) << braces[1];
    return ss.str();
}

template <class SegmentImpl, class HardwareInterface>
class ScaledJointTrajectoryController
  : public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface> {
public:
  using Scalar = typename joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>::Segment::Scalar;
  using Time = typename joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>::Segment::Time;
  using Base = joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>;
  using State = typename joint_trajectory_controller::JointTrajectoryController<SegmentImpl, HardwareInterface>::Segment::State;
  ScaledJointTrajectoryController() = default;
  virtual ~ScaledJointTrajectoryController() = default;

  struct TrajectoryPoint {
    std::vector<Scalar> positions;
    std::vector<Scalar> velocities;
    Time time_from_start;
    TrajectoryPoint() = default;
    TrajectoryPoint(size_t size) {
      positions.resize(size, static_cast<Scalar>(0));
      velocities.resize(size, static_cast<Scalar>(0));
    }
  };

  size_t getTrajectorySize(const typename Base::Trajectory &traj) {
    if(traj.empty()) {
      return 0;
    }
    return traj[0].size();
  }

  TrajectoryPoint getStartTrajectoryPoint(const typename Base::Trajectory& traj, unsigned idx) {
    TrajectoryPoint point(traj.size());
    for (unsigned i = 0; i < traj.size(); i++) {
      point.positions[i] = traj[i][idx].startState().position[0];
      point.velocities[i] = traj[i][idx].startState().velocity[0];
      point.time_from_start = traj[i][idx].startTime();
    }
    return point;
  }

  TrajectoryPoint getEndTrajectoryPoint(const typename Base::Trajectory& traj, unsigned idx) {
    TrajectoryPoint point(traj.size());
    for (unsigned i = 0; i < traj.size(); i++) {
      point.positions[i] = traj[i][idx].endState().position[0];
      point.velocities[i] = traj[i][idx].endState().velocity[0];
      point.time_from_start = traj[i][idx].endTime();
    }
    return point;
  }

  std::string printTrajectory(const typename Base::Trajectory& traj) {
    std::stringstream ss;
    for (unsigned i = 0; i < getTrajectorySize(traj); i++) {
      auto point = getEndTrajectoryPoint(traj, i);
      ss << "idx: " << i << ", position: " << ToString(point.positions, 2, RAD_TO_DEG)
        << ", velocity: " << ToString(point.velocities, 2, RAD_TO_DEG) << ", tfs: " << point.time_from_start * 1e3 << " [ms]\n";
    }
    return ss.str();
  }

  void update(const ros::Time& time, const ros::Duration& period) {
    this->scaling_factor_ = this->joints_[0].getScalingFactor();
    // Get currently followed trajectory
    typename Base::TrajectoryPtr curr_traj_ptr;
    this->curr_trajectory_box_.get(curr_traj_ptr);
    typename Base::Trajectory& curr_traj = *curr_traj_ptr;

    // Update time data
    typename Base::TimeData time_data;
    time_data.time = time;                                                        // Cache current time
    time_data.period = ros::Duration(this->scaling_factor_ * period.toSec());     // Cache current control period
    time_data.uptime = this->time_data_.readFromRT()->uptime + time_data.period;  // Update controller uptime
    ros::Time traj_time = this->time_data_.readFromRT()->uptime + period;
    this->time_data_.writeFromNonRT(time_data);  // TODO: Grrr, we need a lock-free data structure here!

    // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
    // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
    // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
    // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
    // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time
    // we fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts
    // in the next control cycle, leaving the current cycle without a valid trajectory.

    if (!this->rt_active_goal_) {
      return;
    }

    // Update trajectory following info
    static typename Base::RealtimeGoalHandlePtr prev_goal;
    if (prev_goal != this->rt_active_goal_) {
      // Trajectory sanity checks
      if (getTrajectorySize(curr_traj) <= 1) {
        ROS_ERROR_NAMED(this->name_, "Unexpected error: Trajectory must be have more than one point!");
        return;
      }
      if (curr_traj.size() != this->joints_.size()) {
        ROS_ERROR_NAMED(this->name_, "Unexpected error: Trajectory size (%d) does not match joint size (%d)!",
                        (int) curr_traj.size(), (int) this->joints_.size());
        return;
      }
      current_idx = 1; // skipping first point
      point = getEndTrajectoryPoint(curr_traj, 1);
      prev = getEndTrajectoryPoint(curr_traj, 0);
      t0 = this->time_data_.readFromRT()->time;
      tfs0 = 0;
      prev_goal = this->rt_active_goal_;
      ROS_INFO("Received new goal with t0 %f", t0.toSec());
    }

    // Update times
    auto latest = this->time_data_.readFromRT()->time;
    auto elapsed = (latest - t0).toSec();

    // Check if segment or trajectory is done
    while ((point.time_from_start - tfs0) <= elapsed) {
      current_idx++;
      if (current_idx >= getTrajectorySize(curr_traj)) {
        ROS_INFO("TRAJECTORY DONE!");
        this->rt_active_goal_->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        this->rt_active_goal_->setSucceeded(this->rt_active_goal_->preallocated_result_);
        this->rt_active_goal_.reset();
        this->successful_joint_traj_.reset();
        return;
      }
      prev = point;
      point = getEndTrajectoryPoint(curr_traj, current_idx);
    }

    // Sample on segment
    auto segment_s = elapsed - (prev.time_from_start - tfs0);
    auto d_s = point.time_from_start - prev.time_from_start;
    for (unsigned int i = 0; i < this->joints_.size(); ++i) {
      this->desired_joint_state_.position[0] =
        interpolate(segment_s, d_s, prev.positions[i], point.positions[i], prev.velocities[i], point.velocities[i]);
      this->desired_joint_state_.velocity[0] = 0;
      this->desired_joint_state_.acceleration[0] = 0;

      this->desired_state_.position[i] = this->desired_joint_state_.position[0];
      this->desired_state_.velocity[i] = this->desired_joint_state_.velocity[0];
      this->desired_state_.acceleration[i] = this->desired_joint_state_.acceleration[0];

      this->state_joint_error_.position[0] = angles::shortest_angular_distance(this->current_state_.position[i], this->desired_joint_state_.position[0]);
      this->state_joint_error_.velocity[0] = this->desired_joint_state_.velocity[0] - this->current_state_.velocity[i];
      this->state_joint_error_.acceleration[0] = 0.0;

      this->state_error_.position[i] = angles::shortest_angular_distance(this->current_state_.position[i], this->desired_joint_state_.position[0]);
      this->state_error_.velocity[i] = this->desired_joint_state_.velocity[0] - this->current_state_.velocity[i];
      this->state_error_.acceleration[i] = 0.0;
    }

    // Hardware interface adapter: Generate and send commands
    // Send just the position for this controller
    ROS_INFO("SENDING COMMAND: %s, elapsed(%f), next(%f)", ToString(this->desired_state_.position, 2, RAD_TO_DEG).c_str(), elapsed, point.time_from_start - tfs0);
    this->hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period, this->desired_state_, this->state_error_);
//    ROS_INFO("\n%s", printTrajectory(curr_traj).c_str());
//    ROS_INFO("IDX: %d (%s) at %f [ms]: Error Pos: %s", current_idx, ToString(current_tracked_point, 2, RAD_TO_DEG).c_str(),
//      traj_time.toSec() * 1e3, ToString(this->state_error_.position, 2, RAD_TO_DEG).c_str());

    // Set action feedback
    if (this->rt_active_goal_) {
      this->rt_active_goal_->preallocated_feedback_->header.stamp = this->time_data_.readFromRT()->time;
      this->rt_active_goal_->preallocated_feedback_->header.frame_id = std::to_string(current_idx);
      this->rt_active_goal_->preallocated_feedback_->desired.positions = this->desired_state_.position;
      this->rt_active_goal_->preallocated_feedback_->desired.velocities = this->desired_state_.velocity;
      this->rt_active_goal_->preallocated_feedback_->desired.accelerations = this->desired_state_.acceleration;
      this->rt_active_goal_->preallocated_feedback_->actual.positions = this->current_state_.position;
      this->rt_active_goal_->preallocated_feedback_->actual.velocities = this->current_state_.velocity;
      this->rt_active_goal_->preallocated_feedback_->error.positions = this->state_error_.position;
      this->rt_active_goal_->preallocated_feedback_->error.velocities = this->state_error_.velocity;
      this->rt_active_goal_->setFeedback(this->rt_active_goal_->preallocated_feedback_);
    }

    // Publish state (feedback)
    this->publishState(time_data.uptime);
  }

protected:
  double scaling_factor_;

private:
  ///< Tracking information (adapting ur_modern_driver variable names)
  TrajectoryPoint point; ///< Current tracked point
  TrajectoryPoint prev; ///< Previous tracked point
  ros::Time t0; ///< Time when trajectory tracking started
  double tfs0; ///< Time From Start for first point
  int current_idx = -1; ///< Index of trajectory being tracked right now
};
}  // namespace ur_controllers

#endif  // ifndef UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED
