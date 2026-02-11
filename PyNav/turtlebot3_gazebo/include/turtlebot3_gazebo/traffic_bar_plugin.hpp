#ifndef TURTLEBOT3_GAZEBO__TRAFFIC_BAR_PLUGIN_HPP_
#define TURTLEBOT3_GAZEBO__TRAFFIC_BAR_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <iostream>

namespace gazebo
{

class TrafficBar : public ModelPlugin
{
public:
  TrafficBar();

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate();

  // Gazebo pointers
  physics::ModelPtr model;
  physics::WorldPtr world;
  event::ConnectionPtr update_connection;

  // Timing
  common::Time last_time;
  double traffic_cycle;

  // State
  int status;

  // Poses
  ignition::math::Pose3d down_pose;
  ignition::math::Pose3d up_pose;
};

}  // namespace gazebo

#endif  // TURTLEBOT3_GAZEBO__TRAFFIC_BAR_PLUGIN_HPP_

