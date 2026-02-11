#ifndef TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_
#define TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>

#include <gazebo/physics/physics.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <iostream>
#include <string>
#include <vector>

namespace gazebo
{

class TrafficLight : public ModelPlugin
{
public:
  TrafficLight();
  ~TrafficLight() override;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate();

  // Gazebo pointers
  physics::ModelPtr model;
  physics::WorldPtr world;
  event::ConnectionPtr update_connection;

  // Transport (for visual updates)
  gazebo::transport::NodePtr node;
  gazebo::transport::PublisherPtr visPub;
  gazebo::msgs::Visual msg;

  // Timing
  common::Time last_time;
  double traffic_cycle;

  // State
  int status;

  // Texture names
  std::vector<std::string> textures;
};

}  // namespace gazebo

#endif  // TURTLEBOT3_GAZEBO__TRAFFIC_LIGHT_PLUGIN_HPP_
