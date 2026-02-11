#ifndef TURTLEBOT3_GAZEBO__OBSTACLES_HPP_
#define TURTLEBOT3_GAZEBO__OBSTACLES_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>


#include <gazebo/physics/physics.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{

class Obstacles : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:
  physics::ModelPtr model;
};

}  // namespace gazebo

#endif  // TURTLEBOT3_GAZEBO__OBSTACLES_HPP_
