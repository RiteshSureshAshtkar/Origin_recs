#include "turtlebot3_gazebo/obstacles.hpp"

#include <gazebo/common/Animation.hh>
#include <gazebo/common/KeyFrame.hh>
#include <cmath>

namespace gazebo
{

static constexpr double PI = M_PI;

void Obstacles::Load(physics::ModelPtr _parent, sdf::ElementPtr)
{
  this->model = _parent;

  gazebo::common::PoseAnimationPtr anim(
    new gazebo::common::PoseAnimation("move", 40.0, true));

  gazebo::common::PoseKeyFrame * key;

  key = anim->CreateKeyFrame(0);
  key->Translation(ignition::math::Vector3d(0.0, 0.0, 0.0));
  key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  key = anim->CreateKeyFrame(20);
  key->Rotation(ignition::math::Quaterniond(0, 0, PI));

  key = anim->CreateKeyFrame(40);
  key->Rotation(ignition::math::Quaterniond(0, 0, 2 * PI));

  _parent->SetAnimation(anim);
}

}  // namespace gazebo

