#include <ros/console.h>
#include <boost/bind.hpp>
#include <cstdlib>
#include <ctime>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <metareasoning_agent/gazebo_world.hh>

namespace gazebo {
// typedefs and global vars (if Reqd)
GazeboWorld::GazeboWorld() {
  // task management variables

  // episode management variables

  // instance management variables

  // logging options

  // bind variables to methods (PA/Method)

  // initialize agent/planner

  // random seed
  srand(static_cast<unsigned>(time(0)));
}

void GazeboWorld::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
  // store pointers to objects in the environment
  block_[0] = _parent->GetModel("block0");
  block_[1] = _parent->GetModel("block1");
  block_[2] = _parent->GetModel("block2");
  block_[3] = _parent->GetModel("block3");

  // place objects in the right place

  // bind onUpdate function with Gazebo's cylic update event
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWorld::OnUpdate, this, _1));

  ROS_INFO("If you see this, we are in business");
}

void GazeboWorld::OnUpdate(const common::UpdateInfo& /*_info*/) {
  ROS_INFO("The OnUpdate function works");
}
}  // namespace gazebo