#ifndef GAZEBO_WORLD_HH
#define GAZEBO_WORLD_HH

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <array>

namespace gazebo {
class GazeboWorld : public WorldPlugin {
 public:
  GazeboWorld();
  ~GazeboWorld();
  void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  // gazebo system variables
  event::ConnectionPtr updateConnection_;
  // gazebo world management variables
  std::array<physics::ModelPtr, 4> block_;
};

// Register this plugin with simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboWorld)
}  // namespace gazebo

#endif  // !GAZEBO_WORLD_HH