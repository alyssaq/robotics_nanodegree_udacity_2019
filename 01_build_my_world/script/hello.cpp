#include <gazebo/gazebo.hh>

// export GAZEBO_PLUGIN_PATH=/Users/alyssa/Documents/stuff/robotics_nanodegree_udacity_2019/01_build_my_world/build
namespace gazebo {
  class WorldPluginMyRobot : public WorldPlugin {
    public: WorldPluginMyRobot() : WorldPlugin() {
      printf("Welcome to Alyssa's World! \n");
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}
