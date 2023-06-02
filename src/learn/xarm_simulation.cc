#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>
#include <iostream>

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of the urdf to load.");
DEFINE_double(target_realtime_rate, 1.0, "Simulation playback speed.");
DEFINE_double(sim_dt, 3e-3, "Time step for the MultibodyPlant model.");

namespace manr {
namespace xarm {
namespace simulation {

using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;

int DoMain() {

  std::cout << "Trying to simulate!" << std::endl;

  DiagramBuilder<double> builder;

  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);

  const std::string urdf = "/home/shrenikm/Projects/manipulation_research/src/xarm_description/urdf/lite6/lite6.urdf.xacro";
  auto xarm_instance = Parser(&plant, &scene_graph).AddModels(urdf).at(0);
  //plant.WeldFrames(plant.world_frame(), plant.
  plant.Finalize();


  return 0;
}

} // namespace simulation
} // namespace xarm
} // namespace manr

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return manr::xarm::simulation::DoMain();
}
