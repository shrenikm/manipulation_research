#include <drake/geometry/drake_visualizer.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include <iostream>

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of the urdf to load.");
DEFINE_double(target_realtime_rate, 1.0, "Simulation playback speed.");
DEFINE_double(sim_dt, 1e-4, "Time step for the MultibodyPlant model.");
DEFINE_double(actuation, 0., "Actuation value for every join.");

namespace manr {
namespace xarm {
namespace simulation {

using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

int DoMain() {

  std::cout << "Trying to simulate!" << std::endl;

  DiagramBuilder<double> builder;

  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_sim_dt);

  const std::string urdf = "/home/shrenikm/Projects/manipulation_research/src/xarm_description/urdf/lite6.urdf";
  auto xarm_instance = Parser(&plant, &scene_graph).AddModels(urdf).at(0);
  // Can't weld, already welded in the urdf.
  //plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("link_base"));
  plant.Finalize();

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, lcm);

  const int num_joints = plant.num_positions();
  std::cout << "Num joints: " << num_joints << std::endl;

  auto zero_actuation = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(Eigen::VectorXd::Constant(num_joints, FLAGS_actuation));

  // Connect zero actuation.
  builder.Connect(zero_actuation->get_output_port(), plant.get_actuation_input_port());

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_simulation_sec);

  return 0;
}

} // namespace simulation
} // namespace xarm
} // namespace manr

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return manr::xarm::simulation::DoMain();
}
