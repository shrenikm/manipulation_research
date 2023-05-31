#include <drake/systems/framework/diagram_builder.h>
#include <gflags/gflags.h>

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of the urdf to load.");
DEFINE_double(target_realtime_rate, 1.0, "Simulation playback speed.");
DEFINE_double(sim_dt, 3e-3, "Time step for the MultibodyPlant model.");


namespace manr {
namespace xarm {
namespace simulation {

using drake::systems::DiagramBuilder;


int DoMain() {

  DiagramBuilder<double> builder;

  return 0;

}

} // namespace simulation
} // namespace xarm
} // namespace manr

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return manr::xarm::simulation::DoMain();
}
