#include <iostream>
#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using namespace std;
using namespace drake;
using namespace systems;
using namespace multibody;
using namespace geometry;

int main ()
{
  cout<<"___________Simple Pendulum Simulation___________"<<endl;
   //1- Initialize Builder
   DiagramBuilder <double> system_builder;
   SceneGraph<double>& my_scene = *system_builder.AddSystem<SceneGraph>();
   my_scene.set_name("my_scene");
   //2- Initialize and load URDF model to multibody_plant
   string model_path=FindResourceOrThrow("drake/MAIN/pendulum.sdf");
   MultibodyPlant<double>& pendulum=*system_builder.AddSystem<MultibodyPlant>();
   Parser(&pendulum,&my_scene).AddModelFromFile(model_path);
   pendulum.Finalize();
  //Connect Pendulum model to drake visualizer
   system_builder.Connect(pendulum.get_geometry_poses_output_port(),my_scene.get_source_pose_port(pendulum.get_source_id().value()));
  ConnectDrakeVisualizer(&system_builder, my_scene);
  auto diagram = system_builder.Build();
// 3- Create a context for this system:
unique_ptr<Context<double>> diagram_context =diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& pendulum_context =diagram->GetMutableSubsystemContext(pendulum, diagram_context.get());
  // There is no input actuation in this example for the passive dynamics.
  pendulum_context.FixInputPort(pendulum.get_actuation_input_port().get_index(), Vector1d(0));
  // Get joints so that we can set initial conditions.
  const RevoluteJoint<double>& pendulum_ball = pendulum.GetJointByName<RevoluteJoint>("theta");
  // Set initial state.
  pendulum_ball.set_angle(&pendulum_context, 1.0);
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(50);

  return 0;
}
