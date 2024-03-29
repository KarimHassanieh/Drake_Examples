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
#include "drake/lcm/drake_lcm.h"

using namespace std;
using namespace drake;
using namespace systems;
using namespace multibody;
using namespace geometry;

int main ()
{
  cout<<"___________Monoped Robot Simulation___________"<<endl;
   //1- Initialize Builder
   DiagramBuilder <double> builder;

   SceneGraph<double>& my_scene = *builder.AddSystem<SceneGraph>();
   my_scene.set_name("Scene");
   //2- Initialize and load URDF model to multibody_plant
   string model_path=FindResourceOrThrow("drake/MAIN/models/monoped.urdf");
   MultibodyPlant<double>& robot=*builder.AddSystem<MultibodyPlant>(0.01);
  robot.set_name("MonoPed Robot");
   Parser(&robot,&my_scene).AddModelFromFile(model_path);
   robot.Finalize();
  //Connect Robot model to drake visualizer
  builder.Connect(robot.get_geometry_poses_output_port(),my_scene.get_source_pose_port(robot.get_source_id().value()));
  ConnectDrakeVisualizer(&builder, my_scene);
  auto diagram = builder.Build();
// 3- Create a context for this system:
unique_ptr<Context<double>> diagram_context =diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& robot_context =diagram->GetMutableSubsystemContext(robot, diagram_context.get());
robot_context.FixInputPort(robot.get_actuation_input_port().get_index(),{0,0,0});
//robot_context.FixInputPort(0,{0.0});

  // Get joints so that we can set initial conditions.
const RevoluteJoint<double>& haa_joint = robot.GetJointByName<RevoluteJoint>("haa_joint");

haa_joint.set_angle(&robot_context, 1.0);
systems::Simulator<double> simulator(*diagram, move(diagram_context));
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(50);

  return 0;
}
