#include <iostream>
#include <fstream>
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
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/signal_logger.h"
#include<vector>

using namespace std;
using namespace drake;
using namespace systems;
using namespace multibody;
using namespace geometry;

// A model of simple pendulum
//@f[ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = \tau @f]
/// @system{PendulumPlant,
///    @input_port{tau},
///    @output_port{state(which are theta thetadot )}
/// }
class pendulum_system final : public LeafSystem<double>{
public:
pendulum_system(){
DeclareContinuousState(1,1,0);
DeclareInputPort ("control_tau",kVectorValued,1);
DeclareInputPort ("damping_factor",kVectorValued,1);
DeclareInputPort ("length",kVectorValued,1);
DeclareInputPort ("mass",kVectorValued,1);
DeclareInputPort ("gravity",kVectorValued,1);
DeclareVectorOutputPort("pendulum_state",BasicVector<double>(2),&pendulum_system::Output_state);//STATE ={Theta,Tehta_dot}
}
private :

void DoCalcTimeDerivatives(const Context<double>& context,ContinuousState<double>* derivatives) const override {
// Equation  ω̇ = (τ - bω)/(mℓ²) - g/ℓ⋅sin(θ)
//Time derivates of Theta is theta dot

double theta=context.get_continuous_state()[0];
double theta_dot=context.get_continuous_state()[1];

double tau=GetInputPort("control_tau").Eval(context)[0];
(*derivatives)[0]=theta_dot;
double b=GetInputPort("damping_factor").Eval(context)[0];
double m=GetInputPort("mass").Eval(context)[0];
double l=GetInputPort("length").Eval(context)[0];
double g=GetInputPort("gravity").Eval(context)[0];
double theta_double_dot=(tau-b*theta_dot)/(m*l*l)-(g/l)*sin(theta);

(*derivatives)[1]=theta_double_dot;

}

void Output_state(const Context<double>& context,BasicVector<double>* state_output) const {
(*state_output)[0]=context.get_continuous_state()[0];//THETA
(*state_output)[1]=context.get_continuous_state()[1];//THETA DOT
}
};

int main ()
{
  cout<<"___________Simple Pendulum Simulation___________"<<endl;
  /// A model of a simple pendulum
  // Note: for the pendulum, forward dynamics is:
  // ω̇ = (τ - bω)/(mℓ²) - g/ℓ⋅sin(θ), with:
  // ℓ: length, in m.
  // m: mass, in kg.
  // b: dissipation, in N⋅m⋅s.
  // τ: input torque, in N⋅m.
  // g: acceleration of gravity, in m/s²
  //Get  the output
double control_input;
double damping;
double mass;
double length;
double gravity;
double initial_theta;
double initial_theta_dot;
cout<<"____________Pendulum Torque Input_______"<<endl;
cout<<"Insert Control Input Tau Required : ";
cin>>control_input;
cout<<"___________Pendulum Parameters_________"<<endl;
cout<<"Insert Damping Factor Required : ";
cin>>damping;
cout<<"Insert Mass  Required : ";
cin>>mass;
cout<<"Insert Length  Required : ";
cin>>length;
cout<<"Insert Gravity Required : ";
cin>>gravity;
cout<<"_________Initial Conditions________ "<<endl;
cout<<"Insert Initial Angle  : ";
cin>>initial_theta;
cout<<"Insert Initial Angular Velocity  : ";
cin>>initial_theta_dot;
cout<<"___________Simulation Results_______"<<endl;





  DiagramBuilder<double> builder;
  auto pendulum_result = builder.AddSystem<pendulum_system>();
  builder.ExportInput(pendulum_result->get_input_port(0), "control_tau");
   builder.ExportInput(pendulum_result->get_input_port(1), "damping_factor");
  builder.ExportInput(pendulum_result->get_input_port(2), "length");
builder.ExportInput(pendulum_result->get_input_port(3), "mass");
builder.ExportInput(pendulum_result->get_input_port(4), "gravity");
 


auto logger = LogOutput(pendulum_result->GetOutputPort("pendulum_state"), &builder);
 logger->set_publish_period(0.25);
  auto system_final_output = builder.Build();
  //Number of seconds to simulate
  Simulator<double> simulator(*system_final_output);
  ContinuousState<double>& initial_state =simulator.get_mutable_context().get_mutable_continuous_state();

  //Initial condition for theta
  initial_state[0]=initial_theta;
  //Initial condition for theta dot
  initial_state[1]=initial_theta_dot;

//Place input torque

Context<double>& pendulum_context =simulator.get_mutable_context();
pendulum_context.FixInputPort(0,{control_input});
pendulum_context.FixInputPort(1,{damping});
pendulum_context.FixInputPort(2,{length});
pendulum_context.FixInputPort(3,{mass});
pendulum_context.FixInputPort(4,{gravity});


simulator.AdvanceTo(30);
  cout<<"Total Time Steps N in  Simulation : " <<logger->sample_times().size()<<endl;
  // Print out the contents of the log.
  // Open file to print results on
  ofstream pendulum_file;
  pendulum_file.open ("/home/karim/Desktop/KARIM/ROBOTICS/drake_karim/MAIN/Simulation_result.txt");
 for (int n = 0; n < logger->sample_times().size(); ++n) {
    const double current_time = logger->sample_times()[n];
    cout <<"Current Iteration N : "<< n ;
    cout<<" | Time : "<<current_time;
    pendulum_file<<current_time<<",";
    cout<< " | Theta  : " << logger->data()(0, n);
    pendulum_file<<logger->data()(0,n)<<",";
    cout<< " |  Theta Dot : " << logger->data()(1, n)<<endl;
    pendulum_file<<logger->data()(1,n)<<endl;
  }
  pendulum_file.close();
  return 0;
}
