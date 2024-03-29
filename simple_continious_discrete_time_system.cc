
#include <iostream>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"


using namespace std;
using namespace drake;
using namespace systems;

//Update equation : x_(n+1)=f(n,x_n,u_n) in our case x_(n+1)=x_(n)+1
//Output_equation: y_n = g(n, x_n, u_n) in our case y_n = 10 * x_(n)
// Boundary Condition : x_0=x_init in our case x_0=0
class my_discrete_system : public LeafSystem<double>{
public:
my_discrete_system(){
DeclareDiscreteState(1);// Number of variables
DeclarePeriodicDiscreteUpdateEvent(0.25,0,&my_discrete_system::Update_function);//Update
DeclareVectorOutputPort("Sn",BasicVector<double>(1),&my_discrete_system::Output_function);//Final Output

}
private :
void Update_function (const Context<double>& context,DiscreteValues<double>*next_state) const {
const double x_n=context.get_discrete_state()[0];
(*next_state)[0]=x_n+1;
}
void Output_function(const Context<double>& context,BasicVector<double>* final_output) const {
const double x_n=context.get_discrete_state()[0];
const double S_n=10*x_n;
(*final_output)[0]=S_n;
}
};
// State equation xdot=-x^3+6x^2-5x-12
class my_continious_system : public LeafSystem<double>{
public:
my_continious_system(){
DeclareContinuousState(1,1,0);
DeclareVectorOutputPort("y",BasicVector<double>(2),&my_continious_system::Output_function);
}
private :

void DoCalcTimeDerivatives(const Context<double>& context,ContinuousState<double>* derivatives) const override {
  const double x = context.get_continuous_state()[0];
  const double xdot = -pow(x, 3)+6*pow(x,2)-5*x-12;
  (*derivatives)[0] = xdot;
}

void Output_function(const Context<double>& context,BasicVector<double>* final_output) const {
  const double x = context.get_continuous_state()[0];
  const double xdot = context.get_continuous_state()[1];
  (*final_output)[0] = x;
  (*final_output)[1] = xdot;
}
};

int main ()
{

//Get  the output
// Build a Diagram containing the Example system and a data logger that
// samples the Sn output port exactly at the update times.
DiagramBuilder<double> builder;
auto continuous_result = builder.AddSystem<my_continious_system>();
auto logger = LogOutput(continuous_result->GetOutputPort("y"), &builder);
logger->set_publish_period(0.25);
auto system_final_output = builder.Build();
//Number of seconds to simulate
Simulator<double> simulator(*system_final_output);
ContinuousState<double>& initial_value =simulator.get_mutable_context().get_mutable_continuous_state();
double x_int_value;
double xdot_int_value;
cout<<"==========Producing Continous Function Results ============="<<endl;
cout<<"Provide Initial Value for X : ";
cin>>x_int_value;
cout<<"Provide Initial Value for X DOT : ";
cin>>xdot_int_value;
cout<<endl;
initial_value[0]=x_int_value;
initial_value[1]=xdot_int_value;
simulator.AdvanceTo(10);
cout<<"Total Time Steps N in  Simulation : " <<logger->sample_times().size()<<endl;
// Print out the contents of the log.
for (int n = 0; n < logger->sample_times().size(); ++n) {
  const double current_time = logger->sample_times()[n];
  cout <<"Current Iteration N : "<< n ;
  cout<<" | Time : "<<current_time;
  cout<< " | Out Put X_t : " << logger->data()(0, n);
  cout<< " | Out Put X_dot : " << logger->data()(1, n)<<endl;
}

return 0;


}
