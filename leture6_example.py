import matplotlib.pyplot as plt
from pydrake.all import (BasicVector, LeafSystem, DiagramBuilder, Simulator, LogOutput, ConstantVectorSource,
                         LinearSystem)
import numpy as np
import scipy.linalg as la

# Define the system.
# Its father is LeafSystem
class my_DT_syst(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        num_DTState = 1
        num_input = 1
        num_output = 2
        # Define the state vector
        self.DeclareDiscreteState(num_DTState)
        # Define the input
        self.DeclareVectorInputPort("u", BasicVector(num_input))
        # Define the output
        self.DeclareVectorOutputPort("y", BasicVector(num_output), self.CalcOutputY)
        self.DeclarePeriodicDiscreteUpdate(0.01)  # One second timestep.

    def DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        x = context.get_discrete_state_vector().CopyToVector()
        u = self.get_input_port(0).Eval(context)
        xnext = 0.98 * x + 5 * u
        discrete_state.get_mutable_vector().SetFromVector(xnext)

    def CalcOutputY(self, context, output):
        x = context.get_discrete_state_vector().CopyToVector()
        u = self.get_input_port(0).Eval(context)
        y = np.array([x, u])
        output.SetFromVector(y)


def test_DT_syst():
    # testing
    # Create a simple block diagram containing our system.
    builder = DiagramBuilder()
    # add a system block which is a "class" and named it as mySys_Yuntian
    mySys_Yuntian = builder.AddSystem(my_DT_syst())
    # add a step generator block and named it as stepInput
    stepInput = builder.AddSystem(ConstantVectorSource([2]))  # step-input
    # construct the diagram
    builder.Connect(stepInput.get_output_port(0), mySys_Yuntian.get_input_port(0))  # connect input to mysystem
    logger_output = LogOutput(mySys_Yuntian.get_output_port(0), builder)
    logger_input = LogOutput(stepInput.get_output_port(0), builder)
    # after finishing the construction, just Build it.
    diagram = builder.Build()

    # Set the initial conditions, x1(0), x2(0), x3(0)
    context = diagram.CreateDefaultContext()
    context.SetDiscreteState([2.0])

    # Create the simulator
    simulator = Simulator(diagram, context)
    # Simulation time
    simulator.AdvanceTo(5)

    # Plot the results.
    plt.figure()
    plt.plot(logger_output.sample_times(), logger_output.data().transpose())
    plt.xlabel('t')
    plt.ylabel('y(t)')
    plt.show()

if __name__ == '__main__':
    test_DT_syst()