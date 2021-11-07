import matplotlib.pyplot as plt
from pydrake.all import (BasicVector, LeafSystem, DiagramBuilder, Simulator, LogOutput, ConstantVectorSource,
                         LinearSystem)
import numpy as np
import scipy.linalg as la


# Define the system.
class DTLinearSys(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        num_DTState = 2
        num_input = 1
        num_output = 2
        # Define the state vector
        self.DeclareDiscreteState(num_DTState)
        # Define the input
        self.DeclareVectorInputPort("u", BasicVector(num_input))
        # Define the output
        self.DeclareVectorOutputPort("y", BasicVector(num_output), self.CalcOutputY)
        self.DeclarePeriodicDiscreteUpdate(0.5)  # One second timestep.

    def DoCalcDiscreteVariableUpdates(self, context, events, discrete_state):
        x = context.get_discrete_state_vector().CopyToVector()
        xnext = 0.8 * x
        discrete_state.get_mutable_vector().SetFromVector(xnext)

    def CalcOutputY(self, context, output):
        x = context.get_discrete_state_vector().CopyToVector()
        y = x
        output.SetFromVector(y)

def test_ct_syst():
    # testing
    # Create a simple block diagram containing our system.
    builder = DiagramBuilder()
    mySys = builder.AddSystem(DTLinearSys())
    stepInput = builder.AddSystem(ConstantVectorSource([1]))  # step-input
    builder.Connect(stepInput.get_output_port(0), mySys.get_input_port(0))  # connect input to mysystem
    logger_output = LogOutput(mySys.get_output_port(0), builder)
    logger_input = LogOutput(stepInput.get_output_port(0), builder)
    diagram = builder.Build()

    # Set the initial conditions, x1(0), x2(0), x3(0)
    context = diagram.CreateDefaultContext()
    context.SetDiscreteState([0.5, 1])

    # Create the simulator
    simulator = Simulator(diagram, context)
    simulator.AdvanceTo(10)

    # Plot the results.
    plt.figure()
    plt.plot(logger_output.sample_times(), logger_output.data().transpose())
    plt.xlabel('t')
    plt.ylabel('y(t)')
    plt.show()

if __name__ == '__main__':
    test_ct_syst()