# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import pydrake.all
import numpy as np
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder


def LP_test():
    # Solve an optimization program
    # min -3x[0] - x[1] - 5x[2] -x[3] + 2
    # s.t 3x[0] + x[1] + 2x[2] = 30
    #     2x[0] + x[1] + 3x[2] + x[3] >= 15
    #     2x[1] + 3x[3] <= 25
    #     -100 <= x[0] + 2x[2] <= 40
    #   x[0], x[1], x[2], x[3] >= 0, x[1] <= 10
    prog = MathematicalProgram()
    # Declare x as decision variables.
    x = prog.NewContinuousVariables(4)
    # Add linear costs. To show that calling AddLinearCosts results in the sum of each individual
    # cost, we add two costs -3x[0] - x[1] and -5x[2]-x[3]+2
    prog.AddLinearCost(-3 * x[0] - x[1])
    prog.AddLinearCost(-5 * x[2] - x[3] + 2)
    # Add linear equality constraint 3x[0] + x[1] + 2x[2] == 30
    prog.AddLinearConstraint(3 * x[0] + x[1] + 2 * x[2] == 30)
    # Add Linear inequality constraints
    prog.AddLinearConstraint(2 * x[0] + x[1] + 3 * x[2] + x[3] >= 15)
    prog.AddLinearConstraint(2 * x[1] + 3 * x[3] <= 25)
    # Add linear inequality constraint -100 <= x[0] + 2x[2] <= 40
    prog.AddLinearConstraint(A=[[1., 2.]], lb=[-100], ub=[40], vars=[x[0], x[2]])
    prog.AddBoundingBoxConstraint(0, np.inf, x)
    prog.AddLinearConstraint(x[1] <= 10)

    # Now solve the program.
    result = Solve(prog)
    print(f"Is solved successfully: {result.is_success()}")
    print(f"x optimal value: {result.GetSolution(x)}")
    print(f"optimal cost: {result.get_optimal_cost()}")


def Pendulum_example():
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModelFromFile(
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
    plant.Finalize()
    diagram = builder.Build()
    simulator = Simulator(diagram)


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.





# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')
    print(pydrake.__file__)
    Pendulum_example()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
