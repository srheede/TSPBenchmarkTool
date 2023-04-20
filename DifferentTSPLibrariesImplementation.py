## TSPLIB95
import tsplib95

## PYTHON_TSP
from python_tsp.distances import tsplib_distance_matrix
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_simulated_annealing
from python_tsp.heuristics import solve_tsp_local_search

## ORTOOLS
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

## PYTSPSOLVER
from pytspsolver.entities import TSProblem
from pytspsolver.experiments import Experiment
from pytspsolver.solvers import *
from pytspsolver.utilities import get_tsp_lib_problem, create_random_problem, Visualizer
import matplotlib.pyplot as plt

## SATSP
from satsp import solver

## CHRISTOFIDES
# from Christofides import christofides

## RANDOMIZED_TSP
from randomized_tsp.tsp import tsp

## TSP_ALGORITHMS
import tsp_algorithms

## TSP_SOLVER
import tsp_solver

import numpy as np

print(dir(tsp_solver))

## TSPLIB95

problem = tsplib95.load('datasets/tsp/a280.tsp')

print(problem.name)
print(problem.dimension)
print(problem.edge_weight_type)
print(len(list(problem.get_nodes())))
print(len(list(problem.get_edges())))
edge1 = list(problem.get_edges())[1]
print(edge1)
weight = problem.get_weight(*edge1)
print(weight)
print(problem.is_explicit()) #the problem specifies edge weights explicitly
print(problem.is_full_matrix()) #problem is specified as a full matrix
print(problem.is_weighted()) #EDGE_WEIGHT_FORMAT or the EDGE_WEIGHT_TYPE are defined
print(problem.is_complete()) #A complete graph is a graph in which each pair of graph vertices is connected by an edge.
print(problem.is_symmetric())

opt = tsplib95.load('datasets/tsp/a280.opt.tour')
print(problem.trace_tours(opt.tours))

distance_matrix = tsplib_distance_matrix('datasets/tsp/a280.tsp')

# TSPLIB95 doesn't seem to have solvers according to documentation: https://tsplib95.readthedocs.io/en/stable/

## PYTHON_TSP
#UNCOMMMENT TO RUN SOLVER
# permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
# permutation2, distance2 = solve_tsp_simulated_annealing(distance_matrix)
# permutation3, distance3 = solve_tsp_local_search(distance_matrix, x0=permutation2)
# permutation4, distance4 = solve_tsp_local_search(distance_matrix, x0=permutation2, perturbation_scheme="ps3")
#UNCOMMMENT TO RUN SOLVER

## ORTOOLS

def create_data_model(distance_matrix):
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 1
    data['depot'] = int(distance_matrix[0][0])
    return data

def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {} miles'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    plan_output += 'Route distance: {}miles\n'.format(route_distance)
    print(plan_output)

# Instantiate the data problem
data = create_data_model(distance_matrix)
# Create the routing index manager
manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
# Create Routing Model
routing = pywrapcp.RoutingModel(manager)
# Create the callback and register it with the solver 
transit_callback_index = routing.RegisterTransitCallback(distance_callback)
# Set the cost of travel
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
# Set the search parameters and a heuristic method for finding the first solution
search_parameters = pywrapcp.DefaultRoutingSearchParameters()

# Default
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

# Christofides
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)

# Automatic Selection
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)

# Savings algorithm (Clarke & Wright)
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.SAVINGS)

# Sweep algorithm (Wren & Holliday)
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.SWEEP)

# Solve the instance with search parameters and print solution to console
#UNCOMMMENT TO RUN SOLVER
# solution = routing.SolveWithParameters(search_parameters)
# if solution:
#     print_solution(manager, routing, solution)
#UNCOMMMENT TO RUN SOLVER


## PYTSP

# This package is under heavy development. The code may not be efficient and may potentially lead to bugs.

## PYTSPSOLVER

# Generate random problems in sizes of range 
problems = [create_random_problem("UniqueProblemName"+str(i), i) for i in range(3,12)]
# print(problems[0])

# Load problem from TSPLIB
tsp_prob = get_tsp_lib_problem('tsp/a280.tsp')

# Create a new Experiment
experiment = Experiment()

# Add the problems to the experiment (single or list of problems)
experiment.add_problem(tsp_prob) #Problem from TSPLIB95
experiment.add_problems(problems)

# print(dir(pytspsolver.solvers))
# Add solvers to use in the experiment
experiment.add_solver(ExhaustiveSearch(time_limit=50))
experiment.add_solver(GreedySearch(time_limit=100))
experiment.add_solver(GeneticAlgorithm())

# Run the experiment desired number of times
#UNCOMMMENT TO RUN SOLVER
# results = experiment.run(epoch=1)

# visualizer = Visualizer(results)

# visualizer.plot_n_vs_time_all(plt)
#UNCOMMMENT TO RUN SOLVER

## SATSP - solves the Travelling Salesman Problem (TSP) using Simulated Annealing (SA)

#UNCOMMMENT TO RUN SOLVER
# solver.Solve(distance_matrix)
# print(solver.GetBestDist())
#UNCOMMMENT TO RUN SOLVER

## CHRISTOFIDES

# Library is broken
# TSP = christofides.compute(distance_matrix)
# print(dir(Christofides))
# TSP = Christofides.christofides.compute(distance_matrix)
# print(TSP['Chistofides_Solution'])
# print(TSP['Travel_Cost'])

## RANDOMIZED_TSP

tsp_obj = tsp(distance_matrix)
#UNCOMMMENT TO RUN SOLVER
# To run genetic algorithm
# tour, cost = tsp_obj.genetic_algorithm()
# print(cost)
# To run simulated annealing
# tour2, cost2 = tsp_obj.simulated_annealing()
# print(cost2)
# To run ant colony optimization
# tour3, cost3 = tsp_obj.ant_colony()
# print(cost3)
#UNCOMMMENT TO RUN SOLVER

# Optional parameters for the above algorithms
# def genetic_algorithm(self,
#                       population_size=50,
#                       mutation_prob=0.1,
#                       crossover='order'):
#         """
#         :param population_size: Defines the size of the population used in the algorithm
#         :param mutation_prob:   Probability that a offspring will mutate
#         :param crossover:       Defines the crossover operator, currently two options are available
#                                 `order` and `cycle`.
#         :return:                Returns the best tour found and the cost of that tour
#                                 A tour is represented using path representation
#         """

# def simulated_annealing(self):
#         """
#         :return: Returns the best tour found and the cost of that tour
#                  A tour is represented using path representation
#         """

# def ant_colony(self,
#                num_of_ants=20,
#                pheromone_evapouration=0.2):
#         """
#         :param num_of_ants:            Number of ants in the colony
#         :param pheromone_evapouration: Evapouration rate of the pheromone deposited by ants
#         :return:                       Returns the best tour found and the cost of that tour
#                                        A tour is represented using path representation
#         """

## TSP_ALGORITHMS
# No Documentation

## TSP_SOLVER
# No Documentation

## GT-TSP
# No Documentation
