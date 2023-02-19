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

import numpy as np

## TSPLIB95

problem = tsplib95.load('tsp/a280.tsp')

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

opt = tsplib95.load('tsp/a280.opt.tour')
print(len(opt.tours[0]))
print(problem.trace_tours(opt.tours))

distance_matrix = tsplib_distance_matrix('tsp/a280.tsp')

## PYTHON_TSP

# permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
# permutation2, distance2 = solve_tsp_simulated_annealing(distance_matrix)
# permutation3, distance3 = solve_tsp_local_search(distance_matrix, x0=permutation2)
# permutation4, distance4 = solve_tsp_local_search(distance_matrix, x0=permutation2, perturbation_scheme="ps3")

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

# DEFAULT
# search_parameters.first_solution_strategy = (
#     routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

#CHRISTOFIDES
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)

#AUTOMATIC SELECTION
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)

#Savings algorithm (Clarke & Wright)
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.SAVINGS)

#Sweep algorithm (Wren & Holliday)
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.SWEEP)

# Solve the instance with search parameters and print solution to console
solution = routing.SolveWithParameters(search_parameters)
if solution:
    print_solution(manager, routing, solution)
else:
    print("nothing")
