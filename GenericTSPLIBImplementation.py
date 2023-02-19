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

# Instantiate the data problem
data = create_data_model(distance_matrix)
# Create the routing index manager
manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
# Create Routing Model
routing = pywrapcp.RoutingModel(manager)

print(routing)