import os
import tsplib95

from python_tsp.distances import tsplib_distance_matrix
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_simulated_annealing
from python_tsp.heuristics import solve_tsp_local_search

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# tsp instances dimensions (0 for undefined)
min_instance_dimension = 0
max_instance_dimension = 15
max_instance_quantity = 1

# algorithms to run
python_tsp_dynamic_programming = True
python_tsp_simulated_annealing = True
python_tsp_local_search = True
python_tsp_dynamic_programming_local_search = True
python_tsp_simulated_annealing_local_search = True
python_tsp_dynamic_programming_local_search_ps3 = True
python_tsp_simulated_annealing_local_search_ps3 = True

ortools_path_cheapest_arc = True
ortools_christofides = True
ortools_automatic_strategy = True
ortools_clarke_and_wright = True
ortools_wren_and_holliday = True

path = 'datasets/tsp/'
extension = '.tsp'
files = os.listdir(path)

## ORTOOLS METHODS

def create_data_model(distance_matrix):
    data = {}
    data['distance_matrix'] = distance_matrix
    data['num_vehicles'] = 1
    data['depot'] = int(distance_matrix[0][0])
    return data

def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]

def run_algorithm(algorithm):
    if algorithm == 'ortools_path_cheapest_arc':
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    elif algorithm == 'ortools_christofides':
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
    elif algorithm == 'ortools_automatic_strategy':
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    elif algorithm == 'ortools_clarke_and_wright':
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.SAVINGS)
    elif algorithm == 'ortools_wren_and_holliday':
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.SWEEP)

    return routing.SolveWithParameters(search_parameters).ObjectiveValue()


tspinstances = []
problems = []

for file in files:
    name, ext = os.path.splitext(file)
    if ext == extension:
        tspinstance = tsplib95.load(path + file)
        tspinstances.append(tspinstance)
        if (tspinstance.name != name):
            tspinstances.remove(tspinstance)

for tspinstance in tspinstances:
    if len(problems) == max_instance_quantity and max_instance_quantity != 0:
        break
    if tspinstance.is_complete() and tspinstance.is_symmetric() and tspinstance.is_weighted():
        if min_instance_dimension < tspinstance.dimension and tspinstance.dimension < max_instance_dimension:
            problems.append(tspinstance)
        elif min_instance_dimension < tspinstance.dimension and max_instance_dimension == 0:
            problems.append(tspinstance)


for problem in problems:
    print('Results for: ' + problem.name)
    print()

    distance_matrix = tsplib_distance_matrix(path + problem.name + extension)

    if python_tsp_dynamic_programming == True:
        permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
        print('Route distance for python_tsp_dynamic_programming: ' + str(distance))

    if python_tsp_simulated_annealing == True:
        permutation2, distance2 = solve_tsp_simulated_annealing(distance_matrix)
        print('Route distance for python_tsp_simulated_annealing: ' + str(distance2))

    if python_tsp_local_search == True:
        permutation3, distance3 = solve_tsp_local_search(distance_matrix)
        print('Route distance for python_tsp_local_search: ' + str(distance3))

    if python_tsp_dynamic_programming_local_search == True:
        permutation4, distance4 = solve_tsp_local_search(distance_matrix, x0=permutation)
        print('Route distance for python_tsp_dynamic_programming_local_search: ' + str(distance4))

    if python_tsp_simulated_annealing_local_search == True:
        permutation5, distance5 = solve_tsp_local_search(distance_matrix, x0=permutation2)
        print('Route distance for python_tsp_simulated_annealing_local_search: ' + str(distance5))

    if python_tsp_dynamic_programming_local_search_ps3 == True:
        permutation6, distance6 = solve_tsp_local_search(distance_matrix, x0=permutation, perturbation_scheme="ps3")
        print('Route distance for python_tsp_dynamic_programming_local_search_ps3: ' + str(distance6))

    if python_tsp_simulated_annealing_local_search_ps3 == True:
        permutation7, distance7 = solve_tsp_local_search(distance_matrix, x0=permutation2, perturbation_scheme="ps3")
        print('Route distance for python_tsp_simulated_annealing_local_search_ps3: ' + str(distance7))

    data = create_data_model(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()

    if ortools_path_cheapest_arc == True:
        print('Route distance for ortools_path_cheapest_arc: ' + str(run_algorithm('ortools_path_cheapest_arc')))

    if ortools_christofides == True:
        print('Route distance for ortools_christofides: ' + str(run_algorithm('ortools_christofides')))

    if ortools_automatic_strategy == True:
        print('Route distance for ortools_automatic_strategy: ' + str(run_algorithm('ortools_automatic_strategy')))

    if ortools_clarke_and_wright == True:
        print('Route distance for ortools_clarke_and_wright: ' + str(run_algorithm('ortools_clarke_and_wright')))

    if ortools_wren_and_holliday == True:
        print('Route distance for ortools_wren_and_holliday: ' + str(run_algorithm('ortools_wren_and_holliday')))
