import os
import time
import tracemalloc
import tsplib95

from python_tsp.distances import tsplib_distance_matrix
from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_simulated_annealing
from python_tsp.heuristics import solve_tsp_local_search

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from randomized_tsp.tsp import tsp

# tsp instances dimensions (0 for undefined)
min_instance_dimension = 500
max_instance_dimension = 10000
max_instance_quantity = 10

print_results_for_each_instance = False

# algorithms to run
python_tsp_dynamic_programming = False
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

randomized_tsp_genetic_algorithm = True
randomized_tsp_ant_colony = True


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

local_time_total = 0
local_memory_total = 0
local_distance_total = 0
simulated_time_total = 0
simulated_memory_total = 0
simulated_distance_total = 0
dynamic_time_total = 0
dynamic_memory_total = 0
dynamic_distance_total = 0
simulated_local_time_total = 0
simulated_local_memory_total = 0
simulated_local_distance_total = 0
dynamic_local_time_total = 0
dynamic_local_memory_total = 0
dynamic_local_distance_total = 0
simulated_local_p3_time_total = 0
simulated_local_p3_memory_total = 0
simulated_local_p3_distance_total = 0
dynamic_local_p3_time_total = 0
dynamic_local_p3_memory_total = 0
dynamic_local_p3_distance_total = 0
christofides_time_total = 0
christofides_memory_total = 0
christofides_distance_total = 0
clarke_time_total = 0
clarke_memory_total = 0
clarke_distance_total = 0
wen_time_total = 0
wen_memory_total = 0
wen_distance_total = 0
ortools_time_total = 0
ortools_memory_total = 0
ortools_distance_total = 0
cheapest_time_total = 0
cheapest_memory_total = 0
cheapest_distance_total = 0
ant_time_total = 0
ant_memory_total = 0
ant_distance_total = 0
genetic_time_total = 0
genetic_memory_total = 0
genetic_distance_total = 0

for problem in problems:
    if print_results_for_each_instance == True:
        print('\nResults for: ' + problem.name + '\n')

    distance_matrix = tsplib_distance_matrix(path + problem.name + extension)

    data = create_data_model(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()

    tsp_obj = tsp(distance_matrix)

    if python_tsp_local_search == True:
        tracemalloc.start()
        start = time.time()
        permutation3, distance3 = solve_tsp_local_search(distance_matrix)
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        local_time_total += end - start
        local_memory_total += memory
        local_distance_total += distance3
        if print_results_for_each_instance == True:
            print('Local Search:\t\t' + str(distance3) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if python_tsp_simulated_annealing == True:
        tracemalloc.start()
        start = time.time()
        permutation2, distance2 = solve_tsp_simulated_annealing(distance_matrix)
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        simulated_time = end - start
        simulated_memory = memory
        simulated_time_total += end - start
        simulated_memory_total += memory
        simulated_distance_total += distance2
        if print_results_for_each_instance == True:
            print('Simulated Annealing:\t' + str(distance2) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if python_tsp_dynamic_programming == True:
        tracemalloc.start()
        start = time.time()
        permutation, distance = solve_tsp_dynamic_programming(distance_matrix)
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        dynamic_time = end - start
        dynamic_memory = memory
        dynamic_time_total += end - start
        dynamic_memory_total += memory
        dynamic_distance_total += distance
        if print_results_for_each_instance == True:
            print('Dynamic Programming:\t' + str(distance) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if print_results_for_each_instance == True:
        print()

    if python_tsp_simulated_annealing == True and python_tsp_simulated_annealing_local_search == True:
        tracemalloc.start()
        start = time.time()
        permutation5, distance5 = solve_tsp_local_search(distance_matrix, x0=permutation2)
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        simulated_local_time_total += end - start + simulated_time
        simulated_local_memory_total += memory + simulated_memory
        simulated_local_distance_total += distance5
        if print_results_for_each_instance == True:
            print('Simulated Annealing then Local Search:\t\t\t' + str(distance5) + '\t' + str(round(end - start + simulated_time, 4)) + ' Sec\t' + str(round(memory + simulated_memory, 4)) + ' MB')

    if python_tsp_simulated_annealing == True and python_tsp_simulated_annealing_local_search_ps3 == True:
        tracemalloc.start()
        start = time.time()
        permutation7, distance7 = solve_tsp_local_search(distance_matrix, x0=permutation2, perturbation_scheme="ps3")
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        simulated_local_p3_time_total += end - start + simulated_time
        simulated_local_p3_memory_total += memory + simulated_memory
        simulated_local_p3_distance_total += distance7
        if print_results_for_each_instance == True:
            print('Simulated Annealing then Local Search ps3 Variant:\t' + str(distance7) + '\t' + str(round(end - start + simulated_time, 4)) + ' Sec\t' + str(round(memory + simulated_memory, 4)) + ' MB')

    if print_results_for_each_instance == True:
        print()

    if python_tsp_dynamic_programming == True and python_tsp_dynamic_programming_local_search == True:
        tracemalloc.start()
        start = time.time()
        permutation4, distance4 = solve_tsp_local_search(distance_matrix, x0=permutation)
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        dynamic_local_time_total += end - start + dynamic_time
        dynamic_local_memory_total += memory + dynamic_memory
        dynamic_local_distance_total += distance4
        if print_results_for_each_instance == True:
            print('Dynamic Programming then Local Search:\t\t\t' + str(distance4) + '\t' + str(round(end - start + dynamic_time, 4)) + ' Sec\t' + str(round(memory + dynamic_memory, 4)) + ' MB')

    if python_tsp_dynamic_programming == True and python_tsp_dynamic_programming_local_search_ps3 == True:
        tracemalloc.start()
        start = time.time()
        permutation6, distance6 = solve_tsp_local_search(distance_matrix, x0=permutation, perturbation_scheme="ps3")
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        dynamic_local_p3_time_total += end - start + dynamic_time
        dynamic_local_p3_memory_total += memory + dynamic_memory
        dynamic_local_p3_distance_total += distance6
        if print_results_for_each_instance == True:
            print('Dynamic Programming then Local Search ps3 Variant:\t' + str(distance6) + '\t' + str(round(end - start + dynamic_time, 4)) + ' Sec\t' + str(round(memory + dynamic_memory, 4)) + ' MB')

    if print_results_for_each_instance == True:
        print()

    if ortools_christofides == True:
        tracemalloc.start()
        start = time.time()
        result = run_algorithm('ortools_christofides')
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        christofides_time_total += end - start
        christofides_memory_total += memory
        christofides_distance_total += result
        if print_results_for_each_instance == True:
            print('Christofides:\t\t\t' + str(result) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if ortools_clarke_and_wright == True:
        tracemalloc.start()
        start = time.time()
        result = run_algorithm('ortools_clarke_and_wright')
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        clarke_time_total += end - start
        clarke_memory_total += memory
        clarke_distance_total += result
        if print_results_for_each_instance == True:
            print('Clarke and Wright:\t\t' + str(result) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if ortools_wren_and_holliday == True:
        tracemalloc.start()
        start = time.time()
        result = run_algorithm('ortools_wren_and_holliday')
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        wen_time_total += end - start
        wen_memory_total += memory
        wen_distance_total += result
        if print_results_for_each_instance == True:
            print('Wren and Holliday:\t\t' + str(result) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')
    
    if ortools_automatic_strategy == True:
        tracemalloc.start()
        start = time.time()
        result = run_algorithm('ortools_automatic_strategy')
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        ortools_time_total += end - start
        ortools_memory_total += memory
        ortools_distance_total += result
        if print_results_for_each_instance == True:
            print('ORTools Automatic Strategy:\t' + str(result) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if ortools_path_cheapest_arc == True:
        tracemalloc.start()
        start = time.time()
        result = run_algorithm('ortools_path_cheapest_arc')
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        cheapest_time_total += end - start
        cheapest_memory_total += memory
        cheapest_distance_total += result
        if print_results_for_each_instance == True:
            print('ORTools Cheapest Arc Strategy:\t' + str(result) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')
    
    if print_results_for_each_instance == True:
        print()

    if randomized_tsp_ant_colony == True:
        tracemalloc.start()
        start = time.time()
        tour2, cost2 = tsp_obj.ant_colony()
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        ant_time_total += end - start
        ant_memory_total += memory
        ant_distance_total += cost2
        if print_results_for_each_instance == True:
            print('Ant Colony:\t\t' + str(cost2) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if randomized_tsp_genetic_algorithm == True:
        tracemalloc.start()
        start = time.time()
        tour, cost = tsp_obj.genetic_algorithm()
        end = time.time()
        memory = tracemalloc.get_traced_memory()[1]/1000000
        tracemalloc.stop()
        genetic_time_total += end - start
        genetic_memory_total += memory
        genetic_distance_total += cost
        if print_results_for_each_instance == True:
            print('Genetic Algorithm:\t' + str(cost) + '\t' + str(round(end - start, 4)) + ' Sec\t' + str(round(memory, 4)) + ' MB')

    if print_results_for_each_instance == True:
        print()

print('\nResult Totals:\n')

if python_tsp_local_search == True:
    print('Local Search:\t\t' + str(local_distance_total) + '\t' + str(round(local_time_total, 4)) + ' Sec\t' + str(round(local_memory_total, 4)) + ' MB')

if python_tsp_simulated_annealing == True:
    print('Simulated Annealing:\t' + str(simulated_distance_total) + '\t' + str(round(simulated_time_total, 4)) + ' Sec\t' + str(round(simulated_memory_total, 4)) + ' MB')

if python_tsp_dynamic_programming == True:
    print('Dynamic Programming:\t' + str(dynamic_distance_total) + '\t' + str(round(dynamic_time_total, 4)) + ' Sec\t' + str(round(dynamic_memory_total, 4)) + ' MB')

print()

if python_tsp_simulated_annealing == True and python_tsp_simulated_annealing_local_search == True:
    print('Simulated Annealing then Local Search:\t\t\t' + str(simulated_local_distance_total) + '\t' + str(round(simulated_local_time_total, 4)) + ' Sec\t' + str(round(simulated_local_memory_total, 4)) + ' MB')

if python_tsp_simulated_annealing == True and python_tsp_simulated_annealing_local_search_ps3 == True:
    print('Simulated Annealing then Local Search ps3 Variant:\t' + str(simulated_local_p3_distance_total) + '\t' + str(round(simulated_local_p3_time_total, 4)) + ' Sec\t' + str(round(simulated_local_p3_memory_total, 4)) + ' MB')

print()

if python_tsp_dynamic_programming == True and python_tsp_dynamic_programming_local_search == True:
    print('Dynamic Programming then Local Search:\t\t\t' + str(dynamic_local_distance_total) + '\t' + str(round(dynamic_local_time_total, 4)) + ' Sec\t' + str(round(dynamic_local_memory_total, 4)) + ' MB')

if python_tsp_dynamic_programming == True and python_tsp_dynamic_programming_local_search_ps3 == True:
    print('Dynamic Programming then Local Search ps3 Variant:\t' + str(dynamic_local_p3_distance_total) + '\t' + str(round(dynamic_local_p3_time_total, 4)) + ' Sec\t' + str(round(dynamic_local_p3_memory_total, 4)) + ' MB')

print()

if ortools_christofides == True:
    print('Christofides:\t\t\t' + str(christofides_distance_total) + '\t' + str(round(christofides_time_total, 4)) + ' Sec\t' + str(round(christofides_memory_total, 4)) + ' MB')

if ortools_clarke_and_wright == True:
    print('Clarke and Wright:\t\t' + str(clarke_distance_total) + '\t' + str(round(clarke_time_total, 4)) + ' Sec\t' + str(round(clarke_memory_total, 4)) + ' MB')

if ortools_wren_and_holliday == True:
    print('Wren and Holliday:\t\t' + str(wen_distance_total) + '\t' + str(round(wen_time_total, 4)) + ' Sec\t' + str(round(wen_memory_total, 4)) + ' MB')
    
if ortools_automatic_strategy == True:
    print('ORTools Automatic Strategy:\t' + str(ortools_distance_total) + '\t' + str(round(ortools_time_total, 4)) + ' Sec\t' + str(round(ortools_memory_total, 4)) + ' MB')

if ortools_path_cheapest_arc == True:
    print('ORTools Cheapest Arc Strategy:\t' + str(cheapest_distance_total) + '\t' + str(round(cheapest_time_total, 4)) + ' Sec\t' + str(round(cheapest_memory_total, 4)) + ' MB')
    
print()

if randomized_tsp_ant_colony == True:
    print('Ant Colony:\t\t' + str(ant_distance_total) + '\t' + str(round(ant_time_total, 4)) + ' Sec\t' + str(round(ant_memory_total, 4)) + ' MB')

if randomized_tsp_genetic_algorithm == True:
    print('Genetic Algorithm:\t' + str(genetic_distance_total) + '\t' + str(round(genetic_time_total, 4)) + ' Sec\t' + str(round(genetic_memory_total, 4)) + ' MB')

print()