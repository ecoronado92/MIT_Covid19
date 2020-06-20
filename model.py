#CVRP MODEL (Can also consider time constraints)
from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


import pandas as pd
import numpy as np
from tqdm import tqdm
from objects.input_class import Inputs


"""
route optimization algorithm V1
Simulates random points, computes cost matrix and minimized cost).
Computes which trucks should distribute which covending machines.

"""


def create_data_model(num_vehicles,num_points):
    """Stores the data for the problem."""
    matrix , lat_lon , demands = Inputs().cost_matrix(num_points)
    data = {}
    data['distance_matrix'] = matrix

    # CANT CARGA FOR FIRST_ECHELON, DEMANDA FOR ALL THE CLUSTERS
    data['demands'] = demands
    data['vehicle_capacities'] = [550 for i in range(num_vehicles)]
    data['num_vehicles'] = num_vehicles
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, assignment):
    """Prints assignment on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
        # with open(f"{vehicle_id}.txt", "w") as file:
        #     file.write(plan_output)
        #     file.close()
        #
    print('Total cost for all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))
    # with open(f"{data['num_vehicles']}vehicles.txt", "w") as file:
    #     out_file = ""
    #     out_file += str(total_load) + "," + str(total_distance)
    #     file.write(out_file)
    #     file.close()  # OPEN AND ANALYZE LATER WITH PANDAS


def solucionar(num_vehicles,num_points):
    data = create_data_model(num_vehicles,num_points)
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """cost between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.

    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)


    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    # Print solution on console.
    if assignment:
        print_solution(data, manager, routing, assignment)
    else:
        print("No feasible solution found")




solucionar(1,6)
