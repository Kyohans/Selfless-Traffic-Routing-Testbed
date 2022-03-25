#!/usr/bin/env python
# -*- coding: utf-8 -*-

from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
from copy import deepcopy
import csv
from collections import defaultdict

class AStarPolicy(RouteController):
    """
    A* Search chooses nodes in graph traversals based on several heuristic functions
    g(n) = distance to start node
    h(n) = distance to goal node
    f(n) = total cost (g(n) + h(n))
    """
    def __init__(self, connection_info):
        super().__init__(connection_info)

    def heuristic(self, outgoing, goal):
        return outgoing + goal

    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions will use A* Search to find the shortest path for each individual vehicle to their destination
        :param vehicles: List of vehicles
        :param connection_info: Information on map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:
            decision_list = []
            unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list}
            visited = {}
            
            starting_edge = vehicle.current_edge
            g = defaultdict(int) # Distance from starting edge
            g[starting_edge] = 0

            current_edge = vehicle.current_edge
            current_distance = self.connection_info.edge_length_dict[current_edge]
            destination_distance = self.connection_info.edge_length_dict[vehicle.destination]
            
            unvisited[current_edge] = current_distance
            path_list = {edge: [] for edge in self.connection_info.edge_list}
            while unvisited or current_edge != vehicle.destination:
                if current_edge not in self.connection_info.edge_length_dict.keys():
                    continue

                n = None # edge with lowest f(n) found
                for uv in unvisited.items():
                    uv_length = self.connection_info.edge_length_dict[uv[0]]
                    if n is None or g[uv[0]] + self.heuristic(uv_length, destination_distance) < g[n] + self.heuristic(self.connection_info.edge_length_dict[n], destination_distance):
                        n = uv[0]

                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    g[outgoing_edge] = g[starting_edge] + edge_length

                    length_f = g[outgoing_edge] + self.heuristic(edge_length, destination_distance)
                    if length_f < unvisited[outgoing_edge]:
                        unvisited[outgoing_edge] = length_f
                        current_path = deepcopy(path_list[current_edge])
                        current_path.append(direction)
                        path_list[outgoing_edge] = deepcopy(current_path)
            
                    visited[current_edge] = current_distance
                    del unvisited[current_edge]

                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(possible_edges, key = lambda x: x[1])[0]

            for direction in path_list[vehicle.destination]:
                decision_list.append(direction)

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets

