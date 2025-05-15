# Required imports
from json import detect_encoding

import networkx as nx
import numpy as np
from Boundaries import Boundaries
from Map import EPSILON
from utils.debug import *

# Number of nodes expanded in the heuristic search (stored in a global variable to be updated from the heuristic functions)
NODES_EXPANDED = 0


def h1(current_node, objective_node) -> np.float32:
    """First heuristic to implement"""
    global NODES_EXPANDED
    h = abs(objective_node[0] - current_node[0]) + abs(
        objective_node[1] - current_node[1]
    )
    NODES_EXPANDED += 1
    return h


def h2(current_node, objective_node) -> np.float32:
    """Second heuristic to implement"""
    global NODES_EXPANDED
    cost_value = 1
    h = (
        abs(objective_node[0] - current_node[0])
        + abs(objective_node[1] - current_node[1]) * cost_value
    )
    NODES_EXPANDED += 1
    return h


def h3(current_node, objective_node) -> np.float32:
    """
    Third heuristic implemented, an euclidean distance between the current and objective node
    """
    global NODES_EXPANDED
    h = np.sqrt(
        (objective_node[0] - current_node[0]) ** 2
        + (objective_node[1] - current_node[1]) ** 2
    )
    NODES_EXPANDED += 1
    return h


def build_graph(detection_map: np.ndarray, tolerance: np.float32) -> nx.DiGraph:
    """
    Builds an adjacency graph (not an adjacency matrix) from the detection map
    The only possible connections from a point in space (now a node in the graph) are:
       -> Go up
       -> Go down
       -> Go left
       -> Go right
     Not every point has always 4 possible neighbors, and not every point can be traversed based on plane tolerance
    """
    # Define and get basic variables
    height, width = detection_map.shape
    graph = nx.DiGraph()

    # Define possible directions: up, down, left, right.
    # Define their respective changes on the coordinate
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Create edges between cells based on tolerance
    for y in range(height):
        for x in range(width):
            ## Add the node in that position
            graph.add_node((y, x))

            # Check all four directions to create edges
            for dy, dx in directions:
                n_y, n_x = y + dy, x + dx
                # Check if neighbor is within bounds
                if 0 <= n_y < height and 0 <= n_x < width:
                    neighbor_value = detection_map[n_y, n_x]

                    # Add an edge with the tolerance value of the neighbour only if
                    # the tolerance is below or equal to the one of the plane
                    if detection_map[n_x, n_y] <= tolerance:
                        graph.add_edge(
                            (x, y), (n_x, n_y), weight=detection_map[n_x, n_y]
                        )
                        print_d(
                            f"Added new edge from: {x,y} to {n_x,n_y} with weight = {detection_map[n_x, n_y]}"
                        )

    return graph


def discretize_coords(
    high_level_plan: np.ndarray,
    boundaries: Boundaries,
    map_width: np.int32,
    map_height: np.int32,
) -> np.ndarray:
    """
    Converts coordiantes from (lat, lon) into (x, y) on all the high level plan
    """
    locations = np.zeros(shape=(len(high_level_plan), 2), dtype=np.int32)
    lat_step = (boundaries.max_lat - boundaries.min_lat) / (map_height - 1)
    lon_step = (boundaries.max_lon - boundaries.min_lon) / (map_width - 1)
    for i in range(len(high_level_plan)):
        # Lat goes to X
        # Lon goes to Y
        locations[i][0] = int((high_level_plan[i][0] - boundaries.min_lat) / lat_step)
        locations[i][1] = int((high_level_plan[i][1] - boundaries.min_lon) / lon_step)

    return locations


def path_finding(
    G: nx.DiGraph,
    heuristic_function,
    locations: np.ndarray,
    initial_location_index: np.int32,
    boundaries: Boundaries,
    map_width: np.int32,
    map_height: np.int32,
) -> tuple[list, int]:
    """
    Implementation of the main searching / path finding algorithm, using the astar_path algorithm given in the
    nx library.
    The algorithm works in the following way:
    - Starts in the intial position and uses the ASTAR algorithm to look for a path to the next position on the list of positions. If 
        no path can be found then it prints the error when the exception rises, else it appends the obtained path to the complete path (final path)
    - When all paths between the required locations have been searched, the function returns the complete path in string format (the one required for the plotting function)
    """

    # Start by turning locations into their (x,y) form, using the discretize_coords function
    xy_locations = np.zeros(shape=(len(locations), 2), dtype=np.int32)
    xy_locations = discretize_coords(locations, boundaries, map_width, map_height)

    print_d(f"The xy_locations array holds the values: {xy_locations}")

    # Start to look for the path with ASTAR
    current_location = tuple(xy_locations[initial_location_index])
    solution_plan = []
    for location in xy_locations:
        # 1. Search for a path from the current location to the next one
        print_d(f"Now looking for a path to location {location} in {xy_locations}")
        try:
            path = nx.astar_path(
                G, tuple(current_location), tuple(location), heuristic_function
            )
            print_d(f"Path found from {current_location} to {location}: {path}")
        except nx.NetworkXNoPath:
            print(
                f"No path found from {current_location} to {location} with the given tolerance"
            )
            # Exit the program with -1
            exit(-1)

        path_len = nx.astar_path_length(
            G, tuple(current_location), tuple(location), heuristic_function
        )

        # 2. Change the current location to the reached location
        current_location = location

        # 3. Append the path
        solution_plan.append([str(coord) for coord in path])

    print_d(f"Solution plan: {solution_plan}")
    return solution_plan, NODES_EXPANDED


def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """
    Computes the total cost of the whole planning solution.
    Each movement adds one to the cost
    """
    operation_cost= 1
    total_cost = np.float32(0.0)
    for path in solution_plan:
        total_cost += len(path) * operation_cost

    return total_cost
