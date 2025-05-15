# Required imports
import networkx as nx
import numpy as np
from Boundaries import Boundaries
from Map import EPSILON

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
     Not every point has always 4 possible neighbors, and not every point can be traversed based on plane tolerance, and not every point can be traversed based on plane tolerance
    """
    height, width = detection_map.shape
    ...


def discretize_coords(
    high_level_plan: np.ndarray,
    boundaries: Boundaries,
    map_width: np.int32,
    map_height: np.int32,
) -> np.ndarray:
    """Converts coordiantes from (lat, lon) into (x, y)"""
    lat_step = (boundaries.max_lat - boundaries.min_lat)/(map_height-1)
    lon_step = (boundaries.max_lat - boundaries.min_lon)/ (map_width -1)
    # Lat goes to X
    # Lon goes to Y
    x = int(high_level_plan[0] - boundaries.min_lat / lat_step)
    y = int(high_level_plan[1] - boundaries.min_lon/ lon_step)
    return np.ndarray([x,y])


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
    - Starts in the initial position and looks for the shortest path from the initial position to another of the positions in the list of locations. When found, sets this new position as the starting position and repeats. All piece-wise paths are saved
    """
    # Start by turning locations into their (x,y) form: 
    xy_locations = np.array([])
    for location in locations:
        xy_locations = discretize_coords(location, boundaries, map_width, map_height)
    
    locations = xy_locations

    # Get the intial location of the plane
    current_location = locations[initial_location_index]
    # Update the locations so that the initial location is no longer there
    locations = np.delete(locations, current_location)
    # Start searching
    solution_plan = []
    for _ in range(len(locations)):
        path = []
        next_location = []

        # Start looking for the paths from the current location
        for location in locations:
            if len(path) == 0:
                path = nx.astar_path(G, current_location, location, heuristic_function)
                next_location = location
            else:
                temp_path = nx.astar_path(
                    G, current_location, location, heuristic_function
                )
                if len(temp_path) < len(path):
                    path = temp_path
                    next_location = location

        solution_plan.append(path)
        # Get the next location of the plane
        current_location = next_location
        # Update the locations so that the initial location is no longer there
        locations = np.delete(locations, current_location)

    return solution_plan, NODES_EXPANDED


def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """
    Computes the total cost of the whole planning solution.
    Each movement adds one to the cost
    """
    cost_value = 1
    return np.float32(len(solution_plan) * cost_value)
