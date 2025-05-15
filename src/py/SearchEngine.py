# Required imports
import numpy as np
import networkx as nx
from Boundaries import Boundaries
from Map import EPSILON

# Number of nodes expanded in the heuristic search (stored in a global variable to be updated from the heuristic functions)
NODES_EXPANDED = 0

# Creation of the class State: 

class State:


def h1(current_node, objective_node) -> np.float32:
    """ First heuristic to implement """
    global NODES_EXPANDED
    h = 0
    ...
    NODES_EXPANDED += 1
    return h

def h2(current_node, objective_node) -> np.float32:
    """ Second heuristic to implement """
    global NODES_EXPANDED
    h = 0
    ...
    NODES_EXPANDED += 1
    return h

def build_graph(detection_map: np.array, tolerance: np.float32) -> nx.DiGraph:
    """ Builds an adjacency graph (not an adjacency matrix) from the detection map """
    # The only possible connections from a point in space (now a node in the graph) are:
    #   -> Go up
    #   -> Go down
    #   -> Go left
    #   -> Go right
    # Not every point has always 4 possible neighbors
    ...

def discretize_coords(high_level_plan: np.array, boundaries: Boundaries, map_width: np.int32, map_height: np.int32) -> np.array:
    """ Converts coordiantes from (lat, lon) into (x, y) """
    ...

def path_finding(G: nx.DiGraph,
                 heuristic_function,
                 locations: np.array, 
                 initial_location_index: np.int32, 
                 boundaries: Boundaries,
                 map_width: np.int32,
                 map_height: np.int32) -> tuple:
    """ Implementation of the main searching / path finding algorithm """

    # Initialize function variables
    open = [initial_location_index]
    closed = []
    goal = False
    initialHeuristic = currentState.heuristicCost
    
    while len(open) > 0:
        currentState = open.pop()
        
        if currentState.finalState:
            goal = True
            break
        if currentState.heuristicCost == float('inf'):
            print(f"NOTE -- NO SOLUTIONS FOUND FOR A* ALGORITHM")
            return initialHeuristic,expandedNodes,currentState,False

        if currentState in closed:
            continue

        closed.append(currentState)
        

        successors = currentState.expand_state()
        open = sorted(open + successors, key=lambda x: x.totalCost)
        expandedNodes += 1

        # DEBUG:
        print_d(f"DEBUG -- OPEN: {len(open)} CLOSED: {len(closed)} SUCCESSORS: {len(successors)}")
        print_d(f"NOTE -- Expanded Nodes: {expandedNodes}")
        print_d(f"NOTE -- Time expended: {round(time.time()-initialTime,3)}")
        if DEBUG:
            sys.stdout.write("\033[F\033[K\033[F\033[K\033[F\033[K")  # Clear debug lines
                    

    if goal:
        print(f"NOTE -- FINISHED A* ALGORITHM") 
        return initialHeuristic,expandedNodes,currentState,True

    elif len(open) == 0:
        print(f"NOTE -- NO SOLUTIONS FOUND FOR A* ALGORITHM")
        return initialHeuristic,expandedNodes,currentState,False

    else:
        print_d(f"WARNING -- EXITED A*")
        sys.exit(1)



def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """ Computes the total cost of the whole planning solution """
    ...
