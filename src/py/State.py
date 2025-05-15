import networkx as nx
import numpy as np
from utils.debug import *


class State:
    def __init__(
        self,
        plane_position: np.int32,
        prev: "State|None",
        cost: np.float32,
        heuristicType: int,
        map: nx.DiGraph,
        plane_goals: np.ndarray,
        fwCosts: dict[tuple[int, int], dict[tuple[int, int], float]] | None = None,
    ):

        self.plane_position = plane_position
        self.prev = prev
        self.cost = cost
        self.heuristicType = heuristicType
        self.map = map
        self.plane_goals = plane_goals
        self.fwCosts = fwCosts

    def __str__(self) -> str:
        string = f"State:\n\
        * plane_position: {self.plane_position}\n\
        * Cost: {self.cost} \n\
        * HeuristicCost: {self.heuristicCost}\n\
        * TotalCost: {self.totalCost}\n\
        * plane__Goals: {self.plane_goals}\n"

        return string

    def __eq__(self, other) -> bool:
        return self.plane_position == other.plane_position


    @property
    def heuristicCost(self) -> float:
        # Use euler
        if self.heuristicType == 1:
            return self.heuristic_manhattan()
        # Use manhattan
        elif self.heuristicType == 2:
            return self.heuristic_manhattan_cost_scaled()
        elif self.heuristicType == 3:
            return self.heuristic_euclidean()
        # Use floydWarshall: Not implemented
        # elif self.heuristicType == 2:
        #     return self.heuristic_floydWarshall()
        # Use dikstra, no heuristic
        elif self.heuristicType == 0:
            return 0
        # Use dikstra but print the error
        else:
            print_d("ERROR: Invalid heuristic type")
            return 0

    @property
    def totalCost(self) -> float:
        return self.cost + self.heuristicCost

    @property
    def possibleMoves(self) -> list[tuple[int, int]]:
        moves = [(0, 1), (0, -1), (-1, 0), (1, 0)]  # up  # down  # left  # right
        return moves

    @property
    def finalState(self) -> bool:
        """
        If the plane is at any of the required positions we take it
        out of the list of target_positions. When the list is empty -> finalState
        """
        for position in self.plane_goals:
            if self.plane_position == self.plane_goals:
                print_d(f"Reached point {position}")
                self.plane_goals = np.delete(self.plane_goals, position)
                break
        # If there is no position left in the array return finalState = True
        if len(self.plane_goals) == 0:
            return True
        else:
            return False

    @property
    def moveCost(self) -> float:
        return 1

    def heuristic_manhattan(self) -> float:
        """
        Compute the manhattan distance between the
        current position and goal positions,return
        * the sum of all those distances?? 
        * the max distance? --> Choosing this one right now
        :return (float): Heuristic value
        """

        heuristic_values = []
        for i in range(len(self.plane_goals)):
            initial = self.plane_position
            final = self.plane_goals[i]
            heuristic_values.append(
                abs(final[0] - initial) + abs(final[1] - initial)
            )
        return max(heuristic_values)

    def heuristic_manhattan_cost_scaled(self) -> float:
        """
        Compute the manhattan distance between the
        current position and goal positions,return
        * the sum of all those distances?? 
        * the max distance? --> Choosing this one right now
        :return (float): Heuristic value
        """

        heuristic_values = []
        for i in range(len(self.plane_goals)):
            initial = self.plane_position
            final = self.plane_goals[i]
            heuristic_values.append(
                (abs(final[0] - initial) + abs(final[1] - initial)) * self.moveCost
            )
        return max(heuristic_values)

    def heuristic_euclidean(self) -> float:
        """
        Compute the euclidean distance between the
        current position and the goal positions.
        :return (float): Heuristic value
        """
        heuristicsValues = []
        for i in range(len(self.plane_goals)):
            initial = self.plane_position
            final = self.plane_goals[i]
            heuristicsValues.append(
                np.sqrt((final[0] - initial) ** 2 + (final[1] - initial) ** 2)
            )
        return max(heuristicsValues)

    def heuristic_floydWarshall(self) -> float:
        """
        Compute the Floyd-Warshall algorithm if this node is the
        first one. Else use the precalculated values.
        :return (float): Heuristic value
        """
        if self.fwCosts == None:
            self.fwCosts = self.algorithm_floydWarshall()
        heuristicValues = []
        for i in range(len(self.plane_goals)):
            initial = self.plane_position
            final = self.plane_goals[i]
            heuristicCost = self.fwCosts[initial][final]
            heuristicValues.append(heuristicCost)

        return max(heuristicValues)

    def expand_state(self) -> list["State"]:
        """
        This function expands the current state to get all the possible child states/next states.
        Based on the following rules:
        + A plane can move in any direction (up, down, left, right) or wait
        + There cannot be two planes in the same position
        + Two planes cannot cross each other

        :return (list[State]): Sorted list of all the possible child states based on their total cost
        """
        childStates = []
        childCosts = []
        childPositionValues, childMoveValues = self.operator_move()

        # Create ChildStates:
        for elem in childMoveValues:
            childCosts.append(self.get_child_cost(elem))

        for i in range(len(childPositionValues)):
            childStates.append(
                State(
                    childPositionValues[i],
                    self,
                    self.cost + childCosts[i],
                    self.heuristicType,
                    self.map,
                    self.planeGoals,
                    self.fwCosts,
                )
            )

        # NOTE: Use the sorted function to sort based on total cost of objects of the class.
        return sorted(childStates, key=lambda x: x.totalCost)


def astar(initialState: State) -> tuple[float, int, State, bool]:
    """
    This function implements the A* algorithm.
    :param open: Open list of states. Starts with the initial state
    :param closed: Closed list of states. Starts empty
    :param goal: Boolean to check whether the goal has been reached
    :return (float): Initial heuristic value of the first state
    :return (int): Number of expanded nodes
    :return (State): Final state of the problem: Backtrack to get the solution
        -> The get_parse_solution() function will be used to get the solution
    """
    initialTime = time.time()

    # Initialize function variables
    open = [initialState]
    closed = []
    goal = False
    expandedNodes = 0
    currentState: State = open[0]
    initialHeuristic = currentState.heuristicCost

    while len(open) > 0:
        currentState = open.pop(0)

        if currentState.finalState:
            goal = True
            break
        if currentState.heuristicCost == float("inf"):
            print(f"NOTE -- NO SOLUTIONS FOUND FOR A* ALGORITHM")
            return initialHeuristic, expandedNodes, currentState, False
            break

        if currentState in closed:
            continue

        closed.append(currentState)

        if (MAXEXPANSION and expandedNodes >= MAXEXPANSION) or (
            MAXTIME and time.time() - initialTime >= MAXTIME
        ):
            break

        successors = currentState.expand_state()
        open = sorted(open + successors, key=lambda x: x.totalCost)
        expandedNodes += 1

        # DEBUG:
        print_d(
            f"DEBUG -- OPEN: {len(open)} CLOSED: {len(closed)} SUCCESSORS: {len(successors)}"
        )
        print_d(f"NOTE -- Expanded Nodes: {expandedNodes}")
        print_d(f"NOTE -- Time expended: {round(time.time()-initialTime,3)}")
        if DEBUG:
            sys.stdout.write(
                "\033[F\033[K\033[F\033[K\033[F\033[K"
            )  # Clear debug lines

    if goal:
        print(f"NOTE -- FINISHED A* ALGORITHM")
        return initialHeuristic, expandedNodes, currentState, True

    elif len(open) == 0:
        print(f"NOTE -- NO SOLUTIONS FOUND FOR A* ALGORITHM")
        return initialHeuristic, expandedNodes, currentState, False

    else:
        print_d(f"WARNING -- EXITED A*")
        sys.exit(1)


def get_parse_solution(
    final_state: State,
) -> tuple[int, list[list[tuple[int, int]]], list[list[str]]]:
    """
    Reconstructs the solution path from the final state by backtracking.

    Args:
        final_state: The goal state reached by A*

    Returns:
        tuple: (makespan, list of position sequences, list of move sequences)
    """
    # Initialize tracking for each plane
    num_planes = len(final_state.planePositions)
    position_sequences = [[] for _ in range(num_planes)]
    move_sequences = [[] for _ in range(num_planes)]

    # Backtrack through states
    current_state = final_state
    makespan = 0

    while current_state:
        # Record positions for each plane
        for i in range(num_planes):
            position_sequences[i].insert(0, current_state.planePositions[i])

        # Determine moves by comparing consecutive states
        if current_state.prev:
            makespan += 1
            for i in range(num_planes):
                current_pos = current_state.planePositions[i]
                prev_pos = current_state.prev.planePositions[i]

                # Calculate move direction
                dx = current_pos[0] - prev_pos[0]
                dy = current_pos[1] - prev_pos[1]

                # Convert to move symbol
                if (dx, dy) == (0, 0):
                    move_sequences[i].insert(0, "w")
                elif (dx, dy) == (0, 1):
                    move_sequences[i].insert(0, "→")
                elif (dx, dy) == (0, -1):
                    move_sequences[i].insert(0, "←")
                elif (dx, dy) == (1, 0):
                    move_sequences[i].insert(0, "↓")
                elif (dx, dy) == (-1, 0):
                    move_sequences[i].insert(0, "↑")

        current_state = current_state.prev

    return makespan, position_sequences, move_sequences
