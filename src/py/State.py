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
            return self.heuristic_floydWarshall()
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
        for i in range(len(self.planePositions)):
            initial = self.planePositions[i]
            final = self.planeGoals[i]
            heuristicsValues.append(
                math.sqrt((final[0] - initial[0]) ** 2 + (final[1] - initial[1]) ** 2)
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
        for i in range(len(self.planePositions)):
            initial = self.planePositions[i]
            final = self.planeGoals[i]
            heuristicCost = self.fwCosts[initial][final]
            heuristicValues.append(heuristicCost)

        return max(heuristicValues)

    def algorithm_floydWarshall(
        self,
    ) -> dict[tuple[int, int], dict[tuple[int, int], float]]:
        """
        Compute the Floyd-Warshall algorithm with wall detection.
        Nodes are considered adjacent only if they are Manhattan distance 1 apart
        and at least one of them is not a wall ("G").
        :return (dict): Dictionary mapping each position to its minimum distances to all other positions
        """
        # Get the map from the instance
        map = self.map

        # Initialize the mapping from node indices to their positions
        n = 1
        nodes = {}  # Maps index to position
        reverse_nodes = {}  # Maps position to index (needed for result conversion)
        for i in map.keys():
            nodes[n] = i
            reverse_nodes[i] = n
            n += 1
        num_nodes = len(nodes)

        # Initialize the distance matrix with infinities
        distanceArray = [[float("inf")] * num_nodes for _ in range(num_nodes)]

        # Distance from a node to itself is 0
        for i in range(num_nodes):
            distanceArray[i][i] = 0

        # Populate the adjacency matrix
        for i in range(1, num_nodes + 1):
            for j in range(1, num_nodes + 1):
                if i != j:
                    pos_i = nodes[i]
                    pos_j = nodes[j]

                    # Check if positions are adjacent (Manhattan distance of 1)
                    manhattan_dist = abs(pos_i[0] - pos_j[0]) + abs(pos_i[1] - pos_j[1])

                    if manhattan_dist == 1:
                        # Check if at least one position is not a wall
                        if map[pos_i] != "G" and map[pos_j] != "G":
                            distanceArray[i - 1][j - 1] = 1
                        else:
                            distanceArray[i - 1][j - 1] = float("inf")

        # Floyd-Warshall algorithm
        for k in range(num_nodes):
            for i in range(num_nodes):
                for j in range(num_nodes):
                    # Update the distance to the minimum via an intermediate node
                    distanceArray[i][j] = min(
                        distanceArray[i][j], distanceArray[i][k] + distanceArray[k][j]
                    )

        # Convert the result matrix to a dictionary
        result_dict = {}
        for i in range(1, num_nodes + 1):
            pos_i = nodes[i]
            # Create inner dictionary for each position
            distances_from_i = {}
            for j in range(1, num_nodes + 1):
                pos_j = nodes[j]
                distances_from_i[pos_j] = distanceArray[i - 1][j - 1]
            result_dict[pos_i] = distances_from_i

        return result_dict

    def condition_free(self, values: list[tuple[int, int]]) -> bool:
        """
        Check that all the positions in the list are
        free positions in the map.
        All positions are free if they are not "G" in the map.

        :param Values: List of tuples with the positions to check
        """
        for elem in values:
            if self.map[elem] == "G":
                return False

        return True

    def condition_in_map(self, values: list[tuple[int, int]]) -> bool:
        """
        Check that all the positions in the list are
        inside the map.
        :param Values: List of tuples with the positions to check
        """
        for elem in values:
            if not self.map.get(elem):
                return False
        return True

    def condition_same_position(self, values: list[tuple[int, int]]) -> bool:
        """
        Check that no two values in the list are the same.
        :param Values: List of tuples with the positions to check
        """
        for i in range(len(values)):
            for j in range(i + 1, len(values)):
                if values[i] == values[j]:

                    return False
        return True

    def condition_cross(self, values: list[tuple[int, int]]) -> bool:
        """
        Check that no two planes cross each other.Compare the given list
        of values with the current positions of the planes.
        :param Values: List of tuples with the positions to
        """

        for i in range(len(values)):
            for j in range(len(self.planePositions)):
                if i == j:
                    continue
                if (
                    self.planePositions[j] == values[i]
                    and values[j] == self.planePositions[i]
                ):
                    return False

        return True

    def condition_wait(self, values: list[tuple[int, int]]) -> bool:
        """
        Check that a plane can only wiat if the map position is not "A"
        :param Values: List of tuples with the positions to check
        """
        for i in range(len(values)):
            if (values[i] == self.planePositions[i]) and (self.map[values[i]] == "A"):
                return False
        return True

    def get_child_cost(self, values: list[tuple[int, int]]) -> float:
        """
        Given a list of moves of a plane, get the cost of the state change.
        :param Values: List of tuples with the moves of the planes
        """
        totalCost = 0
        for elem in values:
            if elem == (0, 0):
                totalCost += self.waitCost
            else:
                totalCost += self.moveCost

        return totalCost / len(values)

    def operator_move(
        self,
    ) -> tuple[list[list[tuple[int, int]]], list[list[tuple[int, int]]]]:
        """
        Compute the Cartesian product of a set with itself n times,then
        apply the conditions to get the valid combinations.

        :param input_set: Set of tuples, where each tuple contains two integers
        :param n: Number of times to compute the product
        :return (list): List of lists with the valid combinations (childPositionValues)
        :return (list): List of lists with the moves to get the valid combinations (childMoveValues)

        Preconditions:
        * Adjacency: The next position will be adjacent to the current one
        * Free: Checked usig condition_free()
        * Same Position: Checked using condition_same_position()
        * Cross: Checked using condition_cross()
        * Wait: Checked using condition_wait()

        """
        input_set = self.possibleMoves
        n = len(self.planePositions)
        # Initialize result list
        result = ([], [])

        if n <= 0:
            print_d("ERROR -- Plane positions is empty")
            return result

        # Convert set to sorted list for consistent ordering
        # For tuples, we sort based on both elements of the tuple
        elements = sorted(list(input_set), key=lambda x: (x[0], x[1]))
        setSize = len(elements)

        # Total number of combinations remains the same
        # If we have k tuples and select n times, we get k^n combinations
        totalCombinations = setSize**n

        # Generate each combination
        for i in range(totalCombinations):
            # Current combination
            moves = []
            # List of tuples with the new positions
            positions = []
            temp = i

            # The base conversion process remains the same
            # But now each selected element is a tuple instead of a number
            for _ in range(n):
                moves.append(elements[temp % setSize])
                temp //= setSize

            # Apply moves to current position
            for i in range(len(moves)):
                newPosition = (
                    moves[i][0] + self.planePositions[i][0],
                    moves[i][1] + self.planePositions[i][1],
                )
                positions.append(newPosition)

            # Check conditions:
            if self.condition_in_map(positions):
                conditions = [
                    self.condition_free(positions),
                    self.condition_same_position(positions),
                    self.condition_cross(positions),
                    self.condition_wait(positions),
                ]
                if all(conditions):
                    result[0].append(positions)
                    result[1].append(moves)

        return result

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
