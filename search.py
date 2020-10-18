# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor_node to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    The graph search algorithm needs to return a list of actions that reaches the
    goal.

    Run the following commands to understand the search problem that is being passed in:
    NOTE: Comment out these lines otherwise the autograder will give wrong result for "Search Nodes Expanded"
"""
    # print "Start:", problem.getStartState()
    # print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    # print "Start's successors:", problem.getSuccessors(problem.getStartState())

    frontier = util.Stack()
    startLocation = problem.getStartState()
    startNode = (startLocation, [])  # (location, path-list)
    frontier.push(startNode)
    visited_locations = set()  # HashTable with O(1)

    while not frontier.isEmpty():
        node = frontier.pop()
        location = node[0]
        path = node[1]

        if problem.isGoalState(location):  # When a goal is reached, return the path which reaches it.
            return path

        visited_locations.add(location)

        for successor_node in problem.getSuccessors(location):
            successor_location = successor_node[0]
            if successor_location not in visited_locations:
                successor_path = [successor_node[1]]  # Include it inside "[, ]" in order to convert the string into a list.
                frontier.push((successor_location, (path + successor_path)))  # Pushing (successor location, full path to node) to the border.

    return None


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    frontier = util.Queue()  # Frontier (Queue) to store nodes with their paths.
    startLocation = problem.getStartState()
    startNode = (startLocation, [])  # (location, path)
    frontier.push(startNode)
    visited_locations = set()  # A set to maintain all the visited locations

    while not frontier.isEmpty():
        node = frontier.pop()
        location = node[0]
        path = node[1]

        if problem.isGoalState(location):  # When a goal is reached, return the path which reaches it.
            return path

        if location not in visited_locations:  # Skipping already visited nodes
            visited_locations.add(location)  # Adding newly encountered nodes to the set of visited nodes
            for successor_node in problem.getSuccessors(location):
                successor_location = successor_node[0]
                successor_path = [successor_node[1]]  # Include it inside "[, ]" in order to convert the string into a list.
                frontier.push((successor_location, (path + successor_path)))  # Pushing (successor location, full path to node) to the border.

    return None


def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    frontier = util.PriorityQueue()  # Frontier (Priority Queue) to store nodes with their paths.
    startLocation = problem.getStartState()
    startNode = (startLocation, [], 0)  # (location, path, cost)
    frontier.push(startNode, 0)  # startNode, priority-cost

    visited_locations = set()  # A set to maintain all the visited locations

    while not frontier.isEmpty():
        node = frontier.pop()
        location = node[0]
        path = node[1]
        cost = node[2]

        if problem.isGoalState(location):  # When a goal is reached, return the path which reaches it.
            return path

        if location not in visited_locations:  # Skipping already visited nodes
            visited_locations.add(location)  # Adding newly encountered nodes to the set of visited nodes

            for successor_node in problem.getSuccessors(location):
                successor_location = successor_node[0]
                successor_path = [successor_node[1]]  # Include it inside "[, ]" in order to convert the string into a list.
                successor_cost = successor_node[2]

                full_path = path + successor_path  # Computing path of successor_node from start node
                full_cost = cost + successor_cost  # Computing cumulative backward cost of successor_node node from start node
                frontier.push((successor_location, full_path, full_cost), full_cost)
                # Pushing ((successor location, full path to node, cumulative cost), cumulative cost) to the frontier.

    return None


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    frontier = util.PriorityQueue()  # Fringe (Priority Queue) to store nodes with their paths
    startLocation = problem.getStartState()
    startNode = (startLocation, [], 0)  # (location, path, cost)
    priority = heuristic(problem.getStartState(), problem) + 0
    frontier.push(startNode, priority)  # startNode, priority-cost

    visited_nodes = set()  # A set to maintain all the visited nodes

    while not frontier.isEmpty():
        node = frontier.pop()
        location = node[0]
        path = node[1]
        cost = node[2]

        if problem.isGoalState(location):  # When a goal is reached, return the path which reaches it.
            return path

        if location not in visited_nodes:  # Skipping already visited nodes
            visited_nodes.add(location)  # Adding newly encountered nodes to the set of visited nodes

            for successor_node in problem.getSuccessors(location):
                successor_location = successor_node[0]
                successor_path = successor_node[1]
                successor_cost = successor_node[2]

                full_path = path + [successor_path]  # Computing path of successor_node node from start node
                full_cost = cost + successor_cost  # Computing culmulative backward cost of successor_node node from start node
                frontier.push((successor_location, full_path, full_cost), full_cost + heuristic(successor_location, problem))
                # Pushing ((successor location, full path to node, cumulative cost), sum of cumulative cost to node with the heuristic cost) to the frontier.

    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
