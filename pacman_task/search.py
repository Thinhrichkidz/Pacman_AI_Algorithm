"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import util
from game import Directions
from util import Stack, Queue, PriorityQueue
from problems import SingleFoodSearchProblem

n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # stack = Stack()
    # visited = set()
    # stack.push((problem.getStartState(), []))

    # while not stack.isEmpty():
    #     node, path = stack.pop()
    #     if problem.isGoalState(node):
    #         return path
    #     if node not in visited:
    #         visited.add(node)
    #         for successor, action, cost in problem.getSuccessors(node):
    #             new_path = path + [action]
    #             stack.push((successor, new_path))
    # return None
    # TODO 17
    
    visited = {}
    solution = []
    stack = util.Stack()
    route = {}

    start = problem.getStartState()
    stack.push((start, '', 0))
    visited[start] = ''
    if problem.isGoalState(start):
        return solution

    goal = False
    while not (stack.isEmpty() or goal):
        vertex = stack.pop()
        visited[vertex[0]] = vertex[1]
        if problem.isGoalState(vertex[0]):
            child = vertex[0]
            goal = True
            break
        for i in problem.getSuccessors(vertex[0]):
            if i[0] not in visited.keys():
                route[i[0]] = vertex[0]
                stack.push(i)

    while(child in route.keys()):
        parent = route[child]
        solution.insert(0, visited[child])
        child = parent

    return solution
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''  
    # TODO 18
    
    visited = {}
    solution = []
    queue = util.Queue()
    route = {}
    flag = False

    start = problem.getStartState()
    if problem.isGoalState(start):
      return solution
    queue.push((start, 'None', 0))
    visited[start] = 'None'

    while not (queue.isEmpty() or flag):
      vertex = queue.pop()
      visited[vertex[0]] = vertex[1]
      if problem.isGoalState(vertex[0]):
        child = vertex[0]
        flag = True
        break
      
      for i in problem.getSuccessors(vertex[0]):
        if i[0] not in visited.keys() and i[0] not in route.keys():
          route[i[0]] = vertex[0]
          queue.push(i)
    
    while (child in route.keys()):
      parent = route[child]
      solution.insert(0, visited[child])
      child = parent
    
    return solution
    if problem.isGoalState(start):
      return solution
    util.raiseNotDefined()


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # pq = PriorityQueue()
    # visited = set()
    # pq.push((problem.getStartState(), [], 0), 0)

    # while not pq.isEmpty():
    #     node, path, cost = pq.pop()
    #     if problem.isGoalState(node):
    #         return path
    #     if node not in visited:
    #         visited.add(node)
    #         for successor, action, stepCost in problem.getSuccessors(node):
    #             new_path = path + [action]
    #             new_cost = cost + stepCost
    #             pq.push((successor, new_path, new_cost), new_cost)
    # return None
    # TODO 19
    
    visited = {}
    solution = []
    queue = util.PriorityQueue()
    route = {}
    cost = {}

    start = problem.getStartState()
    queue.push((start, '', 0), 0)
    visited[start] = ''
    cost[start] = 0

    if problem.isGoalState(start):
        return solution

    flag = False
    while not (queue.isEmpty() or flag):
        vertex = queue.pop()
        visited[vertex[0]] = vertex[1]
        if problem.isGoalState(vertex[0]):
            child = vertex[0]
            flag = True
            break
        for i in problem.getSuccessors(vertex[0]):
            if i[0] not in visited.keys():
                priority = vertex[2] + i[2]
                if not(i[0] in cost.keys() and cost[i[0]] <= priority):
                    queue.push((i[0], i[1], vertex[2] + i[2]), priority)
                    cost[i[0]] = priority
                    route[i[0]] = vertex[0]

    while(child in route.keys()):
        parent = route[child]
        solution.insert(0, visited[child])
        child = parent

    return solution


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


# =============================================================================
# def singleFoodSearchHeuristic(state, problem=None):
#     """
#     A heuristic function for the problem of single food search
#     """
#     # TODO 20
#     position, foodGrid = state
#     "*** YOUR CODE HERE ***"
#     heuristic = 0
#     nonvisited = foodGrid.asList()
#     pos = state[0]
#     distances = []
#     
#     if len(nonvisited) == 0:
#         return heuristic
# 
#     for i in nonvisited:
#         for j in nonvisited:
#             d = mazeDistance(i, j, problem.startingGameState)
#             distances.append((d, i, j))
#     distances = sorted(distances, reverse = True)
# 
#     heuristic += distances[0][0]
#     distanceTwo = []
#     distanceTwo.append(mazeDistance(distances[0][1],pos, problem.startingGameState))
#     distanceTwo.append(mazeDistance(distances[0][2],pos, problem.startingGameState))
#     distanceTwo = sorted(distanceTwo)
#     heuristic += distanceTwo[0]
#     
#     return heuristic
# =============================================================================

class SingleFoodSearchHeuristic:
    """
    A heuristic for the problem of single food search
    """

    def __init__(self, problem):
        self.problem = problem

    def __call__(self, state):
        """
        Return the heuristic value for a given state
        """
        position, foodGrid = state
        heuristic = 0
        nonvisited = foodGrid.asList()
        pos = state[0]
        distances = []

        if len(nonvisited) == 0:
            return heuristic

        for i in nonvisited:
            for j in nonvisited:
                d = mazeDistance(i, j, self.problem.startingGameState)
                distances.append((d, i, j))
        distances = sorted(distances, reverse=True)

        heuristic += distances[0][0]
        distanceTwo = []
        distanceTwo.append(mazeDistance(distances[0][1], pos, self.problem.startingGameState))
        distanceTwo.append(mazeDistance(distances[0][2], pos, self.problem.startingGameState))
        distanceTwo = sorted(distanceTwo)
        heuristic += distanceTwo[0]

        return heuristic



def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multiple food search
    """
    position, foodGrid = state
    heuristic = 0
    nonvisited = foodGrid.asList()
    pos = state[0]
    
    if len(nonvisited) == 0:
        return heuristic

    # calculate distance to each goal state
    distances = [mazeDistance(pos, goal, problem.startingGameState) for goal in problem.goal]

    # take the minimum distance as the heuristic value
    heuristic += min(distances)

    return heuristic



def aStarSearch(problem, heuristic=nullHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22
    visited = {}
    solution = []
    queue = util.PriorityQueue()
    route = {}
    cost = {}

    start = problem.getStartState()
    queue.push((start, '', 0), 0)
    visited[start] = ''
    cost[start] = 0

    if problem.isGoalState(start):
        return solution

    flag = False
    while not (queue.isEmpty() or flag):
        vertex = queue.pop()
        visited[vertex[0]] = vertex[1]
        if problem.isGoalState(vertex[0]):
            child = vertex[0]
            flag = True
            break
        for i in problem.getSuccessors(vertex[0]):
            if i[0] not in visited.keys():
                #priority = vertex[2] + i[2] + heuristic(i[0], problem)
                priority = vertex[2] + i[2] + heuristic(i[0])

                if not(i[0] in cost.keys() and cost[i[0]] <= priority):
                    queue.push((i[0], i[1], vertex[2] + i[2]), priority)
                    cost[i[0]] = priority
                    route[i[0]] = vertex[0]

    while(child in route.keys()):
        parents = route[child]
        solution.insert(0, visited[child])
        child = parents

    return solution

def mazeDistance(point1, point2, gameState):
    """
    Returns the maze distance between any two points, using the search functions
    you have already built. The gameState can be any game state -- Pacman's
    position in that state is ignored.
    Example usage: mazeDistance( (2,4), (5,6), gameState)
    This might be a useful helper function for your ApproximateSearchAgent.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = SingleFoodSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(breadthFirstSearch(prob))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
