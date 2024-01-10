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
        action, stepCost), where 'successor' is a successor to the current
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
    return  [s, s, w, s, w, w, s, w]


def helper_depth_first(problem,stacks,visited,actions,dictionary,dict2,count):
    count = count + 1
    # pop off from stacks and into visited
    current = stacks.pop()
    if count != 1:
        visited.append(current[0])
        actions.append(current[1])
    else:
        visited.append(current)

    # is it goal or not
    if count != 1 and problem.isGoalState(current[0]) == True:
        #adjacent_nodes = problem.getSuccessors(current[0])
        #adjacent_visited = 'done'
        #for i in adjacent_nodes:
            #if i[0] not in visited:
                #stacks.push(i)
                #dict2[i[0]] = current[0]
            #else:
                #adjacent_visited = i
  

        dictionary[current[0]] = current[1]
        #dict2[current[0]] = adjacent_visited[0]
        return current

    # put adjacent nodes into stack
    if count != 1:
        adjacent_nodes = problem.getSuccessors(current[0])
        adjacent_visited = 'done'
        for i in adjacent_nodes:
            if i[0] not in visited:
                stacks.push(i)
                dict2[i[0]] = current[0]
            else:
                adjacent_visited = i

     

        dictionary[current[0]] = current[1]
        #dict2[current[0]] = adjacent_visited[0]

    else:
        adjacent_nodes = problem.getSuccessors(current)
        adjacent_visited = 'done'
        for i in adjacent_nodes:
            if i[0] not in visited:
                stacks.push(i)
                dict2[i[0]] = current
            else:
                adjacent_visited = i

        dictionary[current] = adjacent_visited
        #dict2[current] = adjacent_visited

    return helper_depth_first(problem,stacks,visited,actions,dictionary,dict2,count)

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    visited = []
    dictionary = {}
    dict2 = {}
    stacks = util.Stack()
    actions = []
    #add start state to dict2
    dict2[problem.getStartState()] = 'done'
    # add start state to visited
    stacks.push(problem.getStartState())
    a = helper_depth_first(problem,stacks,visited,actions,dictionary,dict2,0)[0]

    print(dict2)
    print(dictionary)
    print(a)
    print(visited)

    actions = []
    while a != 'done':
        actions = [dictionary[a]] + actions
        a = dict2[a]

    actions.pop(0)
    print(actions)

    return actions
    util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    # instantiate visited and queue and dictionaries
    listings = []
    visited = []
    queue = util.Queue()
    dictionary = {}
    dict2 = {}
    queue2 = util.Queue()
    # add start state to dict2
    dict2[problem.getStartState()] = 'done'
    # first node is visited
    current = problem.getStartState()
    queue.push(current)
    queue2.push(current)
    # put in helper
    result = bread_helper(problem,visited,queue,dictionary,dict2,0,queue2,listings)[0]

 


    actions = []

    while result != 'done':
        actions = [dictionary[result]] + actions
        result = dict2[result]

    actions.pop(0)

    return actions

def bread_helper(problem,visited,queue,dictionary,dict2,count,queue2,listing):
    count = count + 1
    print(queue.list)
    # pop from queue and mark as visited
    current = queue.pop()
    queue2.pop()
    if count != 1:
        visited.append(current[0])
    else:
        visited.append(current)
    # check if current node is the goal state
    if count != 1 and problem.isGoalState(current[0]) == True:
        #adjacent_nodes = problem.getSuccessors(current[0])
        #adjacent_visited = 'done'
        #for i in adjacent_nodes:
            #if i[0] not in visited:
               # queue.push(i)
              #  dict2[i[0]] = current[0]
           # else:
             #   adjacent_visited = i

        dictionary[current[0]] = current[1]
        #dict2[current[0]] = adjacent_visited[0]
        return current
    # add adjacent nodes into queue
    if count != 1:
        adjacent_nodes = problem.getSuccessors(current[0])
        adjacent_visited = 'done'
        for i in adjacent_nodes:
            if i[0] not in visited:
                if i[0] not in queue2.list:
                    queue.push(i)
                    queue2.push(i[0])
                    listing.append(i[0])
                    dict2[i[0]] = current[0]
                
            else:
                adjacent_visited = i

        dictionary[current[0]] = current[1]
        #dict2[current[0]] = adjacent_visited[0]

    else:
        adjacent_nodes = problem.getSuccessors(current)
        
        adjacent_visited = 'done'
        for i in adjacent_nodes:
            if i[0] not in visited:
                if i[0] not in queue2.list:
                    queue.push(i)
                    queue2.push(i[0])
                    listing.append(i[0])
                    dict2[i[0]] = current
                
            else:
                adjacent_visited = i

        dictionary[current] = adjacent_visited
        #dict2[current] = adjacent_visited

    return bread_helper(problem,visited,queue,dictionary,dict2,count,queue2,listing)

def uniformCostSearch(problem: SearchProblem):
    # we need dictionary to current, we need priority queue full, we need visited current nodes not full path
    dictionary = {}
    dict2 = {}
    dict3 = {}
    pq = util.PriorityQueue()
    visited = []
    count = 0

    # queue start state
    current = problem.getStartState()
    pq.push(current,0)
    dict2[current] = 0
    dict3[current] = 'done'
    dictionary[current] = 'done'

    result = uniform_helper(problem,dictionary,pq,visited,count,dict2,dict3)



    actions = []
    while result != 'done':
        actions = [dict3[result]] + actions
        result = dictionary[result]

    actions.pop(0)
    print(visited)
    return actions

    print(actions)

    return None

def uniform_helper(problem,dictionary,pq,visited,count,dict2,dict3):
    print(pq.heap)
    count = count + 1
    # increase count by one so that we can detect start state
    current = pq.pop()

    if count != 1:
        if current[0] in visited:
            return uniform_helper(problem,dictionary,pq,visited,count,dict2,dict3)
        visited.append(current[0])
    else:
        visited.append(current)

    # check if goal
    if count != 1 and problem.isGoalState(current[0]) == True:

        return current

    # add nodes into fringe
    if count != 1:
        adjacent_nodes = problem.getSuccessors(current[0])
        for i in adjacent_nodes:
            if i[0] not in visited:
                pq.push(i,i[2] + dict2[current]) # keep track of priority
                dictionary[i] = current # keep track of parent.
                dict2[i] = i[2] + dict2[current] # keep track of priority value
                dict3[i] = i[1] 

    else:
        adjacent_nodes = problem.getSuccessors(current)
        for i in adjacent_nodes:
            if i[0] not in visited:
                pq.push(i,i[2] + dict2[current])
                dictionary[i] = current
                dict2[i] = i[2] + dict2[current]
                dict3[i] = i[1]



    



    return uniform_helper(problem,dictionary,pq,visited,count,dict2,dict3)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def cost_path(prev,now):
    return prev + now

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    print(problem.getStartState())
    print(heuristic)

    # first steps: get start node to adjacent nodes

    # get adjacent nodes first:
    priority_que_for_prev_cost = util.PriorityQueue()
    visited_already = []
    dict_mapping = {}
    listing = []
    priority_for_list = util.PriorityQueue()
    current = problem.getStartState()
    list_adjacent = problem.getSuccessors(current)
    pq = util.PriorityQueue()

    visited_already.append(current)

    for i in list_adjacent: # heuristic = heuristic(adjacent) + cost path
        cost_path_now = cost_path(0,i[2])
        heuristics_now = cost_path_now + heuristic(i[0],problem)
        pq.push(i,heuristics_now)
        dict_mapping[i] = current

        listing.append(i)
        priority_for_list.push(listing,heuristics_now)
        listing = []

        # pq for total cost
        priority_que_for_prev_cost.push(cost_path_now,heuristics_now)







    # second step for successors, pop out lowest priority
    while not pq.isEmpty():
        current = pq.pop()
        listing = priority_for_list.pop()
        current_costing = priority_que_for_prev_cost.pop()
        if current[0] in visited_already:
            continue

        visited_already.append(current[0])
        if problem.isGoalState(current[0]) == True:
            print(listing)
            result = []
            for i in listing:
                result.append(i[1])

            return result

        list_adjacent = problem.getSuccessors(current[0])
        for i in list_adjacent:
            current_listing = listing.copy()
            cost_path_this = cost_path(current_costing,i[2])
            heuristics_now = cost_path_this + heuristic(i[0],problem)
            pq.push(i,heuristics_now)
            dict_mapping[i] = current

            current_listing.append(i)
            priority_for_list.push(current_listing,heuristics_now)

            # pq for total cost
            priority_que_for_prev_cost.push(cost_path_this,heuristics_now)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
