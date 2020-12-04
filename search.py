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

def depthFirstSearch(problem):
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
    stack = util.Stack()#usestack for dfs

    visited = [] # visited states
    path = [] #final path we found

    #add start state and empty path to stack
    stack.push((problem.getStartState(),[]))

    while stack.isEmpty()==False:
        
        my_position,path = stack.pop() # Take position and path
        visited.append(my_position)#add postion to visited

        # Terminate when reach goal
        if problem.isGoalState(my_position)==True:
            return path

        # Get successors 
        successor = problem.getSuccessors(my_position)

        # Add new states and direction 
        for next in successor:
            state=next[0]
            direction=next[1]
            #check if visited
            if state not in visited:
                updatePath = path + [direction] #update path
                stack.push((state, updatePath))

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue() #bfs use queue

    visited = [] # visited states
    path = [] # final path we found
    position=[]# all positions before visited update

    # add start state and empty path to queue
    queue.push((problem.getStartState(),[]))

    while queue.isEmpty()==False:
        
        my_position,path = queue.pop() # position and path
        visited.append(my_position)#add  position to visited 

        # Terminate when reach the goal
        if problem.isGoalState(my_position)==True:
            return path

        # Get successors 
        successor = problem.getSuccessors(my_position)

        #all position before
        for item in queue.list:
            position.append(item[0])
            
        # Add new states and direction
        for next in successor:
            state=next[0]
            direction=next[1]
            #check if visited
            if state not in visited and state not in position:
                updatePath = path + [direction] # update path
                queue.push((state, updatePath))


    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    pri_queue = util.PriorityQueue()#use priority queue

    visited = [] # visited states
    path = [] # final path we found
    position=[]

    #add start state empty path and cost to the priority queue                                  
    pri_queue.push((problem.getStartState(),[]),0)

    while pri_queue.isEmpty()==False:

        my_position,path = pri_queue.pop() #position and path
        visited.append(my_position)

        # Terminate when reach the goal
        if problem.isGoalState(my_position)==True:
            return path

        # Get successors 
        successor = problem.getSuccessors(my_position)

        for item in pri_queue.heap:
            position.append(item[2][0])#all postions in priority queue
        
        # Add new states and direction
        for next in successor:
            state=next[0]
            direction=next[1]
            #check if visited or not
            if state not in visited and state not in position:
                updatePath = path + [direction]
                cost = problem.getCostOfActions(updatePath)
                pri_queue.push((state,updatePath),cost)

            #if visited, compare the costs
            elif state not in visited and state in position:
                for item in pri_queue.heap:
                    if item[2][0] == state:#find the visited 
                        cost1 = problem.getCostOfActions(item[2][1])#get the cost of the path

                cost2 = problem.getCostOfActions(path + [direction])
                if cost2<cost1:
                    updatePath = path + [direction]
                    pri_queue.update((state,updatePath),cost2)

    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    pri_queue = util.PriorityQueue()#use priority queue

    visited = [] # visited states
    path = [] # final path we found

    #add start state empty path and cost to the priority queue 
    h=heuristic(problem.getStartState(),problem)                                
    pri_queue.push((problem.getStartState(),[]),h)

    while pri_queue.isEmpty()==False:

        my_position,path = pri_queue.pop() #position and path


        if my_position in visited:
            continue
        visited.append(my_position)

        # Terminate when reach the goal
        if problem.isGoalState(my_position)==True:
            return path

        # Get successors 
        successor = problem.getSuccessors(my_position)

        
        # Add new states and direction
        for next in successor:
            state=next[0]
            direction=next[1]
            #check if visited or not
            if state not in visited:
                updatePath = path + [direction] #update path
                cost = problem.getCostOfActions(updatePath)+heuristic(state,problem) #f=g+h
                pri_queue.push((state,updatePath),cost)

    

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
