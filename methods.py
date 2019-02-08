# PA1
# Name: Xiaoyang Zeng
# PID: A14073155
# Email: xiz460@ucsd.edu
#
# Always use PYTHON 2 to run the program.
#
# ***************************************************************************#

from __future__ import print_function
#Use priority queues from Python libraries, don't waste time implementing your own
from heapq import *

# import queue for BFS
from Queue import *

ACTIONS = [(0,-1),(-1,0),(0,1),(1,0)]
# weight of heuristic function where F = G + ALPHA * H
ALPHA = 100

class Agent:
    def __init__(self, grid, start, goal, type):
        self.grid = grid
        self.previous = {}
        self.explored = []
        self.start = start 
        self.grid.nodes[start].start = True
        self.goal = goal
        self.grid.nodes[goal].goal = True
        self.new_plan(type)
        # manhattan distance from start to goal
        self.manhattan_start_goal = 0
    def new_plan(self, type):
        self.finished = False
        self.failed = False
        self.type = type
        if self.type == "dfs" :
            self.frontier = [self.start]
            self.explored = []
        elif self.type == "bfs":
            #set up base case
            self.queue = Queue(maxsize=0)
            self.queue.put(self.start)
            # a control array that make queue element iretable
            self.control = [self.start]
            self.explored = []

        elif self.type == "ucs":
            # set up variables 
            path_cost = 0
            self.pq = [ ]
            # push base case on the priority queue
            heappush(self.pq, (path_cost, self.start))
            # use an array store visted nodes
            self.explored = []
        elif self.type == "astar":
            # calculate the manhattan distance for heuristic function of start node
            self.manhattan_start_goal = abs(self.start[0] - self.goal[0]) + abs(self.start[1] - self.goal[1])
            path_cost = 0 + ALPHA*self.manhattan_start_goal

            # do exact same thing that we did for UCS
            self.pq = [ ]
            # push base case on the priority queue
            heappush(self.pq, (path_cost, self.start))
            # use an array store visted nodes
            self.explored = []

    def show_result(self):
        current = self.goal
        while not current == self.start:
            current = self.previous[current]
            self.grid.nodes[current].in_path = True #This turns the color of the node to red
    def make_step(self):
        if self.type == "dfs":
            self.dfs_step()
        elif self.type == "bfs":
            self.bfs_step()
        elif self.type == "ucs":
            self.ucs_step()
        elif self.type == "astar":
            self.astar_step()

    def dfs_step(self):
        # check whether there is a valid path. set failed to true if there is no.
        if not self.frontier:
            self.failed = True
            print("no path")
            return
        current = self.frontier.pop()
        print("current node: ", current)

        #set up children nodes that we want to explore next
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]

        #traverse through adjacent children and find next node to be explored
        for node in children:
            #See what happens if you disable this check here
            if node in self.explored or node in self.frontier:
                print("explored before: ", node)
                continue

            #if target node was not visited before, first check whether it is in grid range
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                #check whether target node is an "obstacle"
                if self.grid.nodes[node].puddle:
                    print("puddle at: ", node)
                else:
                    #set target node's parent after we know that it is a valid one
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        return
                    else:
                        # add target to the path
                        self.frontier.append(node)
                        # set the node to the frontier
                        self.grid.nodes[node].frontier = True
            else:
                print("out of range: ", node)


    def bfs_step(self):
        # edge case check: If there is nothing in frontier and 
        # we haven't found goal, return false
        if not self.queue:
            self.failed = True
            print("no path")
            return

        # otherwise do BFS
        current = self.queue.get()
        print("current node: ", current)

        # set up children and do recursive step
        self.grid.nodes[current].checked = True
        self.grid.nodes[current].frontier = False
        self.explored.append(current)
        children = [(current[0]+a[0], current[1]+a[1]) for a in ACTIONS]

        # traverse through the children array to find node explore next
        for node in children:
            # check whether adjacent nodes have already been visited
            if node in self.explored or node in self.control:
                print("Visted before: ", node)
                continue
            
            #if target node was not visited before, first check whether it is in grid range
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                #check whether target node is an "obstacle"
                if self.grid.nodes[node].puddle:
                    print("puddle at: ", node)
                else:
                    #set target node's parent after we know that it is a valid one
                    self.previous[node] = current
                    if node == self.goal:
                        self.finished = True
                        return
                    else:
                        # add target to the path and to the array that keep record of 
                        # explored nodes
                        self.queue.put(node)
                        self.control.append(node)
                        # set the node to the frontier
                        self.grid.nodes[node].frontier = True
            else:
                print("out of range: ", node)

    def ucs_step(self):
        # edge case check: If there is nothing in frontier and 
        # we haven't found goal, return false
        if not self.pq:
            self.failed = True
            print("no path")
            return

        # otherwise do UCS
        # first get the front node of frontier
        current = heappop(self.pq)

        # set up children and do recursive step
        self.grid.nodes[current[1]].checked = True
        self.grid.nodes[current[1]].frontier = False
        self.explored.append(current[1])
        children = [(current[1][0]+a[0], current[1][1]+a[1]) for a in ACTIONS]

        # traverse through the children array to find node explore next
        for node in children:

            #if target node was not visited before, first check whether it is in grid range
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                # pre-calculate the cost of path to the current node
                node_cost = self.grid.nodes[node].cost() + current[0]

                # check whether current node is in frontier (self.pq)
                frontier = [candidate for candidate in self.pq if candidate[1] == node]
                #check whether target node is an "obstacle"
                if self.grid.nodes[node].puddle:
                    continue
                # if not puddle, we then have 3 situation
                else:
                    # return true if current node is the goal we find
                    if node == self.goal:
                        self.previous[node] = current[1]
                        self.finished = True
                        print("Total cost of path: " + str(node_cost))
                        return

                    # add the current node to pq directly if it is not visted before and not in frontier
                    if not node in self.explored and not frontier:
                        self.previous[node] = current[1]
                        heappush(self.pq, (node_cost,node))
                        # set the node to the frontier
                        self.grid.nodes[node].frontier = True

                    # if node is explored before, compare the cost and modify it if neccessary
                    elif frontier:
                        temp = self.pq[self.pq.index((frontier[0][0],node))]
                        # if existed node has higher cost, reset the node cost
                        if temp[0] > node_cost:
                            self.pq.remove(temp)
                            heappush(self.pq, (node_cost,node))
                            self.previous[node] = current[1]
                            
    def astar_step(self):
        # edge case check: If there is nothing in frontier and 
        # we haven't found goal, return false
        if not self.pq:
            self.failed = True
            print("no path")
            return

        # otherwise do A*
        # first get the front node of frontier
        current = heappop(self.pq)

        # set up children and do recursive step
        self.grid.nodes[current[1]].checked = True
        self.grid.nodes[current[1]].frontier = False
        self.explored.append(current[1])
        children = [(current[1][0]+a[0], current[1][1]+a[1]) for a in ACTIONS]
        # pre-calculate the manhattan distance of current node
        manhattan_dis_current = abs(current[1][0] - self.goal[0]) + abs(current[1][1] - self.goal[1])

        # traverse through the children array to find node explore next
        for node in children:

            #if target node was not visited before, first check whether it is in grid range
            if node[0] in range(self.grid.row_range) and node[1] in range(self.grid.col_range):
                # pre-calculate the manhattan distance
                manhattan_dis = abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])

                # pre-calculate the cost of path to the current node
                node_cost = self.grid.nodes[node].cost() + current[0] - ALPHA*manhattan_dis_current

                # get the value function for A*: F = G + H
                node_GH = node_cost + ALPHA*manhattan_dis
                # check whether current node is in frontier (self.pq)
                frontier = [candidate for candidate in self.pq if candidate[1] == node]

                #check whether target node is an "obstacle"
                if self.grid.nodes[node].puddle:
                    continue
                # if not puddle, we then have 3 situation
                else:
                    # return true if current node is the goal we find
                    if node == self.goal:
                        self.previous[node] = current[1]
                        self.finished = True
                        print("Total cost of path: " + str(node_cost))
                        return

                    # add the current node to pq directly if it is not visted before
                    if not node in self.explored and not frontier:
                        self.previous[node] = current[1]
                        heappush(self.pq, (node_GH,node))
                        # set the node to the frontier
                        self.grid.nodes[node].frontier = True

                    # if node is explored before, compare the cost and modify it if neccessary
                    elif frontier:
                        temp = self.pq[self.pq.index((frontier[0][0],node))]
                        # if existed node has higher cost, reset the node cost
                        if temp[0] > node_GH:
                            self.pq.remove(temp)
                            heappush(self.pq, (node_GH,node))
                            self.previous[node] = current[1]
