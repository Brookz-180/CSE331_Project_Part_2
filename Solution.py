from Traversals import bfs_path
import queue
import heapq
from collections import deque
from Simulator import Simulator
import sys


class Solution:

    def __init__(self, problem, isp, graph, info):
        self.problem = problem
        self.isp = isp
        self.graph = graph
        self.info = info

    def output_paths(self):
        """
        This method must be filled in by you. You may add other methods and subclasses as you see fit,
        but they must remain within the Solution class.
        """
        paths, bandwidths, priorities = {}, {}, {}
        bfs = {}
        #print("number of clients: ",len(self.info["list_clients"]))
        bfs_paths = bfs_path(self.graph, self.isp, self.info["list_clients"])
        for client in self.info["list_clients"]:
            bfs[client] = bfs_paths[client]
        i=0
        for client in self.info["list_clients"]:
            i+=1
            optimalpath = self.dijkstra(client,self.isp,self.graph,self.info,bfs)
            paths[client] = optimalpath
            #print(len(paths))
            #print("current iteration: ",i)

        # Note: You do not need to modify all of the above. For Problem 1, only the paths variable needs to be modified. If you do modify a variable you are not supposed to, you might notice different revenues outputted by the Driver locally since the autograder will ignore the variables not relevant for the problem.
        # WARNING: DO NOT MODIFY THE LINE BELOW, OR BAD THINGS WILL HAPPEN
        return (paths, bandwidths, priorities)

    def dijkstra(self, client, isp, graph, info, bfs):
        dist = {node: float('inf') for node in graph} # set initial distances to infinity
        dist[isp] = 0 # distance to start node is 0
        pq = queue.PriorityQueue()
        pq.put((0, isp))
        previous_node = {}
        bandwidths = info["bandwidths"].copy() # copy the bandwidths
        visited = set()
        while not pq.empty():
            current_distance, current_node = pq.get()
            #print(current_distance)
            #print("current node is: ",current_node)
            if current_node in info["list_clients"]:
                if current_distance > info["alphas"][client]*len(bfs[client]): # check the tolerance constraint for the node
                    continue
            visited.add(current_node)
            # explore the neighbors
            for neighbor in graph[current_node]:
                if current_node in bandwidths and bandwidths[current_node] > 0: # check bandwidth constraint
                    # find the potential new distance of the neighbor node
                    newdist = current_distance + 1
                    if newdist < dist[neighbor] and newdist <= info["alphas"][client]*len(bfs[client]):         #self.shortestpath(graph, info, client, isp):
                        dist[neighbor] = newdist
                        previous_node[neighbor] = current_node
                        pq.put((newdist, neighbor))
                        bandwidths[current_node] -= 1 # update the bandwidth
                else:
                    continue
        path = [client]
        current_node = client
        while current_node in previous_node:
            current_node = previous_node[current_node]
            path.append(current_node)
        path.append(isp)
        return list(reversed(path))

    def shortestpath(self, graph, info, client, isp):
        shortest = float('inf')
        for clientnode in info["list_clients"]:
            if clientnode == client:
                continue
            path = bfs_path(graph, isp, [clientnode])
            shortest = min(shortest, len(path)-1)
        return shortest

        #     visited.append(current_node)
        #     for neighbor in graph[current_node]:
        #         tol = info["alphas"][client]
        #         if current_distance <= tol and current_node not in visited:
        #             dist[neighbor] = current_distance
        #             previous_node[neighbor] = current_node
        #             current_distance += 1
        #             pq.put((current_distance,neighbor))
        #         else:
        #             continue

    # def createoptimalpath(self,client):
    #     # need to modify the graph based on bandwidth constraints
    #     newgraph = self.limitbandwidth(client)
    #     newgraph[self.isp] = self.graph[self.isp]
    #     # do BFS on the new graph
    #     optimalpath = bfs_path(newgraph,self.isp,self.info["list_clients"])
    #     #print(optimalpath)
    #     #optimalpath[client] = [self.isp] + optimalpath[client]
    #     return optimalpath[client]
    #
    # def limitbandwidth(self,client):
    #     # this function creates the new graph with the limited bandwidths
    #     newgraph = {node: [] for node in self.graph} # create a new graph
    #     for node in self.graph:
    #         if node != self.isp:
    #             newband = self.updatebandwidth(node,client) # calculate the updated bandwidth for each node
    #             newgraph[node] = self.graph[node] # use the same adjacency list as original graph
    #             for neighbor in newgraph[node]:
    #                 if newband < self.info["alphas"][client]:
    #                     newgraph[node].remove(neighbor)
    #     return newgraph
    #
    # def updatebandwidth(self,router,client):
    #     # this function will use the router's bandwidth and client's tolerance values to find the available bandwidth
    #     band = self.info["bandwidths"][router]/self.info["alphas"][client]
    #     return band
