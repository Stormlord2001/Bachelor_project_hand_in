import numpy as np
import cv2 
import keyboard
import math
from TelloClass import *



class Pathfinder():
    def __init__(self, dictionary):
        self.marker_dictionary = dictionary

    # Return the angle between the drone and the target
    def calculate_heading(self, target_node_in_drone_frame):
        drone_to_target = target_node_in_drone_frame[0:2] / np.linalg.norm(target_node_in_drone_frame[0:2])
        drone_vector = [0, 1]

        return np.degrees(np.math.atan2(np.linalg.det([drone_vector,drone_to_target]),np.dot(drone_vector,drone_to_target)))
    
    # Return the target in world coordinates
    def calculate_target_pose(self, node_id, dist, height):
        new_target = self.marker_dictionary[node_id].copy()
        new_target[0] = new_target[0]-np.sin(new_target[3])*dist
        new_target[1] = new_target[1]+np.cos(new_target[3])*dist
        new_target[2] = height
        if new_target[3] < 0:
            new_target[3] = new_target[3] + np.pi
        else:
            new_target[3] = new_target[3] - np.pi
        return new_target
    
    # Return the distance between the drone and the target
    def dist_to_target(self, world_to_drone_translation, target_marker):
        return math.dist(world_to_drone_translation[0:2], self.marker_dictionary[target_marker][0:2])
        

class Graph():
    def __init__(self, connections, marker_dict):
        self.graph = []
        self.marker_dict = marker_dict
        n = int(cv2.minMaxLoc(connections)[1]) + 1
        self.adjacency_matrix = np.zeros((n,n))
        self.add_connections(connections)
        self.prevNode = None
        self.V = n

    def add_connections(self, connections):
        for node1, node2 in connections:
            cost = self.calc_cost(node1, node2)
            self.graph.append((node1, node2, cost))
            self.adjacency_matrix[node1, node2] = cost
            self.adjacency_matrix[node2, node1] = cost
    
    def calc_cost(self, node1, node2):
        return math.dist(self.marker_dict[node1][0:3], self.marker_dict[node2][0:3])

    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minDistance(self, dist, sptSet):
 
        # Initialize minimum distance for next node
        min = 1e7
        min_index = -1       
 
        # Search not nearest vertex not in the
        # shortest path tree
        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v
 
        return min_index

    def dijkstra(self, goal):

        dist = [1e7] * self.V
        dist[goal] = 0
        sptSet = [False] * self.V
        prevNode = [0] * self.V
        prevNode[goal] = -1
 
        for _ in range(self.V):
 
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.minDistance(dist, sptSet)
 
            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[u] = True
 
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex is not in the shortest path tree
            for v in range(self.V):
                if (self.adjacency_matrix[u][v] > 0 and
                   sptSet[v] == False and
                   dist[v] > dist[u] + self.adjacency_matrix[u][v]):
                    dist[v] = dist[u] + self.adjacency_matrix[u][v]
                    prevNode[v] = u
 
        self.prevNode = prevNode

    def get_next_node(self, current_node):
        return self.prevNode[current_node]


