import math
import random
import threading
from typing import List, Any

import pygame


class Color:
    WHITE = (255, 255, 255)
    GREY = (70, 70, 70)
    BLUE = (0, 0, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    GREY2 = (50,50,50)
    PURPLE = (199,21,133)
    BROWN = (210,105,30)
    LIGHT_BLUE = (176,196,222)

def dist_to_node(n1, n2):
    return dist(n1.get_coords(), n2.get_coords())


def dist_to_point(n, p):
    return dist(n.get_coords(), p)


def dist(p1, p2):
    x, y = p1[0], p1[1]
    xx, yy = p2[0], p2[1]
    return math.hypot(x - xx, y - yy)


def add_edge(n1, n2):
    n1.add_neighbour(n2)
    n2.add_neighbour(n1)


def remove_edge(n1, n2):
    del n1.adj[n2]
    del n1.edge[n2]
    del n2.adj[n1]
    del n2.edge[n1]


class Node:
    def __init__(self, x, y, id):
        self.x = x
        self.y = y
        self.id = id
        self.parent = None
        self.search = None
        self.adj = {}
        self.edge = {}

    def get_coords(self):
        return self.x, self.y

    def add_neighbour(self, neighbour):
        self.adj[neighbour] = self.__euclidean_dist(neighbour)
        self.edge[neighbour] = NodeEdge(self, neighbour)

    def __euclidean_dist(self, neighbour):
        return math.hypot((self.x - neighbour.x), (self.y - neighbour.y))

    def get_connections(self):
        return self.adj.keys()

    def get_weight(self, neighbour):
        return self.adj[neighbour]

    def draw(self, surf, node_radius, width):
        pygame.draw.circle(surf, Color.RED, self.get_coords(), node_radius, width=0)
        for neighbour in self.edge:
            color = Color.GREY
            if neighbour.search == "Dijkstra":
                color = Color.BLUE
            if neighbour.search == "AStar":
                color = Color.RED
            if neighbour.search == "GreedyBFS":
                color = Color.LIGHT_BLUE
            pygame.draw.line(surf, color, self.edge[neighbour].nfrom.get_coords(), self.edge[neighbour].nto.get_coords(), width=width)

    def __str__(self):
        return f"{self.x}, {self.y}, {self.id}"


# Used to visualize pathfinding
class NodeEdge:
    def __init__(self, node_from: Node, node_to: Node):
        self.nfrom = node_from
        self.nto = node_to


class ProbabilisticRoadmap:
    nodes: List[Node]

    def __init__(self, map_dim, start_pose, start_radius, goal_pose, goal_radius, obstacles, k):
        self.map_dim = self.mapx, self.mapy = map_dim
        self.start_pose = self.sx, self.sy = start_pose
        self.start_radius = start_radius
        self.goal_pose = self.gx, self.gy = goal_pose
        self.goal_radius = goal_radius
        self.obstacles = obstacles
        self.k = k
        self.nodes = []
        self.edges = {}
        self.lock = threading.Lock()

    def sample(self, sample_size):
        self.nodes = []
        self.add_node(0, *self.start_pose)
        for i in range(sample_size):
            n = len(self.nodes)
            collision = True
            x, y = (-1, -1)
            while collision:
                x, y = self.sample_envir()
                collision = self.on_obstacle((x, y))
            self.add_node(n, x, y)
        return self.nodes

    def sample_envir(self):
        x = int(random.uniform(0, self.mapx))
        y = int(random.uniform(0, self.mapy))
        return x, y

    def on_obstacle(self, point):
        for obs in self.obstacles:
            if obs.collidepoint(point):
                return True
        return False

    def add_node(self, n, x, y):
        self.nodes.insert(n, Node(x, y, n))

    def remove_node(self, n):
        self.nodes.pop(n)

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

    def cross_obstacle(self, start_pos, end_pos):
        sx, sy = start_pos[0], start_pos[1]
        ex, ey = end_pos[0], end_pos[1]

        for obs in self.obstacles:
            for i in range(100):
                u = i / 100
                x = sx * u + ex * (1 - u)
                y = sy * u + ey * (1 - u)
                if obs.collidepoint(x, y):
                    return True
        return False

    def find_k_nearest(self, n, k):
        k_dists = {}
        for i in range(k + 1):
            d = dist_to_node(self.nodes[i], n)
            k_dists[d] = self.nodes[i]

        for i in range(len(self.nodes)):
            nn = self.nodes[i]
            d = dist_to_node(nn, n)
            if d not in k_dists.keys():
                max_dist = max(k_dists.keys())
                if d < max_dist:
                    k_dists.pop(max_dist)
                    k_dists[d] = nn

        return k_dists.values()

    def connect(self, n1, n2):
        if self.cross_obstacle(n1.get_coords(), n2.get_coords()):
            return False
        else:
            add_edge(n1, n2)

    def add_edges(self, n):
        k_nearest = self.find_k_nearest(n, self.k)
        for node in k_nearest:
            self.connect(n, node)
        return n

    def update_edges_gt(self, n):
        k_nearest = self.find_k_nearest(n, self.k)
        for node in k_nearest:
            if node in n.adj:
                continue
            self.connect(n, node)
        return n

    def update_edges_lt(self, n):
        k_nearest = self.find_k_nearest(n, self.k)
        adj = n.adj.copy()
        for node in adj:
            if node in k_nearest:
                continue
            else:
                del n.adj[node]
                del n.edge[node]
        return n

    def find_node_in_radius(self, p, r):
        x = p[0]
        y = p[1]
        if len(self.nodes) == 0:
            return None
        max_dist = r
        closest_node = None
        for node in self.nodes:
            d = dist_to_point(node, p)
            if d <= max_dist:
                max_dist = d
                xx, yy = node.get_coords()
                if (x - xx) ** 2 + (y - yy) ** 2 <= r ** 2:
                    closest_node = node
        return closest_node

    def get_start_node(self):
        return self.find_node_in_radius(self.start_pose, self.start_radius)

    def get_end_node(self):
        return self.find_node_in_radius(self.goal_pose, self.goal_radius)

    def create_network(self, surf, nr, neighbours):
        self.k = neighbours
        for node in self.nodes:
            node.adj = {}
            node.edge = {}
            self.add_edges(node)
            node.draw(surf, nr, 1)

    def update_network_gt(self):
        for node in self.nodes:
            self.update_edges_gt(node)

    def update_network_lt(self):
        for node in self.nodes:
            self.update_edges_lt(node)

    def update_k(self, k):
        for node in self.nodes:
            node.search = None
            node.parent = None
        if k > self.k:
            self.k = k
            self.update_network_gt()
        elif k < self.k:
            self.k = k
            self.update_network_lt()

    def update_pose(self, sp, gp):
        self.start_pose = sp
        self.goal_pose = gp