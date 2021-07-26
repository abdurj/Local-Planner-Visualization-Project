from abc import ABC, abstractmethod
import math
import queue as Q
import time
from typing import List

import planners.planners
from planners.planners import Node, NodeEdge


class Search(ABC):
    def __init__(self):
        self.path = []

    @abstractmethod
    def solve(self, nodes, sn, gn):
        pass

    @abstractmethod
    def update_solution(self, sn, gn):
        pass

    def construct_path(self, sn, gn):
        if gn is not None:
            path = [gn]
            nn = gn.parent
            while nn is not None:
                path.append(nn)
                nn = nn.parent

            path.reverse()
            self.path = path
        else:
            self.path = []

class InformedSearch(Search):
    def __init__(self, nodes: List[Node], name):
        super().__init__()
        self.name = name
        self.nodes = nodes
        self.sn = None
        self.gn = None
        self.path = []
        self.goal_found = False
        self.pq = None
        self.visited = []
        self.dist = []

    def solve(self, nodes, sn, gn):
        if sn is None or gn is None:
            self.path = []
            return
        if gn.parent is not None and sn == self.sn:
            self.construct_path(sn, gn)
            return
        
        self.path = []
        self.pq = Q.PriorityQueue()
        self.nodes = nodes
        self.visited = [False for i in range(len(self.nodes))]
        self.dist = [math.inf for i in range(len(self.nodes))]
        self.sn = sn
        self.gn = gn

        self.pq.put((0, sn.id))

        self.dist[sn.id] = 0

        self.search(sn, gn)

    def update_solution(self, sn, gn):
        if self.pq is None:
            self.path = []
            return
        if sn is None or gn is None:
            self.path = []
            return
        if gn.parent is not None and sn == self.sn:
            self.construct_path(sn, gn)
            return
        if sn != self.sn:
            self.solve(self.nodes, sn, gn)
            return
        self.sn = sn
        self.gn = gn
        self.search(sn, gn)

    def search(self, sn, gn):
        while not self.pq.empty():
            min_val, i = self.pq.get()
            self.visited[i] = True
            self.nodes[i].search = self.name
            for neighbour in self.nodes[i].get_connections():
                ni = neighbour.id
                if self.visited[ni]: continue
                if self.dist[ni] < min_val: continue
                new_dist = self.cost(self.nodes[i], neighbour) + self.heuristic(self.nodes[i], neighbour)
                if new_dist < self.dist[ni]:
                    self.dist[ni] = new_dist
                    neighbour.parent = self.nodes[i]
                    self.pq.put((new_dist, neighbour.id))
            time.sleep(0.01)
            if self.nodes[i] == gn:
                break
        if gn.parent is None:
            self.path = []
        else:
            self.goal_found = True
            self.construct_path(sn, gn)

    @abstractmethod
    def cost(self, node, neighbour):
        pass

    @abstractmethod
    def heuristic(self, node, neighbour):
        pass


class Dijkstra(InformedSearch):
    def __init__(self, nodes: List[Node]):
        super().__init__(nodes, 'Dijkstra')

    def cost(self, node, neighbour):
        return self.dist[node.id] + self.nodes[node.id].get_weight(neighbour)

    def heuristic(self, node, neighbour):
        return 0


class AStar(InformedSearch):
    def __init__(self, nodes: List[Node]):
        super().__init__(nodes, 'AStar')

    def cost(self, curr_node, neighbour_node):
        return self.dist[curr_node.id] + self.nodes[curr_node.id].get_weight(neighbour_node)

    def heuristic(self, curr_node, neighbour_node):
        return planners.planners.dist_to_node(neighbour_node, self.gn)

    def search(self, sn, gn):
        while not self.pq.empty():
            min_val, i = self.pq.get()
            self.visited[i] = True
            self.nodes[i].search = self.name
            for neighbour in self.nodes[i].get_connections():
                ni = neighbour.id
                if self.visited[ni]: continue
                if self.dist[ni] < min_val: continue
                new_dist = self.cost(self.nodes[i], neighbour) + self.heuristic(self.nodes[i], neighbour)
                if new_dist < self.dist[ni]:
                    self.dist[ni] = self.cost(self.nodes[i], neighbour)
                    neighbour.parent = self.nodes[i]
                    self.pq.put((new_dist, neighbour.id))
            time.sleep(0.01)
            if self.nodes[i] == gn:
                break
        if gn.parent is None:
            self.path = []
        else:
            self.goal_found = True
            self.construct_path(sn, gn)


class GreedyBFS(InformedSearch):
    def __init__(self, nodes: List[Node]):
        super().__init__(nodes, 'GreedyBFS')

    def cost(self, curr_node, neighbour_node):
        return 0

    def heuristic(self, curr_node, neighbour_node):
        return planners.planners.dist_to_node(neighbour_node, self.gn)

    def update_solution(self, sn, gn):
        for node in self.nodes:
            if node.search == "GreedyBFS":
                node.search = None
        self.solve(self.nodes, sn, gn)
