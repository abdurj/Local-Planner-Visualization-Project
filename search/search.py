from abc import ABC, abstractmethod
import math
import queue as Q
import time
from typing import List

import planners.prm
from planners.prm import Node, NodeEdge


class Search(ABC):
    @abstractmethod
    def solve(self, nodes, sn, gn):
        pass

    @abstractmethod
    def update_solution(self, sn, gn):
        pass


class Dijkstra(Search):
    def __init__(self, nodes: List[Node]):
        self.nodes = nodes
        self.sn = None
        self.gn = None
        self.path = []
        self.goal_found = False
        self.pq = None
        self.visited = []
        self.dist = []

    def solve(self, nodes, sn, gn):
        self.path = []
        if sn is None or gn is None:
            self.path = []
            return
        if gn.parent is not None and sn == self.sn:
            self.path = self.construct_path(sn, gn)
            return

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
            return
        if sn is None or gn is None:
            self.path = []
            return
        if gn.parent is not None and sn == self.sn:
            self.path = self.construct_path(sn, gn)
            return
        if sn != self.sn:
            self.solve(self.nodes, sn, gn)
            return

        self.search(sn, gn)

    def search(self, sn, gn):
        while not self.pq.empty():
            min_val, i = self.pq.get()
            self.visited[i] = True
            self.nodes[i].search = "Dijkstra"
            for neighbour in self.nodes[i].get_connections():
                ni = neighbour.id
                if self.visited[ni]: continue
                if self.dist[ni] < min_val: continue
                new_dist = self.dist[i] + self.nodes[i].get_weight(neighbour)
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
            self.path = self.construct_path(sn, gn)

    def construct_path(self, sn, gn):
        if self.goal_found:
            path = [gn]
            nn = gn.parent
            while nn is not None:
                path.append(nn)
                nn = nn.parent

            path.reverse()
            self.path = path
            return path


class AStar(Dijkstra):
    def __init__(self, nodes: List[Node]):
        super().__init__(nodes)

    def search(self, sn, gn):
        while not self.pq.empty():
            min_val, i = self.pq.get()
            self.visited[i] = True
            self.nodes[i].search = "AStar"
            for neighbour in self.nodes[i].get_connections():
                ni = neighbour.id
                if self.visited[ni]: continue
                if self.dist[ni] < min_val: continue
                new_dist = self.dist[i] + self.nodes[i].get_weight(neighbour) + planners.prm.dist_to_node(neighbour, gn)
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
            self.path = self.construct_path(sn, gn)