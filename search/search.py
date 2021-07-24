import math
import queue as Q
import time
from typing import List

from planners.prm import Node, NodeEdge


class Dijkstra:
    def __init__(self, nodes: List[Node]):
        self.nodes = nodes
        self.sn = None
        self.gn = None
        self.path = []
        self.goal_found = False

    def solve(self, nodes, sn, gn):
        if sn is None or gn is None:
            return []
        if gn.parent is not None and sn == self.sn:
            return self.construct_path(sn, gn)

        pq = Q.PriorityQueue()
        self.nodes = nodes
        visited = [False for i in range(len(self.nodes))]
        dist = [math.inf for i in range(len(self.nodes))]
        self.sn = sn
        self.gn = gn

        pq.put((0, sn.id))

        dist[sn.id] = 0

        while not pq.empty():
            min_val, i = pq.get()
            visited[i] = True
            self.nodes[i].search = "Dijkstra"
            for neighbour in self.nodes[i].get_connections():
                ni = neighbour.id
                if visited[ni]: continue
                if dist[ni] < min_val: continue
                new_dist = dist[i] + self.nodes[i].get_weight(neighbour)
                if new_dist < dist[ni]:
                    dist[ni] = new_dist
                    neighbour.parent = self.nodes[i]
                    pq.put((new_dist, neighbour.id))
            time.sleep(0.01)
            if self.nodes[i] == gn:
                break
        if gn.parent is None:
            return []
        else:
            self.goal_found = True
            return self.construct_path(sn, gn)

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
