import math
import random
import threading
import time
from typing import List, Any

import numpy as np
import pygame


class Color:
    WHITE = (255, 255, 255)
    LIGHTGREY = (130, 130, 130)
    GREY = (70, 70, 70)
    BLUE = (0, 0, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    GREY2 = (50, 50, 50)
    PURPLE = (199, 21, 133)
    BROWN = (210, 105, 30)
    LIGHT_BLUE = (176, 196, 250)
    LIGHT_PURPLE = (102,102,255)


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


# the following functions two are taken from https://github.com/jlehett/Pytential-Fields
def drawArrow(surface, startCoord, endCoord, LINE_WIDTH=3):
    """
        Draw an arrow via pygame.
    """
    A = startCoord
    B = endCoord
    dir_ = (B[0] - A[0], B[1] - A[1])
    dir_mag = math.sqrt(dir_[0] ** 2 + dir_[1] ** 2)
    H = dir_mag / 4.0
    W = H * 2.0
    if dir_mag == 0:
        dir_mag = 0.00001
    dir_ = (dir_[0] / dir_mag, dir_[1] / dir_mag)

    q = (dir_[1], -dir_[0])

    C = (
        B[0] - (H * dir_[0]) + (W * q[0] / 2.0),
        B[1] - (H * dir_[1]) + (W * q[1] / 2.0)
    )

    D = (
        B[0] - (H * dir_[0]) - (W * q[0] / 2.0),
        B[1] - (H * dir_[1]) - (W * q[1] / 2.0)
    )

    pygame.draw.line(
        surface, Color.GREY, A, B, LINE_WIDTH
    )
    pygame.draw.line(
        surface, Color.GREY, B, C, LINE_WIDTH
    )
    pygame.draw.line(
        surface, Color.GREY, B, D, LINE_WIDTH
    )


@np.vectorize
def cvtRange(x, in_min, in_max, out_min, out_max):
    """
        Convert a value, x, from its old range of
        (in_min to in_max) to the new range of
        (out_min to out_max)
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


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
        for neighbour in self.edge:
            color = Color.GREY
            if neighbour.search == "Dijkstra":
                color = Color.BLUE
            if neighbour.search == "AStar":
                color = Color.RED
            if neighbour.search == "GreedyBFS":
                color = Color.LIGHT_BLUE
            pygame.draw.line(surf, color, self.edge[neighbour].nfrom.get_coords(),
                             self.edge[neighbour].nto.get_coords(), width=width)
        pygame.draw.circle(surf, Color.LIGHTGREY, self.get_coords(), node_radius, width=0)

    def __str__(self):
        return f"{self.x}, {self.y}, {self.id}"


# Used to visualize pathfinding
class NodeEdge:
    def __init__(self, node_from: Node, node_to: Node):
        self.nfrom = node_from
        self.nto = node_to


class CircularObstacle:
    def __init__(self, x, y, rad):
        self.x = x
        self.y = y
        self.rad = rad

    def collidepoint(self, point):
        d = math.hypot(point[0]-self.x, point[1]-self.y)
        if(d<=self.rad):
            return True
        return False

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
        self.network_created = False

    def sample(self, sample_size):
        self.network_created = False
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
        if (self.network_created):
            return
        self.k = neighbours
        for node in self.nodes:
            node.adj = {}
            node.edge = {}
            self.add_edges(node)
            node.draw(surf, nr, 1)
        self.network_created = True

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


class RRT:
    nodes: List[Node]

    def __init__(self, map_dim, start_pose, start_radius, goal_pose, goal_radius, obstacles, bias):
        self.map_dim = self.mapx, self.mapy = map_dim
        self.start_pose = self.sx, self.sy = start_pose
        self.start_radius = start_radius
        self.goal_pose = self.gx, self.gy = goal_pose
        self.goal_radius = goal_radius
        self.obstacles = obstacles
        self.b = bias
        self.nodes = []
        self.path = []
        self.gn = None
        self.sn = None

    def start(self):
        print('start')
        self.nodes = []
        self.path = []
        self.add_node(0, *self.start_pose)
        self.sn = self.nodes[0]
        self.gn = None
        while self.gn is None:
            i = random.random()
            if i < self.b:
                self.bias(self.goal_pose)
            else:
                self.expand()

            time.sleep(0.05)

        self.construct_path(self.sn, self.gn)

    def bias(self, goal_pose):
        nearest_node = self.nearest(goal_pose)
        self.step(nearest_node, self.goal_pose)

    def expand(self):
        x, y = self.sample_envir()
        nearest_node = self.nearest((x, y))
        self.step(nearest_node, (x, y))

    def nearest(self, pose):
        min_dist = dist_to_point(self.nodes[0], pose)
        nearest_node = self.nodes[0]
        for node in self.nodes:
            d = dist_to_point(node, pose)
            if d < min_dist:
                min_dist = d
                nearest_node = node
        return nearest_node

    def step(self, nearest_node, pose, step_size=35):
        d = dist_to_point(nearest_node, pose)
        if d > step_size:
            (xnear, ynear) = nearest_node.get_coords()
            (xrand, yrand) = pose
            (dx, dy) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(dy, dx)
            x = int(xnear + step_size * math.cos(theta))
            y = int(ynear + step_size * math.sin(theta))
            # remove random node now that we have a node in the direction that is step_size away
            for obs in self.obstacles:
                if obs.collidepoint((x, y)):
                    return

            if not self.cross_obstacle(nearest_node.get_coords(), (x, y)):
                n = len(self.nodes)
                self.add_node(n, x, y)
                add_edge(nearest_node, self.nodes[n])
                self.nodes[n].parent = nearest_node

                if abs(x - self.goal_pose[0]) <= self.goal_radius and abs(y - self.goal_pose[1]) <= self.goal_radius:
                    self.gn = self.nodes[n]
        elif pose == self.goal_pose:
            if not self.cross_obstacle(nearest_node.get_coords(), (pose[0], pose[1])):
                n = len(self.nodes)
                self.add_node(n, pose[0], pose[1])
                add_edge(nearest_node, self.nodes[n])
                self.nodes[n].parent = nearest_node
                if abs(pose[0] - self.goal_pose[0]) <= self.goal_radius and abs(
                        pose[1] - self.goal_pose[1]) <= self.goal_radius:
                    self.gn = self.nodes[n]

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

    def update_pose(self, sp, gp):
        self.start_pose = sp
        self.goal_pose = gp

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

    def set_bias(self, bias):
        self.b = bias


#  Implementation from https://github.com/jlehett/Pytential-Fields used
class PotentialField:
    def __init__(self, map_dim, start_pose, start_radius, goal_pose, goal_radius, obstacles, map_surf, virtual):

        self.min_vel = 2
        self.max_vel = 40
        self.map_dim = self.mapw, self.maph = map_dim

        self.field = np.zeros((self.mapw, self.maph, 2))

        self.start_pose = self.sx, self.sy = start_pose
        self.start_radius = start_radius
        self.sn = None

        self.goal_pose = self.gx, self.gy = goal_pose
        self.goal_radius = goal_radius
        self.gn = None

        self.goal_field = None
        self.obstacles = obstacles
        self.obstacle_field = dict((i, np.array(0)) for i in obstacles)
        self.path = []

        self.updated = False
        self.virtual = virtual
        self.fcf = 5

        self.map_surf = map_surf

    def start(self):
        self.goal_field = self.attract_goal(self.goal_radius)
        self.field = self.goal_field
        self.updated = False
        for obs in self.obstacles:
            if not obs in self.obstacle_field.keys():
                self.obstacle_field[obs] = self.repel_obstacle(obs)
            self.field += self.obstacle_field[obs]

        self.clampField(25)
        self.make_path()

    def draw(self, surface, stride=(25, 25)):
        # Iterate through the field with proper strides
        bufferX = math.floor(stride[0] / 2.0)
        bufferY = math.floor(stride[1] / 2.0)
        for fieldX in range(bufferX, self.mapw - bufferX, stride[0]):
            for fieldY in range(bufferY, self.maph - bufferY, stride[1]):
                # Grab the field vector for the cell
                fieldVector = self.field[fieldX, fieldY]
                # Determine the x and y coordinate for the origin of the
                # potential line segment.
                startPixelX = fieldX
                startPixelY = fieldY
                # Determine the x and y coordinate for the end point of the
                # potential line segment.
                endPixelX = math.floor(startPixelX + fieldVector[0])
                endPixelY = math.floor(startPixelY + fieldVector[1])
                # Draw the vector to the pygame surface
                drawArrow(
                    surface,
                    (startPixelX, startPixelY),
                    (endPixelX, endPixelY)
                )

    def attract_goal(self, radius):
        target_pos = self.goal_pose

        x = np.linspace(0, self.mapw - 1, self.mapw)
        y = np.linspace(0, self.maph - 1, self.maph)
        meshgrid = np.meshgrid(x, y, sparse=False, indexing='ij')

        meshgridX = target_pos[0] - meshgrid[0]
        meshgridY = target_pos[1] - meshgrid[1]
        field = np.zeros((self.mapw, self.maph, 2))
        field[:, :, 0] = meshgridX
        field[:, :, 1] = meshgridY

        magnitudeField = np.sqrt(
            (field[:, :, 0] ** 2 + field[:, :, 1] ** 2)*2
        )
        magnitudeField = np.clip(
            magnitudeField, 0.0000001, math.inf
        )

        # Create normal field
        normalField = np.zeros(
            (self.mapw, self.maph, 2)
        )
        normalField[:, :, 0] = field[:, :, 0] / magnitudeField
        normalField[:, :, 1] = field[:, :, 1] / magnitudeField
        # Adjust magnitude field to fit radius parameter
        magnitudeField[
            np.where(magnitudeField <= self.goal_radius)
        ] = cvtRange(magnitudeField[
                         np.where(magnitudeField <= self.goal_radius)
                     ], 0, radius, self.max_vel, self.min_vel)

        magnitudeField[
            np.where(magnitudeField > radius)
        ] = 15
        # Create final field
        field[:, :, 0] = normalField[:, :, 0] * magnitudeField
        field[:, :, 1] = normalField[:, :, 1] * magnitudeField
        return field

    def repel_obstacle(self, obs):
        repulsePos = (obs.x, obs.y)
        # Create coordinate array to find distance
        x = np.linspace(0, self.mapw - 1, self.mapw)
        y = np.linspace(0, self.maph - 1, self.maph)
        meshgrid = np.meshgrid(x, y, sparse=False, indexing='ij')

        # Find distance from target to coordinate
        meshgridX = meshgrid[0] - repulsePos[0]
        meshgridY = meshgrid[1] - repulsePos[1]
        # Create field out of these distance calculations
        field = np.zeros(
            (self.mapw, self.maph, 2)
        )
        field[:, :, 0] = meshgridX
        field[:, :, 1] = meshgridY
        # Create magnitude field representing these distances
        magnitudeField = np.sqrt(
            field[:, :, 0] ** 2 + field[:, :, 1] ** 2
        )
        magnitudeField = np.clip(
            magnitudeField, 0.0000001, math.inf
        )
        # Create normal field
        normalField = np.zeros(
            (self.mapw, self.maph, 2)
        )
        normalField[:, :, 0] = field[:, :, 0] / magnitudeField
        normalField[:, :, 1] = field[:, :, 1] / magnitudeField
        # Adjust magnitude field to fit radius parameter
        filter_ = np.where(magnitudeField <= obs.rad*2.5)
        if len(filter_) != 0:
            magnitudeField[filter_] = cvtRange(magnitudeField[filter_], 0, obs.rad*2.5, self.max_vel, self.min_vel)
        filter_ = np.where(magnitudeField > obs.rad*2.5)
        if len(filter_) != 0:
            magnitudeField[filter_] = 0

        # Create final field
        field[:, :, 0] = normalField[:, :, 0] * magnitudeField
        field[:, :, 1] = normalField[:, :, 1] * magnitudeField
        return field

    def clampField(self, maxVel):
        """
            Clamp potential field such that the magnitude does not
            exceed maxVel
        """
        magnitudeField = np.sqrt(
            self.field[:, :, 0] ** 2 + self.field[:, :, 1] ** 2
        )
        magnitudeField = np.clip(
            magnitudeField, 0.000001, math.inf
        )
        normalField = np.zeros(
            (self.mapw, self.maph, 2)
        )
        normalField[:, :, 0] = self.field[:, :, 0] / magnitudeField
        normalField[:, :, 1] = self.field[:, :, 1] / magnitudeField
        magnitudeField = np.clip(magnitudeField, 0, maxVel)
        self.field[:, :, 0] = normalField[:, :, 0] * magnitudeField
        self.field[:, :, 1] = normalField[:, :, 1] * magnitudeField

    def update_pose(self, start_pose, goal_pose):
        self.start_pose = start_pose
        self.goal_pose = goal_pose

    def make_path(self):
        self.sn = Node(*self.start_pose, 0)
        self.gn = None
        self.path = [self.sn]
        curr = self.sn
        stuck = False
        while self.gn is None:
            if self.updated:
                self.updated = False
                break
            curr_pose = curr.get_coords()
            vec = self.field[curr_pose[0], curr_pose[1]]
            new_pose = int(curr_pose[0]+vec[0]), int(curr_pose[1]+vec[1])

            if new_pose == curr_pose:
                stuck = True

            if self.virtual:
                # Relative angle to goal
                if stuck:
                    theta = math.atan2(new_pose[1]-curr_pose[1], new_pose[0]-curr_pose[0])
                    self.field[curr_pose[0], curr_pose[1]] += [self.fcf*math.cos(theta)*vec[0], self.fcf*math.sin(theta)*vec[1]]
                    self.clampField(25)
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.02)

            new_node = Node(*new_pose, len(self.path))
            new_node.parent = curr
            self.path.append(new_node)

            if (new_pose[0] - self.goal_pose[0]) ** 2 + (new_pose[1] - self.goal_pose[1])**2 < self.goal_radius**2:
                self.gn = new_node
            curr = new_node

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles