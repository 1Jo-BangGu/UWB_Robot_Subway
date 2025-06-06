import math
import heapq
import cv2
import numpy as np

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __lt__(self, other):
        return self.f < other.f

def heuristic(pos, goal):
    return math.hypot(goal[0] - pos[0], goal[1] - pos[1])

def collision_check(position, obstacle_list, robot_radius=10):
    x, y = position
    for (ox, oy, size) in obstacle_list:
        if math.hypot(ox - x, oy - y) <= size + robot_radius:
            return True
    return False

def get_action():
    return [(0, 1, 1), (1, 0, 1), (0, -1, 1), (-1, 0, 1),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2))]

def reconstruct_path(current_node):
    path = []
    while current_node is not None:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]

def find_nearest_goal(current_pos, goals):
    min_dist = float('inf')
    nearest = None
    for goal in goals:
        dist = heuristic(current_pos, goal)
        if dist < min_dist:
            min_dist = dist
            nearest = goal
    return nearest

def a_star(start, goal, space, obstacle_list):
    start_node = Node(None, tuple(start))
    goal_node = Node(None, tuple(goal))
    open_list = []
    closed_set = set()
    heapq.heappush(open_list, (start_node.f, start_node))

    while open_list:
        _, current_node = heapq.heappop(open_list)
        if heuristic(current_node.position, goal_node.position) < 1.0:
            return reconstruct_path(current_node)
        if current_node.position in closed_set:
            continue
        closed_set.add(current_node.position)
        for dx, dy, cost in get_action():
            x = current_node.position[0] + dx
            y = current_node.position[1] + dy
            new_pos = (x, y)
            if not (0 <= x <= 180 and 0 <= y <= 90):
                continue
            if collision_check(new_pos, obstacle_list):
                continue
            neighbor = Node(current_node, new_pos)
            neighbor.g = current_node.g + cost
            neighbor.h = heuristic(new_pos, goal_node.position)
            neighbor.f = neighbor.g + neighbor.h
            heapq.heappush(open_list, (neighbor.f, neighbor))
    return []

def draw_path_on_frame(frame, path, M_real2pixel):
    for i in range(1, len(path)):
        pt1 = cv2.perspectiveTransform(np.array([[[path[i-1][0], path[i-1][1]]]], dtype=np.float32), M_real2pixel)[0][0]
        pt2 = cv2.perspectiveTransform(np.array([[[path[i][0], path[i][1]]]], dtype=np.float32), M_real2pixel)[0][0]
        cv2.line(frame, tuple(pt1.astype(int)), tuple(pt2.astype(int)), (255, 0, 255), 2)
    return frame
