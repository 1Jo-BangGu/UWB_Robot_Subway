# planning.py

import heapq
import math

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __lt__(self, other):
        return self.f < other.f

class Planning:
    def __init__(self, robot_radius=10):
        self.robot_radius = robot_radius
        self.current_path_idx = 0
        self.precomputed_paths = []
        self.path_sent = False

    def get_action(self):
        return [(0, 1, 1), (1, 0, 1), (0, -1, 1), (-1, 0, 1),
                (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2))]

    def heuristic(self, pos, goal):
        return math.hypot(goal[0] - pos[0], goal[1] - pos[1])

    def collision_check(self, position, obstacle_list):
        x, y = position
        for (ox, oy, size) in obstacle_list:
            if math.hypot(ox - x, oy - y) <= size + self.robot_radius:
                return True
        return False

    def reconstruct_path(self, current_node):
        path = []
        while current_node is not None:
            path.append(current_node.position)
            current_node = current_node.parent
        return path[::-1]

    def a_star(self, start, goal, map_size, obstacle_list):
        start_node = Node(None, tuple(start))
        goal_node = Node(None, tuple(goal))
        open_list = []
        closed_set = set()
        heapq.heappush(open_list, (start_node.f, start_node))

        while open_list:
            _, current_node = heapq.heappop(open_list)
            if self.heuristic(current_node.position, goal_node.position) < 1.0:
                return self.reconstruct_path(current_node)
            if current_node.position in closed_set:
                continue
            closed_set.add(current_node.position)
            for dx, dy, cost in self.get_action():
                x = current_node.position[0] + dx
                y = current_node.position[1] + dy
                new_pos = (x, y)
                if not (0 <= x <= map_size[0] and 0 <= y <= map_size[1]):
                    continue
                if self.collision_check(new_pos, obstacle_list):
                    continue
                neighbor = Node(current_node, new_pos)
                neighbor.g = current_node.g + cost
                neighbor.h = self.heuristic(new_pos, goal_node.position)
                neighbor.f = neighbor.g + neighbor.h
                heapq.heappush(open_list, (neighbor.f, neighbor))
        return []

    def plan(self, robot_pose, obstacle_list, map_size, goals, patrol_signal=False):
        stop_signal = False
        path = []

        if not goals:
            stop_signal = True
            return path, stop_signal

        if not self.precomputed_paths:
            self.precomputed_paths = self.generate_all_paths(robot_pose[:2], goals, obstacle_list, map_size)
            self.current_path_idx = 0
            self.path_sent = False

        if self.current_path_idx < len(self.precomputed_paths):
            path = self.precomputed_paths[self.current_path_idx]
            if path:
                last_goal = path[-1]
                if self.heuristic(robot_pose[:2], last_goal) < 5:
                    self.current_path_idx += 1
                    self.path_sent = False
                    if self.current_path_idx >= len(self.precomputed_paths):
                        print("✅ 모든 경로 완료!")
        return path, stop_signal

    def generate_all_paths(self, start_pos, goals, obstacles, map_size):
        paths = []
        current_pos = start_pos[:2]
        for i, goal in enumerate(goals):
            path = self.a_star(current_pos, goal, map_size, obstacles)
            if not path:
                print(f"⚠️ {i+1}번 경로 생성 실패")
                paths.append([])
            else:
                paths.append(path)
                current_pos = goal
        return paths
