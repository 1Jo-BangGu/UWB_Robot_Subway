# # planning.py

# import heapq
# import math

# class Node:
#     def __init__(self, parent=None, position=None):
#         self.parent = parent
#         self.position = position
#         self.g = 0
#         self.h = 0
#         self.f = 0
#     def __lt__(self, other):
#         return self.f < other.f

# class Planning:
#     def __init__(self, robot_radius=10):
#         self.robot_radius = robot_radius
#         self.current_goal_idx = 0
#         self.latest_path = []
#         self.goal_reached = False
#         self.arrive_flag = False

#     def get_action(self):
#         return [(0, 1, 1), (1, 0, 1), (0, -1, 1), (-1, 0, 1),
#                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
#                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2))]

#     def heuristic(self, pos, goal):
#         return math.hypot(goal[0] - pos[0], goal[1] - pos[1])

#     def collision_check(self, position, obstacle_list):
#         x, y = position
#         for (ox, oy, size) in obstacle_list:
#             if math.hypot(ox - x, oy - y) <= size + self.robot_radius:
#                 return True
#         return False

#     def reconstruct_path(self, current_node):
#         path = []
#         while current_node is not None:
#             path.append(current_node.position)
#             current_node = current_node.parent
#         return path[::-1]

#     def a_star(self, start, goal, map_size, obstacle_list):
#         start_node = Node(None, tuple(start))
#         goal_node = Node(None, tuple(goal))
#         open_list = []
#         closed_set = set()
#         heapq.heappush(open_list, (start_node.f, start_node))

#         while open_list:
#             _, current_node = heapq.heappop(open_list)
#             if self.heuristic(current_node.position, goal_node.position) < 1.0:
#                 return self.reconstruct_path(current_node)
#             if current_node.position in closed_set:
#                 continue
#             closed_set.add(current_node.position)
#             for dx, dy, cost in self.get_action():
#                 x = current_node.position[0] + dx
#                 y = current_node.position[1] + dy
#                 new_pos = (x, y)
#                 if not (0 <= x <= map_size[0] and 0 <= y <= map_size[1]):
#                     continue
#                 if self.collision_check(new_pos, obstacle_list):
#                     continue
#                 neighbor = Node(current_node, new_pos)
#                 neighbor.g = current_node.g + cost
#                 neighbor.h = self.heuristic(new_pos, goal_node.position)
#                 neighbor.f = neighbor.g + neighbor.h
#                 heapq.heappush(open_list, (neighbor.f, neighbor))
#         return []

#     def plan(self, initial_robot_pose, robot_pose, obstacle_list, map_size, goals, return_flag=False, fire_signal=0, fall_signal=0):
#         stop_signal = False
#         path = []

#         # 객체 감지
#         if fire_signal == 1 or fall_signal == 1:
#             count = 0
#             while count < 5000:
#                 stop_signal = True
#                 self.arrive_flag = False

#                 if return_flag == 1:
#                     ## 초기 위치로 이동
#                     self.latest_path = []
#                     self.latest_path = self.a_star(robot_pose[:2], initial_robot_pose, map_size, obstacle_list)
#                     stop_signal = False
#                     self.arrive_flag = False

#                 elif return_flag == 0:
#                     stop_signal = False
#                     if not goals or self.current_goal_idx >= len(goals): # goal이 없는 경우, 인덱스 초과인 경우
#                         stop_signal = True
#                         self.latest_path = []

#                     current_goal = goals[self.current_goal_idx]

#                     # 아직 도달하지 않았고 경로가 없으면 생성
#                     if not self.latest_path:
#                         self.arrive_flag = False
#                         self.latest_path = self.a_star(robot_pose[:2], current_goal, map_size, obstacle_list)
#                         if not self.latest_path:
#                             print(f"⚠️ {self.current_goal_idx+1}번 경로 생성 실패 → 다음으로")
#                             self.current_goal_idx += 1

#                     # 도달 판정
#                     if self.heuristic(robot_pose[:2], current_goal) < 10:
#                         self.arrive_flag = True
#                         self.current_goal_idx += 1
#                         self.latest_path = []
#                         if self.current_goal_idx >= len(goals):
#                             stop_signal = True
#                             print("✅ 모든 목표 도달 완료!")
#                             # print(self.latest_path, stop_signal, self.arrive_flag)
#                             return self.latest_path, stop_signal, self.arrive_flag
                 
#                         current_goal = goals[self.current_goal_idx]
#                 else:
#                     stop_signal = 1
                    
#                 count += 1

#                 return self.latest_path, stop_signal, self.arrive_flag

#         # 객체 감지 X
#         else:
#             if return_flag == 1:
#                 ## 초기 위치로 이동
#                 self.latest_path = []
#                 self.latest_path = self.a_star(robot_pose[:2], initial_robot_pose, map_size, obstacle_list)
#                 stop_signal = False
#                 self.arrive_flag = False

#             else:
#                 stop_signal = False
#                 if not goals or self.current_goal_idx >= len(goals): # goal이 없는 경우, 인덱스 초과인 경우
#                     stop_signal = True
#                     self.latest_path = []
#                     return self.latest_path, stop_signal, self.arrive_flag

#                 current_goal = goals[self.current_goal_idx]

#                 # 아직 도달하지 않았고 경로가 없으면 생성
#                 if not self.latest_path:
#                     self.arrive_flag = False
#                     self.latest_path = self.a_star(robot_pose[:2], current_goal, map_size, obstacle_list)
#                     if not self.latest_path:
#                         print(f"⚠️ {self.current_goal_idx+1}번 경로 생성 실패 → 다음으로")
#                         self.current_goal_idx += 1

#                 # 도달 판정
#                 if self.heuristic(robot_pose[:2], current_goal) < 10:
#                     self.arrive_flag = True
#                     self.current_goal_idx += 1
#                     self.latest_path = []
#                     if self.current_goal_idx >= len(goals):
#                         stop_signal = True
#                         print("✅ 모든 목표 도달 완료!")
#                         # print(self.latest_path, stop_signal, self.arrive_flag)
#                         return self.latest_path, stop_signal, self.arrive_flag
                
#                     current_goal = goals[self.current_goal_idx]
                
#             return self.latest_path, stop_signal, self.arrive_flag

       
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
    def __init__(self, robot_radius=10, stop_signal_threshold=50):
        self.robot_radius = robot_radius
        self.current_goal_idx = 0
        self.latest_path = []
        self.goal_reached = False
        self.arrive_flag = False
        # 안전 신호 유지 관련 변수
        self.stop_signal_count = 0
        self.stop_signal_threshold = stop_signal_threshold
        self.detect_flag = 0

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

    def plan(self, initial_robot_pose, robot_pose, obstacle_list, map_size, goals, return_flag=None, fire_signal=0, fall_signal=0):
        stop_signal = False
        print(f'return_flag : {return_flag}, detect_flag : {self.detect_flag}')

        if self.detect_flag == 0 and fire_signal == 1 or fall_signal == 1:
            print('detect_flag changed to True')
            self.detect_flag = 1
            return_flag = -1

        if self.detect_flag == 1:
            # 아래 분기: return_flag에 따라 동작
            if return_flag == 1:
                # 초기 위치로 이동(주행)
                # print("🔥 안전 신호 + 복귀 명령: 초기 위치로 이동")
                self.latest_path = self.a_star(robot_pose[:2], initial_robot_pose, map_size, obstacle_list)
                stop_signal = False
                self.arrive_flag = False

                # print('1_stop_signal: ', stop_signal)
                # print('1_arrive_flag: ', self.arrive_flag)

                return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag

            elif return_flag == 0:
                print('return flag == 0')
                self.detect_flag = 0
                fire_signal = 0
                fall_signal = 0
                # 목표 리스트에 따라 주행 지속
                # print("🔥🔥 안전 신호 + 주행 지속 명령: 목표 리스트 따라 주행")
                if not goals or self.current_goal_idx >= len(goals):
                    stop_signal = True
                    self.latest_path = []
                    self.arrive_flag = True
                    return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag

                current_goal = goals[self.current_goal_idx]

                if not self.latest_path:
                    self.arrive_flag = False
                    self.latest_path = self.a_star(robot_pose[:2], current_goal, map_size, obstacle_list)
                    if not self.latest_path:
                        print(f"⚠️ {self.current_goal_idx+1}번 경로 생성 실패 → 다음으로")
                        self.current_goal_idx += 1

                if self.heuristic(robot_pose[:2], current_goal) < 10:
                    self.arrive_flag = True
                    self.current_goal_idx += 1
                    self.latest_path = []
                    if self.current_goal_idx >= len(goals):
                        stop_signal = True
                        print("✅ 모든 목표 도달 완료!")
                        return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag
                    current_goal = goals[self.current_goal_idx]

                # print('0_stop_signal: ', stop_signal)
                # print('0_arrive_flag: ', self.arrive_flag)

                return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag

            else:
                print('else')
                # 그 외: 명령 없음, 정지 상태 유지
                # print("🔥 안전 신호 + 명령 없음: 정지 상태 유지")
                stop_signal = True
                self.latest_path = []
                self.arrive_flag = False

                # print('Null_stop_signal: ', stop_signal)
                # print('Null_arrive_flag: ', self.arrive_flag)

                return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag

        # 2. fire/fall 신호가 없으면: 목표 리스트에 따라 주행 지속
        else:
            # print("🟢 정상 주행: 목표 리스트 따라 주행")
            if not goals or self.current_goal_idx > len(goals):
                stop_signal = True
                self.latest_path = []
                return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag

            current_goal = goals[self.current_goal_idx]

            if not self.latest_path:
                self.arrive_flag = False
                self.latest_path = self.a_star(robot_pose[:2], current_goal, map_size, obstacle_list)
                if not self.latest_path:
                    print(f"⚠️ {self.current_goal_idx+1}번 경로 생성 실패 → 다음으로")
                    self.current_goal_idx += 1

            if self.heuristic(robot_pose[:2], current_goal) < 10:
                self.arrive_flag = True
                self.current_goal_idx += 1
                self.latest_path = []
                if self.current_goal_idx > len(goals):
                    stop_signal = True
                    print("✅ 모든 목표 도달 완료!")
                    return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag
                current_goal = goals[self.current_goal_idx]

        return self.latest_path, stop_signal, self.arrive_flag, fire_signal, fall_signal, return_flag
