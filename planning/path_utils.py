# planning/path_utils.py

from planning.planning import a_star

def plan_full_path(start_pos, goal_list_cm, map_size, obstacle_list):
    """
    여러 목표지점을 순차적으로 따라가는 전체 경로 생성
    """
    segment_path = []
    curr_pos = start_pos
    for goal in goal_list_cm:
        path = a_star(curr_pos, goal, map_size, obstacle_list)
        if not path:
            print(f"경로 없음: {curr_pos} → {goal}")
            continue
        if segment_path and path[0] == segment_path[-1]:
            segment_path += path[1:]
        else:
            segment_path += path
        curr_pos = goal
    return segment_path
