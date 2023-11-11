import heapq
from collections import deque


class State:
    def __init__(self, name, g_value, h_value):
        # Initialize the State with a name, g_value (cost from start), and h_value (heuristic value)
        self.name = name
        self.g_value = g_value
        self.h_value = h_value

    def f_value(self):
        # Calculate the total cost (f_value) by adding g_value and h_value
        return self.g_value + self.h_value

    def __lt__(self, other):
        # Define less-than comparison based on f_value and name
        return (self.f_value(), self.name) < (other.f_value(), other.name)


def bfs_search(start, goal, live_traffic, sunday_traffic):
    # Implementation of BFS search algorithm

    open_queue = deque([State(start, 0, sunday_traffic[start])])
    visited = set()
    came_from = {}

    while open_queue:
        current = open_queue.popleft()

        if current.name == goal:
            # Reconstruct and return the path if the goal is reached
            path = []
            while current.name in came_from:
                path.append((current.name, current.g_value))
                current = came_from[current.name]
            path.append((start, 0))
            path.reverse()
            return path

        visited.add(current.name)

        for neighbor, cost in sorted(live_traffic.get(current.name, [])):
            if neighbor not in visited and not any(
                state.name == neighbor for state in open_queue
            ):
                # If neighbor is not visited and not in the open_queue, add it to the queue
                g_value = current.g_value + cost
                h_value = sunday_traffic[neighbor]
                neighbor_state = State(neighbor, g_value, h_value)
                open_queue.append(neighbor_state)
                came_from[neighbor_state.name] = current

    return None


def ucs_search(start, goal, live_traffic, sunday_traffic):
    # Implementation of UCS search algorithm

    open_queue = [State(start, 0, sunday_traffic[start])]
    closed_set = set()
    came_from = {}

    while open_queue:
        current = heapq.heappop(open_queue)

        if current.name == goal:
            path = []
            # Reconstruct and return the path if the goal is reached
            while current.name in came_from:
                path.append((current.name, current.g_value))
                current = came_from[current.name]
            path.append((start, 0))
            path.reverse()
            return path

        closed_set.add(current.name)

        for neighbor, cost in sorted(live_traffic.get(current.name, [])):
            if neighbor in closed_set:
                continue

            g_value = current.g_value + cost
            h_value = sunday_traffic[neighbor]
            neighbor_state = State(neighbor, g_value, h_value)

            if any(
                state.name == neighbor and state.g_value <= g_value
                for state in open_queue
            ):
                continue

            heapq.heappush(open_queue, neighbor_state)
            came_from[neighbor] = current

    return None


from collections import deque


def dfs_search(start, goal, live_traffic, sunday_traffic):
    # Implementation of DFS search algorithm

    open_stack = [State(start, 0, sunday_traffic[start])]
    closed_set = set()
    came_from = {}

    while open_stack:
        current = open_stack.pop()

        if current.name == goal:
            # If neighbor is not visited and not in the open_queue, add it to the queue
            path = []
            while current.name in came_from:
                path.append((current.name, current.g_value))
                current = came_from[current.name]
            path.append((start, 0))
            path.reverse()
            return path

        closed_set.add(current.name)

        for neighbor, cost in live_traffic.get(current.name, []):
            if neighbor in closed_set:
                continue

            g_value = current.g_value + cost
            h_value = sunday_traffic[neighbor]
            neighbor_state = State(neighbor, g_value, h_value)

            if neighbor_state in open_stack:
                continue

            open_stack.append(neighbor_state)
            came_from[neighbor] = current

    return None


def astar_search(start, goal, live_traffic, sunday_traffic):
    # Implementation of A* search algorithm

    open_queue = [State(start, 0, sunday_traffic[start])]
    closed_set = set()
    came_from = {}

    while open_queue:
        current = heapq.heappop(open_queue)

        if current.name == goal:
            # If neighbor is not visited and not in the open_queue, add it to the queue
            path = []
            while current.name in came_from:
                path.append((current.name, current.g_value))
                current = came_from[current.name]
            path.append((start, 0))
            path.reverse()
            return path

        closed_set.add(current.name)

        for neighbor, cost in sorted(live_traffic.get(current.name, [])):
            if neighbor in closed_set:
                continue

            g_value = current.g_value + cost
            h_value = sunday_traffic[neighbor]
            neighbor_state = State(neighbor, g_value, h_value)

            if any(
                state.name == neighbor and state.g_value <= g_value
                for state in open_queue
            ):
                continue

            heapq.heappush(open_queue, neighbor_state)
            came_from[neighbor] = current

    return None


def main():
    # read input from input.txt file
    with open("input.txt", "r") as input_file:
        algorithm = input_file.readline().strip()
        start_state = input_file.readline().strip()
        goal_state = input_file.readline().strip()
        num_live_traffic = int(input_file.readline())
        live_traffic = {}
        for _ in range(num_live_traffic):
            state1, state2, cost = input_file.readline().split()
            cost = int(cost)
            live_traffic.setdefault(state1, []).append((state2, cost))
        num_sunday_traffic = int(input_file.readline())
        sunday_traffic = {}
        for _ in range(num_sunday_traffic):
            state, time = input_file.readline().split()
            time = int(time)
            sunday_traffic[state] = time

    if algorithm == "BFS":
        result = bfs_search(start_state, goal_state, live_traffic, sunday_traffic)
    elif algorithm == "DFS":
        result = dfs_search(start_state, goal_state, live_traffic, sunday_traffic)
    elif algorithm == "UCS":
        result = ucs_search(start_state, goal_state, live_traffic, sunday_traffic)
    elif algorithm == "A*":
        result = astar_search(start_state, goal_state, live_traffic, sunday_traffic)
    else:
        print("Invalid algorithm specified in input.txt")
        return

    if result:
        with open("output.txt", "w") as output_file:
            for state, cost in result:
                output_file.write(f"{state} {cost}\n")
    else:
        print("No path found from start to goal.")


if __name__ == "__main__":
    main()
