import heapq

# Define a sample graph for testing
graph = {
    'A': ['B', 'C'],
    'B': ['D', 'E'],
    'C': ['F'],
    'D': [],
    'E': ['G'],
    'F': [],
    'G': [],
}

# Breadth-First Search (BFS)
def bfs(graph, start, goal):
    visited = set()
    queue = [[start]]
    
    while queue:
        path = queue.pop(0)
        node = path[-1]
        
        if node == goal:
            return path
        
        if node not in visited:
            for neighbor in graph[node]:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
                visited.add(node)

# Depth-First Search (DFS)
def dfs(graph, start, goal):
    def dfs_recursive(node, path):
        if node == goal:
            return path
        for neighbor in graph[node]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                result = dfs_recursive(neighbor, new_path)
                if result:
                    return result
        return None
    
    return dfs_recursive(start, [start])

# Beam Search
def beam_search(graph, start, goal, beam_width):
    visited = set()
    queue = [[start]]
    
    while queue:
        paths = []
        for path in queue:
            node = path[-1]
            if node == goal:
                return path
            if node not in visited:
                visited.add(node)
                for neighbor in graph[node]:
                    new_path = list(path)
                    new_path.append(neighbor)
                    paths.append(new_path)
        queue = sorted(paths, key=lambda x: heuristic(x[-1], goal))[:beam_width]

# Hill Climbing
def hill_climbing(graph, start, goal, heuristic_function):
    current_node = start
    path = [start]
    
    while current_node != goal:
        neighbors = graph[current_node]
        if not neighbors:
            break
        
        neighbor_scores = [(neighbor, heuristic_function(neighbor, goal)) for neighbor in neighbors]
        neighbor_scores.sort(key=lambda x: x[1])
        
        next_node, next_score = neighbor_scores[0]
        
        if next_score >= heuristic_function(current_node, goal):
            break
        
        current_node = next_node
        path.append(current_node)
    
    return path

# Oracle Search (assuming an oracle function oracle_fn)
def oracle_search(graph, start, goal, oracle_fn):
    path = [start]
    current_node = start
    
    while current_node != goal:
        oracle_value = oracle_fn(current_node)
        neighbors = graph[current_node]
        
        if not neighbors:
            break
        
        next_node = max(neighbors, key=lambda n: oracle_fn(n))
        path.append(next_node)
        current_node = next_node
    
    return path

# Branch and Bound
def branch_and_bound(graph, start, goal, heuristic_function):
    def bound(path):
        return sum(heuristic_function(node, goal) for node in path)
    
    queue = [(bound([start]), [start])]
    
    while queue:
        _, path = queue.pop(0)
        current_node = path[-1]
        
        if current_node == goal:
            return path
        
        for neighbor in graph[current_node]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append((bound(new_path), new_path))
        
        queue.sort(key=lambda x: x[0])

# Branch and Bound with Extended Heuristics
def branch_and_bound_extended(graph, start, goal, heuristic_function, extended_heuristic_function):
    def bound(path):
        return sum(heuristic_function(node, goal) + extended_heuristic_function(node) for node in path)
    
    queue = [(bound([start]), [start])]
    
    while queue:
        _, path = queue.pop(0)
        current_node = path[-1]
        
        if current_node == goal:
            return path
        
        for neighbor in graph[current_node]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append((bound(new_path), new_path))
        
        queue.sort(key=lambda x: x[0])

# Branch and Bound with Extended List
def branch_and_bound_extended_list(graph, start, goal, heuristic_function, extended_list_function):
    def bound(path):
        return sum(heuristic_function(node, goal) + extended_list_function(node) for node in path)
    
    queue = [(bound([start]), [start])]
    
    while queue:
        _, path = queue.pop(0)
        current_node = path[-1]
        
        if current_node == goal:
            return path
        
        for neighbor in graph[current_node]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append((bound(new_path), new_path))
        
        queue.sort(key=lambda x: x[0])

# A* Algorithm
def astar(graph, start, goal, heuristic_function):
    def f_score(node, path):
        return len(path) + heuristic_function(node, goal)
    
    open_set = [(f_score(start, [start]), [start])]
    closed_set = set()
    
    while open_set:
        _, path = heapq.heappop(open_set)
        current_node = path[-1]
        
        if current_node == goal:
            return path
        
        if current_node in closed_set:
            continue
        
        closed_set.add(current_node)
        
        for neighbor in graph[current_node]:
            if neighbor not in path:
                new_path = list(path)
                new_path.append(neighbor)
                heapq.heappush(open_set, (f_score(neighbor, new_path), new_path))

# Define a sample heuristic function (can be customized)
def heuristic(node, goal):
    # In this example, assume a simple distance heuristic
    return 1

# Example usage:
goal_node = 'G'
print("Breadth-First Search:", bfs(graph, 'A', goal_node))
print("Depth-First Search:", dfs(graph, 'A', goal_node))
print("Beam Search:", beam_search(graph, 'A', goal_node, beam_width=2))
print("Hill Climbing:", hill_climbing(graph, 'A', goal_node, heuristic))
print("Oracle Search:", oracle_search(graph, 'A', goal_node, oracle_fn=lambda x: ord(goal_node) - ord(x)))
print("Branch and Bound:", branch_and_bound(graph, 'A', goal_node, heuristic))
print("Branch and Bound with Extended Heuristics:", branch_and_bound_extended(graph, 'A', goal_node, heuristic, extended_heuristic_function=heuristic))
print("Branch and Bound with Extended List:", branch_and_bound_extended_list(graph, 'A', goal_node, heuristic, extended_list_function=lambda x)
