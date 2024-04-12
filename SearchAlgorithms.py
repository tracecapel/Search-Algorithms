from collections import deque
import heapq

class SearchAlgorithms:
    @staticmethod
    def grid(rows, columns, start, end):
        grid = [[0] * columns for _ in range(rows)]
        
        start_row, start_col = divmod(start, columns)
        grid[start_row][start_col] = start
    
        end_row, end_col = divmod(end, columns)
        grid[end_row][end_col] = end
    
        return grid
    
    @staticmethod
    def print_grid(grid, start, end, optimal_path):
        grid_copy = [row[:] for row in grid]

        for point in optimal_path:
            grid_copy[point[0]][point[1]] = -1

        for row in grid_copy:
            for cell in row:
                if cell == start:
                    print("S", end=" ")
                elif cell == end:
                    print("E", end=" ")
                elif cell == -1:
                    print("*", end=" ")
                else:
                    print(".", end=" ")
            print()

    @staticmethod
    def heuristic(a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    @staticmethod
    def greedy_best_first_search(grid, start, goal):
        queue = []
        heapq.heappush(queue, (0, start))
        visited = set()
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                return True
            if current in visited:
                continue
            visited.add(current)
            for direction in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                next_row, next_col = current[0] + direction[0], current[1] + direction[1]
                if (0 <= next_row < len(grid) and 0 <= next_col < len(grid[0]) and
                        grid[next_row][next_col] != 1):
                    heapq.heappush(queue, (SearchAlgorithms.heuristic(goal, (next_row, next_col)), (next_row, next_col)))
        return False

    @staticmethod
    def a_star_search(grid, start, goal):
        queue = []
        heapq.heappush(queue, (0, start))
        g_costs = {start: 0}
        f_costs = {start: SearchAlgorithms.heuristic(start, goal)}
        came_from = {start: None}
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                return True
            for direction in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                next_row, next_col = current[0] + direction[0], current[1] + direction[1]
                if (0 <= next_row < len(grid) and 0 <= next_col < len(grid[0]) and
                        grid[next_row][next_col] != 1):
                    new_g_cost = g_costs[current] + 1
                    if (next_row, next_col) not in g_costs or new_g_cost < g_costs[(next_row, next_col)]:
                        g_costs[(next_row, next_col)] = new_g_cost
                        f_costs[(next_row, next_col)] = new_g_cost + SearchAlgorithms.heuristic(goal, (next_row, next_col))
                        came_from[(next_row, next_col)] = current
                        heapq.heappush(queue, (f_costs[(next_row, next_col)], (next_row, next_col)))
        return False

    @staticmethod
    def bfs(rows, cols, start, end):
        dx = [-1, 1, 0, 0]
        dy = [0, 0, -1, 1]
        
        parent_map = {}
        queue = deque()
        start_point = (start // cols, start % cols)
        end_point = (end // cols, end % cols)
        
        queue.append(start_point)
        parent_map[start_point] = None
        optimal_path = []
        nodes_explored = 0
        
        while queue:
            nodes_explored += 1
            curr = queue.popleft()
            if curr == end_point:
                backtrack = curr
                while backtrack is not None:
                    optimal_path.append(backtrack)
                    backtrack = parent_map.get(backtrack)
                optimal_path.reverse()
                return optimal_path, nodes_explored
            for i in range(4):
                new_x = curr[0] + dx[i]
                new_y = curr[1] + dy[i]
                new_point = (new_x, new_y)
                if 0 <= new_x < rows and 0 <= new_y < cols and new_point not in parent_map:
                    queue.append(new_point)
                    parent_map[new_point] = curr
        
        return None, nodes_explored

    @staticmethod
    def dfs(rows, cols, start, end):
        dx = [-1, 1, 0, 0]
        dy = [0, 0, -1, 1]
        
        parent_map = {}
        stack = []
        start_point = (start // cols, start % cols)
        end_point = (end // cols, end % cols)
        
        stack.append(start_point)
        parent_map[start_point] = None
        optimal_path = []
        nodes_explored = 0
        
        while stack:
            nodes_explored += 1
            curr = stack.pop()
            if curr == end_point:
                backtrack = curr
                while backtrack is not None:
                    optimal_path.append(backtrack)
                    backtrack = parent_map.get(backtrack)
                optimal_path.reverse()
                return optimal_path, nodes_explored
            for i in range(4):
                new_x = curr[0] + dx[i]
                new_y = curr[1] + dy[i]
                new_point = (new_x, new_y)
                if 0 <= new_x < rows and 0 <= new_y < cols and new_point not in parent_map:
                    stack.append(new_point)
                    parent_map[new_point] = curr
        
        return None, nodes_explored

    @staticmethod
    def cost_search(rows, cols, start, end):
        dx = [-1, 1, 0, 0]
        dy = [0, 0, -1, 1]
        
        parent_map = {}
        queue = []
        start_point = (start // cols, start % cols)
        end_point = (end // cols, end % cols)
        
        heapq.heappush(queue, (0, start_point))
        parent_map[start_point] = (None, 0)
        optimal_path = []
        nodes_explored = 0
        
        while queue:
            nodes_explored += 1
            curr_cost, curr = heapq.heappop(queue)
            if curr == end_point:
                backtrack = curr
                while backtrack is not None:
                    optimal_path.append(backtrack)
                    backtrack, _ = parent_map.get(backtrack)
                optimal_path.reverse()
                return optimal_path, nodes_explored
            for i in range(4):
                new_x = curr[0] + dx[i]
                new_y = curr[1] + dy[i]
                new_point = (new_x, new_y)
                new_cost = curr_cost + 1
                if 0 <= new_x < rows and 0 <= new_y < cols and (new_point not in parent_map or new_cost < parent_map[new_point][1]):
                    heapq.heappush(queue, (new_cost, new_point))
                    parent_map[new_point] = (curr, new_cost)
        
        return None, nodes_explored
    
    

    

    
    
    @staticmethod
    def a_star(grid, start, end):
        
        priority_queue = []  
        visited = set()  
        g_scores = {start: 0}  
        parent_map = {}  
        rows = len(grid)
        cols = len(grid[0])

        
        def heuristic(node):
            x, y = node
            end_x, end_y = end // cols, end % cols
            return abs(end_x - x) + abs(end_y - y)

       
        heapq.heappush(priority_queue, (heuristic(start), start))
        parent_map[start] = None

        
        while priority_queue:
            _, current = heapq.heappop(priority_queue)
            if current == end:
                
                optimal_path = []
                backtrack = current
                while backtrack is not None:
                    optimal_path.append(backtrack)
                    backtrack = parent_map.get(backtrack)
                optimal_path.reverse()
                return optimal_path, len(visited)

            visited.add(current)

           
            x, y = current
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                new_x, new_y = x + dx, y + dy
                new_pos = (new_x, new_y)
                if 0 <= new_x < rows and 0 <= new_y < cols and grid[new_x][new_y] != 1:
                    
                    new_g = g_scores[current] + 1
                    if new_pos not in g_scores or new_g < g_scores[new_pos]:
                        
                        g_scores[new_pos] = new_g
                        heapq.heappush(priority_queue, (new_g + heuristic(new_pos), new_pos))
                        parent_map[new_pos] = current

        return None, len(visited)
    
if __name__ == "__main__":
    print("Enter number of rows:")
    rows = int(input())
    print("Enter number of columns:")
    cols = int(input())
    print("Enter the starting position:")
    start = int(input())
    print("Enter the ending position:")
    end = int(input())

    grid = SearchAlgorithms.grid(rows, cols, start, end)
    SearchAlgorithms.print_grid(grid, start, end, [])

    algorithms = ["BFS", "DFS", "UCS", "Greedy Best-First", "A*"]
    for algorithm in algorithms:
        if algorithm == "BFS":
            optimal_path, nodes_explored = SearchAlgorithms.bfs(rows, cols, start, end)
        elif algorithm == "DFS":
            optimal_path, nodes_explored = SearchAlgorithms.dfs(rows, cols, start, end)
        elif algorithm == "UCS":
            optimal_path, nodes_explored = SearchAlgorithms.cost_search(rows, cols, start, end)
        

        print(f"\nAlgorithm: {algorithm}")
        if optimal_path:
            print("Optimal Path:")
            for point in optimal_path:
                print(point)
            SearchAlgorithms.print_grid(grid, start, end, optimal_path)
            print("Path length:", len(optimal_path))
            print("Nodes explored:", nodes_explored)
        else:
            print("No path exists from S to E.")
