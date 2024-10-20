import heapq
import cv2
import numpy as np
start = None
goal = None

def dijkstra(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    pq = []  
    heapq.heappush(pq, (0, start))  
    distances = {start: 0}
    prev = {start: None}
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        if current_node == goal:
            break  
        
        r, c = current_node
        neighbors = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]  
        
        for nr, nc in neighbors:
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0: 
                new_distance = current_distance + 1
                
                if (nr, nc) not in distances or new_distance < distances[(nr, nc)]:
                    distances[(nr, nc)] = new_distance
                    prev[(nr, nc)] = (r, c)
                    heapq.heappush(pq, (new_distance, (nr, nc)))
    

    path = []
    node = goal
    while node:
        path.append(node)
        node = prev.get(node)
    return path[::-1]  

def select_point(event, x, y, flags, param):
    global start, goal, maze_display  

    if event == cv2.EVENT_LBUTTONDOWN:

        grid_x = y // grid_size
        grid_y = x // grid_size

        if start is None:
            start = (grid_x, grid_y)
            print(f'Start point selected: {start}')
            cv2.circle(maze_display, (x, y), 5, (0, 255, 0), -1)  
        elif goal is None:
            goal = (grid_x, grid_y)
            print(f'Goal point selected: {goal}')
            cv2.circle(maze_display, (x, y), 5, (0, 0, 255), -1)  

def main():
    global maze, maze_display, grid_size, start, goal 

    maze = cv2.imread('/home/arpan/map_1728923023.pgm', cv2.IMREAD_GRAYSCALE)
    maze = (maze < 10).astype(np.uint8)  
    print(f'Maze shape: {maze.shape}')  
    maze_display = np.zeros((*maze.shape, 3), dtype=np.uint8) 
    maze_display[maze == 0] = [255, 255, 255]
    maze_display[maze == 1] = [0, 0, 0]        

    grid_size = 1  

    cv2.namedWindow('Maze')
    cv2.setMouseCallback('Maze', select_point)

    while True:
        cv2.imshow('Maze', maze_display)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        if start is not None and goal is not None:
            path = dijkstra(maze, start, goal)
            if path:
                print("Path found:", path)

                for (x, y) in path:
                    maze_display[x * grid_size:(x + 1) * grid_size, y * grid_size:(y + 1) * grid_size] = (127, 127, 127)  

            start = None
            goal = None

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
