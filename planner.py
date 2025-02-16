import heapq
import numpy as np
import time
from multiprocessing import Queue

def a_star(map, start, end):
    class Node:
        def __init__(self, row, col, obstacle):
            self.g = float('inf')
            self.h = 0
            self.row = row
            self.col = col
            self.obstacle = obstacle
            self.prev = None
        
        def __lt__(self, other):
            return (self.g + self.h) < (other.g + other.h)
        
        def __repr__(self):
            return f"({self.row}, {self.col}, {self.g + self.h}, {self.obstacle})"

    def h_value(s, d):
        return (s.row - d.row) ** 2 + (s.col - d.col) ** 2

    rows, cols = map.shape
    open = []
    closed = set()
    grid = [
        [Node(row, col, map[row, col] == 1) for col in range(cols)]
        for row in range(rows)
    ]
    start_node = grid[start[0]][start[1]]
    # ~ print("@@@@@", end, len(end))
    # ~ print("@@@@@", grid[0][0], grid[100][0])
    # ~ print("@@@@@", end[0], end[1])
    
    end_node = grid[end[0]][end[1]]

    start_node.g = 0
    start_node.h = h_value(start_node, end_node)

    heapq.heappush(open, start_node)

    # ~ print("@@@@@ grid", grid)

    while open:
        current = heapq.heappop(open)
        if (current.row, current.col) in closed:
            continue
        # ~ print("aaa")
        if current.row == end_node.row and current.col == end_node.col:
            path = []
            while current:
                path.append([current.row, current.col])
                current = current.prev
            path.reverse()
            return path
        # ~ print("bbb")

        closed.add((current.row, current.col))
        # ~ print("ccc")

        for rr, cc in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            new_row = current.row + rr
            new_col = current.col + cc

            if new_row < 0 or new_row >= rows or new_col < 0 or new_col >= cols:
                continue
            # ~ print("ddd")
            adj = grid[new_row][new_col]
            if adj.obstacle or ((adj.row, adj.col) in closed):
                continue
            # ~ print("eee")
            if (current.g + 1) < adj.g:
                adj.prev = current
                adj.g = current.g + 1
                adj.h = h_value(adj, end_node)
                heapq.heappush(open, adj)
            # ~ print("fff")

    return []


def test():
    print(">>> Running example test case")
    map_array = np.array([
        [0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 1, 0],
        [1, 1, 0, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
        [0, 1, 0, 0, 0, 0, 1, 0, 1, 1],
        [0, 1, 1, 1, 1, 0, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 1, 1, 0]
    ])
    print("Map:\n", map_array)
    start = [8, 0]
    end = [0, 0]

    print("Start:", start)
    print("End:", end)

    path = a_star(map_array, start, end)
    print(">>> Result")
    print("Path:" if path else "No path")
    print(path)

if __name__ == '__main__':
    test()
