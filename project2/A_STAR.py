import heapq
import math



# 启发式函数
def heuristic_manhattan(a, b):
    """曼哈顿距离"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def heuristic_euclidean(a, b):
    """欧氏距离"""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

#八方向
def heuristic_chebyshev(a, b):
    """切比雪夫距离"""
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))



# 获取邻居节点
def get_neighbors(node, grid, mode=4):
    """
    mode=4 -> 四方向
    mode=8 -> 八方向
    """

    if mode == 4:
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    else:
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

    neighbors = []
    #防止越界
    rows = len(grid)
    cols = len(grid[0])
    #遍历所有可能的方向找全所有可能点
    for d in directions:
        nx = node[0] + d[0]
        ny = node[1] + d[1]

        if 0 <= nx < rows and 0 <= ny < cols:
            if grid[nx][ny] == 0:  # 不是障碍物
                neighbors.append((nx, ny))

    return neighbors



# 路径回溯
def reconstruct_path(parent, current):
    """
    根据父节点字典重建从起点到当前节点的完整路径
    参数:
        parent: 字典类型，记录每个节点的父节点关系 {node: parent_node}
        current: 当前节点坐标，通常是终点
    返回:
        list: 从起点到当前节点的完整路径列表，每个元素为节点坐标元组
    """
    path = []
    # 逆向回溯父节点，构建反向路径
    while current in parent:
        path.append(current)
        current = parent[current]

    # 添加起始节点并反转路径得到正确顺序
    path.append(current)
    path.reverse()

    return path




# A*算法核心
def astar(grid, start, goal, heuristic_func, mode=4):
    open_list = []
    heapq.heappush(open_list, (0, start))
    parent = {}
    g_score = {start: 0}
    f_score = {start: heuristic_func(start, goal)}
    closed_set = set()

    while open_list:

        current = heapq.heappop(open_list)[1]

        if current == goal:
            return reconstruct_path(parent, current)

        closed_set.add(current)
        neighbors = get_neighbors(current, grid, mode)

        for neighbor in neighbors:
            if neighbor in closed_set:  #若邻居已访问过，跳过该节点
                continue
            tentative_g = g_score[current] + 1  #计算从起点经当前节点到该邻居的实际代价+1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:  #若邻居未访问过或找到更优路径（g 值更小）
                parent[neighbor] = current  #记录邻居的父节点为当前节点
                g_score[neighbor] = tentative_g #更新邻居的 g 值
                f_score[neighbor] = tentative_g + heuristic_func(neighbor, goal)    #更新邻居的 f 值（g 值 + 启发式估计值）
                heapq.heappush(open_list, (f_score[neighbor], neighbor))    #将邻居加入开放列表，优先级为 f 值

    return None



# 打印地图路径
# def print_path(grid, path):
#     grid_copy = [row[:] for row in grid]
#
#     for x, y in path:
#         grid_copy[x][y] = "*"
#
#     for row in grid_copy:
#         print(row)



def print_path(grid, path, start, goal):

    rows = len(grid)
    cols = len(grid[0])

    display = [['.' for _ in range(cols)] for _ in range(rows)]

    # 障碍物
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                display[i][j] = '#'

    # 路径
    for x, y in path:
        display[x][y] = '*'

    # 起点终点优先显示
    sx, sy = start
    gx, gy = goal

    display[sx][sy] = 'S'
    display[gx][gy] = 'G'

    print("\nPath Map:\n")

    # 打印列坐标
    print("    ", end="")
    for i in range(cols):
        print(f"{i:3}", end="")
    print()

    # 打印地图
    for i in range(rows):
        print(f"{i:3} |", end="")
        for j in range(cols):
            print(f"{display[i][j]:3}", end="")
        print()



# 主函数
def main():
    grid = [
        [0, 0, 0, 0, 1],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0]
    ]#地图
    grid2 = [

        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0],
        [0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0],
        [1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    ]
    start = (0, 0)
    goal = (4, 4)
    print("使用曼哈顿距离:")

    path = astar(
        grid,
        start,
        goal,
        heuristic_manhattan,
        mode=4
    )

    print("路径:", path)
    # print("地图路径:")
    # print_path(grid, path)
    print_path(grid, path, start, goal)



if __name__ == "__main__":
    main()