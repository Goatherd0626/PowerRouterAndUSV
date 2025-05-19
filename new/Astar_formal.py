"""

A* grid planning

"""

import math

# import matplotlib.pyplot as plt
import numpy as np

show_animation = False

# 海里与米转换系数，1海里=1852米
nm2m = 1852.0


def rdp(points, epsilon):
    if len(points) < 3:
        return points

    # Find the point with the maximum distance from the line segment
    start, end = points[0], points[-1]
    max_dist, index = 0, 0
    for i in range(1, len(points) - 1):
        dist = abs((end[1] - start[1]) * points[i][0] - (end[0] - start[0]) * points[i][1] + end[0] * start[1] - end[1] * start[0]) / math.hypot(end[0] - start[0], end[1] - start[1])
        if dist > max_dist:
            max_dist, index = dist, i

    # If max distance is greater than epsilon, recursively simplify
    if max_dist > epsilon:
        left = rdp(points[:index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [start, end]

class AStarPlanner:

    def __init__(self, obstacles, resolution=100, rr=100):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m],地图的像素
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0 # 地图实际坐标范围，m
        self.max_x, self.max_y = 225 * nm2m, 130 * nm2m # 225海里，130海里
        # self.obstacle_map = None
        # self.x_width, self.y_width = 0, 0
        # 地图的像素数
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # obstacle map generation
        self.obstacle_map = [[0 for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        self.motion = self.get_motion_model()
        self.input_obstacle(obstacles)
        # self.calc_obstacle_map(ox, oy)

    class Node:
        """定义搜索区域节点类,每个Node都包含坐标x和y, 移动代价cost和父节点索引。
        """
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        输入起始点和目标点的坐标(sx,sy)和(gx,gy)，
        最终输出的结果是路径包含的点的坐标集合rx和ry。
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]


            # 通过追踪当前位置current.x和current.y来动态展示路径寻找
            if current.x == goal_node.x and current.y == goal_node.y:
                # print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        
        simplified_route = self.astar_route_simplify(rx, ry)

        return simplified_route # 单位是米

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        # 将rx、ry反序
        rx.reverse()
        ry.reverse()
                
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        """计算启发函数

        Args:
            n1 (_type_): _description_
            n2 (_type_): _description_

        Returns:
            _type_: _description_
        """
        w = 1.5  # TODO: weight of heuristic,加速参数
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y] == 1:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[0 for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr + self.resolution:
                        self.obstacle_map[ix][iy] = 1
                        break
        print(self.obstacle_map)
        # print(self.obstacle_map)
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        
        # motion = [[5, 0, 5],
        #           [0, 5, 5],
        #           [-5, 0, 5],
        #           [0, -5, 5],
        #           [-5, -5, 5*math.sqrt(2)],
        #           [-5, 5, 5*math.sqrt(2)],
        #           [5, -5, 5*math.sqrt(2)],
        #           [5, 5, 5*math.sqrt(2)]]
        return motion

    def input_obstacle(self, obstacles):
        for polygon in obstacles:
            # Extract the x and y coordinates of the polygon vertices
            px, py = zip(*polygon)
            # Find the bounding box of the polygon
            min_px, max_px = min(px), max(px)
            min_py, max_py = min(py), max(py)
            
            # Iterate through the bounding box
            for ix in range(self.calc_xy_index(min_px, self.min_x), self.calc_xy_index(max_px, self.min_x) + 1):
                for iy in range(self.calc_xy_index(min_py, self.min_y), self.calc_xy_index(max_py, self.min_y) + 1):
                    x = self.calc_grid_position(ix, self.min_x)
                    y = self.calc_grid_position(iy, self.min_y)
                    
                    # Check if the point is inside the polygon
                    if self.is_point_in_polygon(x, y, polygon):
                        self.obstacle_map[ix][iy] = 1
        
        return self.obstacle_map

    def is_point_in_polygon(self, x, y, polygon):
        """
        使用射线法判断一个点是否在多边形内部。

        参数:
            x (float): 要检查的点的x坐标。
            y (float): 要检查的点的y坐标。
            polygon (list of tuples): 表示多边形顶点的(x, y)元组列表。

        返回:
            bool: 如果点在多边形内返回True，否则返回False。
        """
        n = len(polygon)  # 多边形顶点的数量
        inside = False  # 初始化标志为False，表示点不在多边形内
        px, py = x, y  # 要检查的点的坐标

        # 遍历多边形的每条边
        for i in range(n):
            x1, y1 = polygon[i]  # 边的起点
            x2, y2 = polygon[(i + 1) % n]  # 边的终点（循环到第一个顶点）

            # 检查点是否在边的垂直范围内，并且在边的左侧
            if min(y1, y2) < py <= max(y1, y2) and px <= max(x1, x2):
                if y1 != y2:  # 避免水平边导致的除零错误
                    # 计算边与水平线py的交点的x坐标
                    xinters = (py - y1) * (x2 - x1) / (y2 - y1) + x1
                    # 如果点在交点的左侧，切换inside标志
                    if x1 == x2 or px <= xinters:
                        inside = not inside

        return inside

    def astar_route_simplify(self, rx, ry):
        """
        对路径进行简化，去除冗余点

        :param rx: x坐标列表
        :param ry: y坐标列表
        :return: 简化后的x和y坐标列表
        """
        # 使用Ramer-Douglas-Peucker算法进行路径简化
        points = list(zip(rx, ry))
        simplified_points = [points[0]]  # 保留起点
        simplified_points = rdp(points, epsilon=1.5 * self.resolution)
        return simplified_points


    def input_energyIsland(self, energy_islandxy):
        '''
        输入若干能量岛xy坐标[[x1,y1],[x2,y2],...,[xn,yn]],加入到obstacle_map中，赋值为2
        :param energy_islandxy: 能量岛坐标
        '''
        for i in range(len(energy_islandxy)):
            x = self.calc_xy_index(energy_islandxy[i][0], self.min_x)
            y = self.calc_xy_index(energy_islandxy[i][1], self.min_y)
            if 0 <= x < self.x_width and 0 <= y < self.y_width:
                self.obstacle_map[x][y] = 2
        return self.obstacle_map


# def main():
#     print(__file__ + " start!!")

#     # start and goal position
#     sx = 10.0  # [m]
#     sy = 10.0  # [m]
#     gx = 50.0  # [m]
#     gy = 50.0  # [m]
#     grid_size = 2.0  # [m]
#     robot_radius = 1.0  # [m]

#     # set obstacle positions
#     ox, oy = [], []
#     for i in range(-10, 60):
#         ox.append(i)
#         oy.append(-10.0)
#     for i in range(-10, 60):
#         ox.append(60.0)
#         oy.append(i)
#     for i in range(-10, 61):
#         ox.append(i)
#         oy.append(60.0)
#     for i in range(-10, 61):
#         ox.append(-10.0)
#         oy.append(i)
#     for i in range(-10, 40):
#         ox.append(20.0)
#         oy.append(i)
#     for i in range(0, 40):
#         ox.append(40.0)
#         oy.append(60.0 - i)

#     if show_animation:  # pragma: no cover
#         plt.plot(ox, oy, ".k")
#         plt.plot(sx, sy, "og")
#         plt.plot(gx, gy, "xb")
#         plt.grid(True)
#         plt.axis("equal")

#     a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
#     rx, ry = a_star.planning(sx, sy, gx, gy)

#     if show_animation:  # pragma: no cover
#         plt.plot(rx, ry, "-r")
#         plt.pause(0.001)
#         plt.show()

#     plt.imshow(a_star.obstacle_map)
#     plt.show()
    
# def main2():
#     print(__file__ + " start!!")

#     # start and goal position
#     sx = 100.0  # [m]
#     sy = 100.0  # [m]
#     gx = 50000.0  # [m]
#     gy = 50000.0  # [m]

#     # 给我一组障碍物的坐标，表示为多边形的顶点，边长不小于10000m
    
#     obstacles = [
#         [(20000, 20000), (20000, 30000), (30000, 30000), (30000, 20000)],
#         [(35000, 35000), (35000, 45000), (45000, 45000), (45000, 35000)],
#     ]

#     if show_animation:  # pragma: no cover
#         # 画出obstacles
#         for polygon in obstacles:
#             px, py = zip(*polygon)
#             plt.fill(px, py, "k", alpha=0.5)
#         plt.plot(sx, sy, "og")
#         plt.plot(gx, gy, "xb")
#         plt.grid(True)
#         plt.axis("equal")

#     a_star = AStarPlanner(obstacles)
#     route = a_star.planning(sx, sy, gx, gy)
#     route = np.array(route)
#     # simplified_route = a_star.astar_route_simplify(rx, ry)
#     print(route)  # 打印路径坐标

#     if show_animation:  # pragma: no cover
#         plt.plot(route[:,0], route[:,1], "-r")
#         plt.pause(0.001)
#         plt.show()

# if __name__ == '__main__':
#     main2()


