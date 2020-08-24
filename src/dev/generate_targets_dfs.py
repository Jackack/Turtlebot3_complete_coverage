from __future__ import print_function
import numpy as np
from numpy import savetxt
# from geometry_msgs.msg import Quaternion
import math
import copy

class generateTargets():
    width = 0
    height = 0
    resolution = 0.05
    
    def __init__(self):
        pass

    def convert_pgm_to_2Dlist(self):
        pgmf = open('/home/jack/map.pgm', 'rb')
        print(pgmf.readline().decode('utf-8'))
        print(pgmf.readline().decode('utf-8'))    
        (width, height) = [int(i) for i in pgmf.readline().decode('utf-8').split()]
        print(width, height)
        self.width = width
        self.height = height

        depth = int(pgmf.readline().decode('utf-8'))
        assert depth <= 255

        points = []
        for y in range(height):
            row = []
            for x in range(width):
                val = ord(pgmf.read(1))
                if val == 254:
                    val = 0
                else:
                    val = 1
                row.append(val)
            # if y == 200:
            #     print("raw", row)

            points.append(row)

        pgmf.close()
        # print(points)
        return points

    def transform_to_gazebo_frame(self, (x, y)):
      # convert x, y such that the origin matches that of Gazebo's,
      # and that x, y are scaled to real distances in meters
      x = x * self.resolution - 10
      y = -(y - 184.5) * self.resolution  
      return (x, y)

    def expand_obstacle(self, points, radius):
        new_points = [row[:] for row in points]
        for y in range(self.height):
            if y > radius and y < self.height - radius:
                for x in range(self.width):
                    if x > radius and x < self.width - radius:
                        if not self.surrounding_area_is_clear(x, y, points, radius):
                            new_points[y][x] = 1

        return new_points

    def search_2Dlist(self, list, item):
        # print("list", list)
        for row in list:
            try:
                return (row.index(item), list.index(row))
            except ValueError:
                pass

        return "not found"

    def norm(self, x, y):
        return math.sqrt(x**2 + y**2)

    def find_nearest_item(self, points, x, y, item):
        radius = min(min(self.width - 1 - x, x), min(self.height - 1 - y, y))
        
        for y_index in range(y - radius, y + radius):
            for x_index in range(x - radius, x + radius):
                # print(len(points))
                # print(len(points[y]))

                if points[y_index][x_index] == item and (x_index != x or y_index != y):
                    return (x_index, y_index)
        return True

    def surrounding_area_is_clear(self, x, y, points, radius):
        for y_index in range(y - radius, y + radius):
            for x_index in range(x - radius, x + radius):
                if points[y_index][x_index] == 1:
                    return False
        return True

    def find_path_dfs(self):
        map_points = self.expand_obstacle(
            self.convert_pgm_to_2Dlist(), 10)
        source = self.search_2Dlist(map_points, 0)
        path_points = []
        unvisited = [ row for row in map_points]
        current_node = source
        while True:
            if self.search_2Dlist(unvisited, 0) == "not found":
                print("nf")
                #return 0
            print(len(unvisited[current_node[1]]))
            current_node = self.find_nearest_item(unvisited, current_node[0], current_node[1], 0)
            if not unvisited[current_node[0]]:
                break
            unvisited[current_node[1]][]
            path_points.append(self.transform_to_gazebo_frame(current_node) + (0,))
            # print(self.transform_to_gazebo_frame(current_node))
        self.generate_yaml_path(path_points)

    def generate_yaml_path(self, path_points):
      # /home/jack/catkin_ws/src/turtlebot3/dev/nodes/
      with open('/home/jack/catkin_ws/src/turtlebot3/dev/nodes/route.yaml', 'w') as f:
        index = 0
        for point in path_points:
          index += 1    
          print("- {filename: 'p%s', position: { x: %s, y: %s}, rotation: %s}" % (index, point[0], point[1], point[2]), file = f)
      print("File generated!")
      

if __name__ == '__main__':
    g = generateTargets()
    g.find_path_dfs()


