from __future__ import print_function
import numpy as np
from numpy import savetxt
# from geometry_msgs.msg import Quaternion
import math

class generateTargets():
    width = 0
    height = 0
    resolution = 0.05
    
    def __init__(self):
        pass

    def convert_pgm_to_dict(self):
        pgmf = open('/home/jack/map.pgm', 'rb')
        print(pgmf.readline().decode('utf-8'))
        print(pgmf.readline().decode('utf-8'))    
        (width, height) = [int(i) for i in pgmf.readline().decode('utf-8').split()]
        print(width, height)
        self.width = width
        self.height = height

        depth = int(pgmf.readline().decode('utf-8'))
        assert depth <= 255

        points = {}
        for y in range(height):
            print(y)
            for x in range(width):
                val = ord(pgmf.read(1))
                if val == 254:
                    val = 0
                else:
                    val = 1
                if y % 2 == 0:
                    points[x, y] = val
                else:
                    points[width - x - 1, y] = val

        pgmf.close()
        #print(points)
        return points

    def transform_to_gazebo_frame(self, x, y):
      # convert x, y such that the origin matches that of Gazebo's,
      # and that x, y are scaled to real distances in meters
      x = x * self.resolution - 10
      y = -(y - 184.5) * self.resolution  
      return (x, y)

    def convert_vector_to_angle(self, deltaX, deltaY):
      print(deltaX, deltaY)
      if deltaY == 0:
        return (0,)
      else:
        return (math.atan2(deltaY, deltaX) + 0.5 * math.pi,)


    def is_too_close_to_nearby(self, x, y, path_points, radius):
      new_point = self.transform_to_gazebo_frame(x, y)
      for point in path_points:
        if math.sqrt( ((new_point[0]-point[0])**2)+((new_point[1]-point[1])**2) ) < radius:
          return True
      return False

    def generate_targets(self):
        #all available points 
        points = self.convert_pgm_to_dict()
        path_points = []
        # for item in sorted(points.iteritems()):
        #     x = item['x']
        #     y = item['y']
        #     if points[x, y] == 0 and points[x - 4, y - 4] == 0 and points[x + 4, y - 4] == 0 and points[x - 4, y + 4] == 0 and points[x + 4, y + 4] == 0 and not self.is_too_close_to_nearby(x, y, path_points, 0.1):
        #       # print(x, y, self.transform_to_gazebo_frame(x, y))   
        #     	gazeboPoint = self.transform_to_gazebo_frame(x, y)
        #         path_points.append(gazeboPoint + (0,))
        #         print("target:", x, y , gazeboPoint)
        # self.generate_yaml_path(path_points)
        for y in range(self.height):
            for x in range(self.width):
                print(x, y)
                if points[x, y] == 0 and points[x - 4, y - 4] == 0 and points[x + 4, y - 4] == 0 and points[x - 4, y + 4] == 0 and points[x + 4, y + 4] == 0 and not self.is_too_close_to_nearby(x, y, path_points, 0.1):
                    # print(x, y, self.transform_to_gazebo_frame(x, y))   
                    gazeboPoint = self.transform_to_gazebo_frame(x, y)
                    path_points.append(gazeboPoint + (0,))
                    print("target:", x, y , gazeboPoint)
        self.generate_yaml_path(path_points)
        return path_points

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
    g.generate_targets()


