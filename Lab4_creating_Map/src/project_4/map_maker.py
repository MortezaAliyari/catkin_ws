from project_4.geometry import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from math import sin, cos, degrees
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ------------------------------------------------------------------------------
# MapMaker class
# ------------------------------------------------------------------------------
class MapMaker:
  def __init__(self, origin_x, origin_y, resolution, size_x, size_y, transformer):
    self.origin_x = origin_x
    self.origin_y = origin_y
    self.resolution = resolution
    self.size_x = size_x
    self.size_y = size_y
    self.transformer = transformer
    
    self.grid = OccupancyGrid()
    self.grid.header.frame_id = 'odom'
    self.grid.info.resolution = resolution
    self.grid.info.width = size_x
    self.grid.info.height = size_y
    self.grid.info.origin.position.x = origin_x
    self.grid.info.origin.position.y = origin_y
    self.grid.info.origin.orientation.w = 1.0
    self.grid.data = [-1] * (size_x * size_y)
    self.numScansReceived = 0

    # Insert additional code here if needed
    self.process_odom_flag = 0

  # ----------------------------------------------------------------------------
  # Convert from world coordinates to grid coordinates. This is convenience 
  # wrapper around the to_grid function from the first part of the assignment.
  # Usage:
  #   (x_grid, y_grid) = self.to_grid(x_world, y_world)
  def to_grid(self, x, y):
    return to_grid(x, y, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

  # ----------------------------------------------------------------------------
  # Convert from grid coordinates to world coordinates. This is convenience 
  # wrapper around the to_world function from the first part of the assignment.
  # Usage:
  #   (x_world, y_world) = self.to_world(x_grid, y_grid)
  def to_world(self, gx, gy):
    return to_world(gx, gy, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

  # ----------------------------------------------------------------------------
  # Process odometry message. You code should go here.
  def process_odom(self, msg):

    position = msg.pose.pose.position
    self.x_now = position.x
    self.y_now = position.y

    orientation = msg.pose.pose.orientation
    orientation = (orientation.x, orientation.y, orientation.z, orientation.w)
    self.theta_now = euler_from_quaternion(orientation)[2]

    # print "-------------------------------------------------------------------------"
    # print "process_odom"
    # print self.theta_now

    self.process_odom_flag = 1

    None
  # ----------------------------------------------------------------------------
  # Process laserscan message. You code should go here.
  def process_scan(self, msg):

    self.numScansReceived+=1
    print (self.numScansReceived)

    i = 0
    rays = []
    free_cells = []
    occupied_cells = []

    if self.process_odom_flag ==1:
      for range_value in msg.ranges:

        angle_to_robot_frame = msg.angle_min + ( i * msg.angle_increment )

        # # Now convert the point to world coordinate system
        x_world = range_value * math.cos(angle_to_robot_frame+self.theta_now) + self.x_now
        y_world = range_value * math.sin(angle_to_robot_frame+self.theta_now) + self.y_now
        # Ray is from robot position to range_value
        rays.append( (x_world,y_world) )

        # Convert points to grid cells
        try:
          (gx_1,gy_1) = self.to_grid(self.x_now, self.y_now)
          (gx_2,gy_2) = self.to_grid(x_world, y_world)
          grid_go_ahead = 1
        except Exception as e:
          print ('oops', Exception) 
          grid_go_ahead = 0
        
        if grid_go_ahead == 1:
          # Set of points on the ray
          points_on_ray = bresenham(gx_1, gy_1, gx_2, gy_2)

          for j in range(len(points_on_ray)):

            point_on_ray_x = points_on_ray[j][0]
            point_on_ray_y = points_on_ray[j][1]

            point_on_ray_index = to_index(point_on_ray_x, point_on_ray_y, self.size_x)

            (point_on_ray_x_world, point_on_ray_y_world) = self.to_world(point_on_ray_x, point_on_ray_y)

            # All points except last is free cell
            if j < ( len(points_on_ray) - 1 ):
              self.grid.data[point_on_ray_index] = 0
            # Last point is occupied cell
            if j == ( len(points_on_ray) - 1 ):
              if range_value >= msg.range_max:
                self.grid.data[point_on_ray_index] = 0
              else:
                self.grid.data[point_on_ray_index] = 100

        i = i +1



    None        

  # ----------------------------------------------------------------------------
  # Visualize robot pose, current laserscan, free map cells and occupied map
  # cells. This function is useful for debugging your code. Feel free to modify
  # this to display additional information.
  # 
  # Parameters:
  #   robot_pose      robot pose represented by a tuple (x, y, theta)
  #   rays            laserscan rays represented as an array of tuples i.e. ((x_0, y_0), ... , (x_n, y_n))
  #   free_cells      grid coordinates of the free cells represented as an array of tuples i.e. ((x_0, y_0), ... , (x_n, y_n))
  #   occupied_cells  grid coordinates of the occupied cells represented as an array of tuples i.e. ((x_0, y_0), ... , (x_n, y_n))
  #   pause           if True this function will stop execution until a key is pressed
  # Usage:
  #   self.visualize_scans(self, robot_pose, rays, free_cells, occupied_cells, pause)
  # If you want to visualize laserscans only and no cell information:
  #   self.visualize_scans(self, robot_pose, rays, [], [], pause)
  def visualize_scans(self, robot_pose, rays, free_cells, occupied_cells, pause):

    # Initialize figure
    my_dpi = 96
    plt.figure(1, figsize=(800/my_dpi, 800/my_dpi), dpi=my_dpi)
    plt.cla()
    plt.xlim ( self.grid.info.origin.position.x, self.grid.info.origin.position.x + self.grid.info.width )
    plt.ylim ( self.grid.info.origin.position.y, self.grid.info.origin.position.y + self.grid.info.height )
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title('Laserscan ' + str(self.numScansReceived))
    plt.xlabel('X world')
    plt.xlabel('Y world')
    ax = plt.axes()

    # Add free cells
    free_cells = self.unique_list(free_cells)
    for cell in free_cells:
      (x, y) = self.to_world(cell[0], cell[1])
      patch = patches.Rectangle ( (x - self.resolution/2, y - self.resolution/2), self.resolution, self.resolution, color='b', alpha=0.5)
      ax.add_patch(patch)

    # Add occupied cells
    occupied_cells = self.unique_list(occupied_cells)
    for cell in occupied_cells:
      (x, y) = self.to_world(cell[0], cell[1])
      patch = patches.Rectangle ( (x - self.resolution/2, y - self.resolution/2), self.resolution, self.resolution, color='k', alpha=0.8)
      ax.add_patch(patch)

    # Add odometry frame
    self.plot_coordinate_frame((0.0, 0.0, 0.0), 4, "odom")

    # Add robot frame
    self.plot_coordinate_frame(robot_pose, 4, "base_link")

    # Add laserscanner rays
    for ray in rays:
      plt.plot([ray[0]], [ray[1]], 'ro', markersize=2, markeredgecolor='r')

    # Add grid (comment this out if your visualization code is too slow)
    ax = plt.axes()
    step = 5
    minor_ticks = range(self.grid.info.width / step)
    for i in range(len(minor_ticks)):
      minor_ticks[i] = minor_ticks[i] * 5 + int(self.grid.info.origin.position.x)
    ax.set_xticks(minor_ticks, minor=True)                                           
    ax.set_yticks(minor_ticks, minor=True) 
    ax.grid(which='minor')                                                            

    # Show plot
    plt.draw()
    if pause:
      k = plt.waitforbuttonpress(1)
      while not k:
        k = plt.waitforbuttonpress(1)
    else:
      plt.waitforbuttonpress(1e-6)

  None  

  # ----------------------------------------------------------------------------
  # Draw a coordinate frame
  def plot_coordinate_frame(self, pose, size, frame_id=""):
    (x, y, theta) = pose
    plt.plot( [x,  x + cos(theta)*size],
              [y,  y + sin(theta)*size],
              'r', linewidth=4)
    plt.plot( [x,  x - sin(theta)*size],
              [y,  y + cos(theta)*size],
              'g', linewidth=4)
    plt.plot(x, y, 'bo', markersize=6, markeredgecolor='b')
    ax = plt.axes()
    ax.text(x, y-1, frame_id)

    None

  # ----------------------------------------------------------------------------
  # Remove duplicate entries from a list
  def unique_list(self, list_in):
    return list(set(list_in))
