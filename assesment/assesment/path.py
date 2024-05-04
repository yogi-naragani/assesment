import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import logging
import numpy as np
import math


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            "map_topic",  
            self.map_callback,
            10,
        )
        self.path_publisher = self.create_publisher(
            Path, "path_topic", 10  
        )

    def map_callback(self, msg):

        occupancy_grid = msg

        path = self.plan_path(occupancy_grid)
        
        self.path_publisher.publish(path)


        

    def plan_path(self, occupancy_grid):

        data = np.array(occupancy_grid.data).reshape((occupancy_grid.info.height, occupancy_grid.info.width))
        data = np.where(data == 0 , -1, -2)
        data = data.astype(np.int32)

        start_pose_point = (-6.29, -2.96)  
        end_pose_point = (0.477, 1.55)  
        
        start_pose = (math.floor((10 + start_pose_point[0])/0.05), math.floor((10 + start_pose_point[1])/0.05))
        end_pose = (math.floor((10 + end_pose_point[0])/0.05), math.floor((10 + end_pose_point[1])/0.05))
        path = self.grassfire(data, start_pose, end_pose)

        
       
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for pose in path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = float( math.floor(-10+pose[0]*0.05))
            pose_stamped.pose.position.y = float(math.floor(-10+pose[1]*0.05))
            ros_path.poses.append(pose_stamped)
        return ros_path

    def grassfire(self, grid, start, end):
    
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        queue = [start]
        grid[start[1] ,start[0]] = 0
        width, height = grid.shape
        while queue:
    	    current_cell = queue.pop(0)
    	    current_distance = grid[current_cell[1], current_cell[0]]
    	    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor_cell = (current_cell[0] + dx, current_cell[1] + dy)
                if 0 <= neighbor_cell[0] < width and 0 <= neighbor_cell[1] < height and grid[neighbor_cell[1], neighbor_cell[0]] != -2:
                    if grid[neighbor_cell[1], neighbor_cell[0]] == -1:
                        grid[neighbor_cell[1], neighbor_cell[0]] = current_distance + 1
                        queue.append(neighbor_cell)


        # Reconstruct path
        goal = end
        path = [end]
        while grid[goal[1], goal[0]] > 0:
            for dx, dy in directions:
                nx, ny = goal[0]+dx, goal[1]+dy
                if 0 <= nx < width and 0 <= ny < height and grid[ny][nx] == grid[goal[1], goal[0]] - 1:
                    path.append((nx, ny))
                    goal = (nx, ny)
                    break

        path.reverse()
        return path

def main(args=None):
    rclpy.init(args=args)
    path_planning_node = PathPlanningNode()
    rclpy.spin(path_planning_node)
    path_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

