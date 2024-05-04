import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
import cv2 as cv
from std_msgs.msg import String

map_path = 'map.pgm'


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.declare_parameter('image', '/home/yogi/map.pgm')
		
		
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map_topic',10)
        self.map_callback()
        
        
        
        
    def map_callback(self):
    	my_param = self.get_parameter("image").get_parameter_value().string_value
    	self.get_logger().info(my_param)
    	occupancy_grid = self.read_pgm(my_param)
    	self.map_publisher.publish(occupancy_grid)
    
        
        
    def read_pgm(self, file_path):
        pgm_image = cv.imread(file_path,  cv.IMREAD_GRAYSCALE)


        data = np.asarray(pgm_image)
        data = np.where(data > 180, 0, -2).astype(np.int8)
        width, height = data.shape
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.resolution = 0.05  
        occupancy_grid.info.origin.position.x = -10.000000
        occupancy_grid.info.origin.position.y = -10.000000
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.data = data.flatten().tolist()

        return occupancy_grid


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


