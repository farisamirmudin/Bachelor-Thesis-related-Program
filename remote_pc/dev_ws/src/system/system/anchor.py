import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class visualize(Node):
    def __init__(self):
        super().__init__('anchor_node')
        self.pub = self.create_publisher(Marker, 'anchor_marker', 10)
        self.init_anchor()
        self.timer = self.create_timer(0.1, self.anchor_callback)

    def init_anchor(self):
        self.anchor = Marker()
        self.anchor.header.frame_id = '/map'
        self.anchor.header.stamp = self.get_clock().now().to_msg()
        self.anchor.ns = "anchors"
        self.anchor.id = 0
        self.anchor.type = Marker.CUBE_LIST
        self.anchor.action = Marker.ADD
        height = 10.
        self.anchor.scale.x = 5.
        self.anchor.scale.y = 5.
        self.anchor.scale.z = height
        self.anchor.color.r = 1.
        self.anchor.color.a = 1.
        os.chdir(os.path.dirname(__file__)) 
        with open('anchor_coordinate.txt', 'r') as f:
            lines = f.readlines()
            for line in lines:
                point = [float(x) for x in line.strip().split(',')]
                A = Point()
                A.x = point[0]
                A.y = point[1]
                if len(point) == 2:
                    A.z = height/2
                else:
                    A.z = point[2]+height/2
                self.anchor.points.append(A)

        # dim = int(input('Dimension # 3 for 3D and 2 for 2D: '))
        # points = []
        # for i in range(dim+1):
        #     point = [float(x) for x in input('Coordinate {0} # values separated by comma -> x,y,z: '.format(i+1)).strip().split(',')]
        #     points.append(point)


    def anchor_callback(self):
        self.pub.publish(self.anchor)
        
def main():
    os.chdir(os.path.dirname(__file__)) 
    rclpy.init()
    visualiser = visualize()
    try:
        rclpy.spin(visualiser)
    except KeyboardInterrupt:
        pass
    visualiser.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
