import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker

class visualize(Node):
    def __init__(self):
        super().__init__('visualization_node')
        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        # self.init_anchor()
        self.init_marker()
        self.i = 0
        self.create_subscription(Pose, '/tag/pose', self.handle_tag_pose, 10)

    def init_marker(self):
        self.sphere_list = self.line_strip = Marker()
        self.sphere_list.header.frame_id = self.line_strip.header.frame_id = '/map'
        self.sphere_list.ns = "spheres"
        self.line_strip.ns = "lines"
        self.sphere_list.id = 1
        self.line_strip.id = 2
        self.sphere_list.type = Marker.SPHERE_LIST
        self.line_strip.type = Marker.LINE_STRIP
        self.sphere_list.action = self.line_strip.action = Marker.ADD

    # def init_anchor(self):
    #     self.anchor = Marker()
    #     self.anchor.header.frame_id = '/map'
    #     self.anchor.header.stamp = self.get_clock().now().to_msg()
    #     self.anchor.ns = "anchors"
    #     self.anchor.id = 0
    #     self.anchor.type = Marker.CUBE_LIST
    #     self.anchor.action = Marker.ADD
    #     height = 10.
    #     self.anchor.scale.x = 20.
    #     self.anchor.scale.y = 20.
    #     self.anchor.scale.z = height
    #     self.anchor.color.r = 1.
    #     self.anchor.color.a = 1.

    #     dim = int(input('Dimension # 3 for 3D and 2 for 2D: '))
    #     points = []
    #     for i in range(dim+1):
    #         tmp = [float(x) for x in input('Coordinate {0} # values separated by comma -> x,y,z: '.format(i+1)).strip().split(',')]
    #         points.append(tmp)

    #     for point in points:
    #         A = Point()
    #         A.x = point[0]
    #         A.y = point[1]
    #         A.z = point[2]+height/2
    #         self.anchor.points.append(A)

    def handle_tag_pose(self, data):
        self.sphere_list.header.stamp = self.line_strip.header.stamp = self.get_clock().now().to_msg()
        my_point = Point()
        my_point = data.position

        self.sphere_list.scale.x = 5.
        self.sphere_list.scale.y = 5.
        self.sphere_list.scale.z = 5.
        self.sphere_list.color.b = 1.0
        self.sphere_list.color.a = 1.0

        self.line_strip.scale.x = 5.
        self.line_strip.color.r = 1.0
        self.line_strip.color.a = 1.0
        
        # if self.i > 10:
        #    self.sphere_list.points.pop(0)
        #    self.line_strip.points.pop(0)
            
        self.sphere_list.points.append(my_point)
        self.line_strip.points.append(my_point)
        # self.pub.publish(self.anchor)
        # self.pub.publish(self.sphere_list)
        self.pub.publish(self.line_strip)
        
        self.i += 1

def main():
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
