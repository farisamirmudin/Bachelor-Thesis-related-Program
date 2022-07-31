import rclpy
from rclpy.node import Node
import socket
import pickle
from geometry_msgs.msg import Pose


class PublishData(Node):

    def __init__(self):
        super().__init__('read_node')
        self.publisher = self.create_publisher(Pose, '/tag/pose', 10)


    def pub_tag(self, data):
        p = Pose()
        p.position.x = data[0]
        p.position.y = data[1]
        if len(data)==2:
            p.position.z = 0.
        else:
            p.position.z = data[2]
        self.publisher.publish(p)
        self.get_logger().info(f'Position = {[round(x,2) for x in data]}')


def main():
    rclpy.init()

    publisher = PublishData()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('192.168.1.50', 9000))
    try:
        while rclpy.ok():
            msg = s.recv(2048)
            data = pickle.loads(msg)
            if isinstance(data[0], list):
                with open('anchor_coordinate', 'w+') as f:
                    for row in data:
                        f.write(','.join(list(map(str, row))) + '\n')
            publisher.pub_tag(data)
    except KeyboardInterrupt:
        pass

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
