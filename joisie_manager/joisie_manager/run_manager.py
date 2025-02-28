# from joisie_vision.opencv_model import CV2DetectionNode

import rclpy
from taskmanager import *

def main(args=None):
    rclpy.init(args=args)

    manager_node = TaskManagerNode()

    rclpy.spin(manager_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
