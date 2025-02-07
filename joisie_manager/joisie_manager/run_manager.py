# from joisie_vision.opencv_model import CV2DetectionNode

import rclpy

def main(args=None):
    rclpy.init(args=args)

    # color_detection_node = CV2DetectionNode()

    # rclpy.spin(color_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # color_detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
