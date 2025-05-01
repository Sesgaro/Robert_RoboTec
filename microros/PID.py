# publish_positions.py
import rclpy
from std_msgs.msg import Float32MultiArray

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('position_publisher')
    publisher = node.create_publisher(Float32MultiArray, 'target_positions', 10)
    msg = Float32MultiArray()
    msg.data = [90.0, 90.0, 90.0, 90.0]  # Ejemplo: 90° para cada llanta
    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info('Publicando posiciones: %s' % str(msg.data))
        rclpy.spin_once(node, timeout_sec=0.1)

if __name__ == '__main__':
    main()
