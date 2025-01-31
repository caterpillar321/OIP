#!/usr/bin/env python3

import rclpy
from drone.Connection import Connection


def main(args=None):
    rclpy.init(args=args)

    connection_node = Connection("0.0.0.0")

    try:
        connection_node.start_server()
        connection_node.make_basic_connect()
        rclpy.spin(connection_node)
    except KeyboardInterrupt:
        connection_node.get_logger().info("Shutting down connection node.")
    finally:
        connection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()