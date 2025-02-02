#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from drone.Connection import Connection
from drone.StreamCam import StreamCam
import threading
from drone.utility import kill_process_on_port

def main(args=None):
    rclpy.init(args=args)

    kill_process_on_port(5001)
    kill_process_on_port(5010)
    kill_process_on_port(5005)

    connection_node = Connection("0.0.0.0")
    drone_node = connection_node.drone
    camera_node = StreamCam()

    executor = MultiThreadedExecutor()
    executor.add_node(connection_node)
    executor.add_node(drone_node)
    executor.add_node(camera_node)

    connection_thread = threading.Thread(target=connection_node.start_server, daemon=True)
    connection_thread.start()

    communication_thread = threading.Thread(target=connection_node.make_basic_connect, daemon=True)
    communication_thread.start()

    camera_thread = threading.Thread(target=camera_node.start_server, daemon=True)
    camera_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        connection_node.get_logger().info("Shutting down connection node.")
    finally:
        connection_node.destroy_node()
        drone_node.destroy_node()
        camera_node.destroy_node()
        rclpy.shutdown()
