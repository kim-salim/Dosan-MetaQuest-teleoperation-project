#!/usr/bin/env python

import rclpy
from rclpy.executors import ExternalShutdownException

from ros_tcp_endpoint import TcpServer


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")

    tcp_server.start()
    try:
        tcp_server.setup_executor()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tcp_server.destroy_nodes()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
