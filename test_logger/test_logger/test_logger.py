# my_pkg/my_node.py
import rclpy
from rclpy.node import Node

import sys
import os

from Logger import LogType, VectorLogType, Logger, install_shutdown_logging


class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")
        self.time_logtype = LogType("time", 0)

        self.x_logtype    = LogType("x",    1)
        self.y_logtype    = LogType("y",    2)
        self.z_logtype    = LogType("z",    3)
        self.yaw_logtype  = LogType("psi", 4)

        self.xref_logtype = LogType("x_ref", 5)
        self.yref_logtype = LogType("y_ref", 6)
        self.zref_logtype = LogType("z_ref", 7)
        self.yawref_logtype = LogType("psi_ref", 8)

        self.input_logtype = VectorLogType("input", 5, ["force", "moment_x", "moment_y", "moment_z"])

        self.t = 0.0
        self.timer = self.create_timer(0.01, self.step)

    def step(self):
        self.t += 0.01
        self.time_logtype.append(self.t)

        self.x_logtype.append(1.0)
        self.y_logtype.append(2.0)
        self.z_logtype.append(3.0)
        self.yaw_logtype.append(0.0)

        self.xref_logtype.append(1.0)
        self.yref_logtype.append(2.0)
        self.zref_logtype.append(3.0)
        self.yawref_logtype.append(0.0)

        self.input_logtype.append(1,2,3,4)

        if self.t >= 1.0:
            # raise ValueError("stop")
            exit(0)

def main():
    import sys, os
    rclpy.init()
    node = OffboardControl()

    filename = sys.argv[1] if len(sys.argv) > 1 else "log.log"
    base_path = os.path.dirname(os.path.abspath(__file__))  # Get the script's directory
    base_dir = sys.argv[2] if len(sys.argv) > 2 else base_path
    log = Logger(filename, base_dir)

    install_shutdown_logging(log, node)#, also_shutdown=rclpy.shutdown)# Ensure logs are flushed on Ctrl+C / SIGTERM / normal exit
    try:
        rclpy.spin(node)
    finally:
        # belt & suspenders
        pass

if __name__ == "__main__":
    main()
