# my_pkg/my_node.py
import rclpy
from rclpy.node import Node

import os
import sys
import inspect
import argparse
import traceback

from Logger import LogType, VectorLogType, Logger # pyright: ignore[reportAttributeAccessIssue]

class OffboardControl(Node):
    def __init__(self):
        super().__init__("offboard_control")
        self.time_logtype = LogType("time", 0)

        self.x_logtype    = LogType("x",    1)
        self.y_logtype    = LogType("y",    2)
        self.z_logtype    = LogType("z",    3)
        self.yaw_logtype  = LogType("yaw", 4)

        self.xref_logtype = LogType("x_ref", 5)
        self.yref_logtype = LogType("y_ref", 6)
        self.zref_logtype = LogType("z_ref", 7)
        self.yawref_logtype = LogType("yaw_ref", 8)

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


BANNER = "=" * 65

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument("--log-file",
                        required=True)
    args, unknown = parser.parse_known_args(sys.argv[1:])
    print(f"Arguments: {args}, Unknown: {unknown}")
    filename = args.log_file
    base_path = os.path.dirname(os.path.abspath(__file__))  # Get the script's directory
    print(f"{filename=}, {base_path=}")

    rclpy.init()
    node = OffboardControl()
    logger = None

    def shutdown_logging(*args):
        print("\nInterrupt/Error/Termination Detected, Triggering Logging Process and Shutting Down Node...")

        try:
            if logger:
                logger.log(node)
            node.destroy_node()
        except Exception as e:
            frame = inspect.currentframe()
            func_name = frame.f_code.co_name if frame is not None else "<unknown>"
            print(f"\nError in {__name__}:{func_name}: {e}")
            traceback.print_exc()


    try:
        print(f"{BANNER}\nInitializing ROS 2 node\n{BANNER}")
        logger = Logger(filename, base_path) # pyright: ignore[reportCallIssue]
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected (Ctrl+C), exiting...")
    except Exception as e:
            frame = inspect.currentframe()
            func_name = frame.f_code.co_name if frame is not None else "<unknown>"
            print(f"\nError in {__name__}:{func_name}: {e}")
            traceback.print_exc()
    finally:
        shutdown_logging()
        print("\nNode has shut down.")



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
            frame = inspect.currentframe()
            func_name = frame.f_code.co_name if frame is not None else "<unknown>"
            print(f"\nError in {__name__}:{func_name}: {e}")
            traceback.print_exc()
