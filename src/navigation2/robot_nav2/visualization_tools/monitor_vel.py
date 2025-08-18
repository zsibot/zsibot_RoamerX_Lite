import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

class CmdVelPlotter(Node):
    def __init__(self):
        super().__init__('cmd_vel_plotter')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription 

        self.history_size = 100
        self.linear_x = deque(maxlen=self.history_size)
        self.linear_y = deque(maxlen=self.history_size)
        self.angular_z = deque(maxlen=self.history_size)
        self.time_steps = deque(maxlen=self.history_size)

        self.counter = 0

        plt.ion()
        self.fig, self.ax = plt.subplots(3, 1, figsize=(6, 8))
        self.lines = []
        labels = ["Linear X (m/s)", "Linear Y (m/s)", "Angular Z (rad/s)"]

        for i in range(3):
            line, = self.ax[i].plot([], [], label=labels[i])
            self.lines.append(line)
            self.ax[i].legend()
            self.ax[i].grid()

        self.get_logger().info("CmdVelPlotter node started")

    def cmd_vel_callback(self, msg):
        """ 处理 /cmd_vel 消息并更新数据 """
        self.linear_x.append(msg.linear.x)
        self.linear_y.append(msg.linear.y)
        self.angular_z.append(msg.angular.z)
        self.time_steps.append(self.counter)
        self.counter += 1

        self.update_plot()

    def update_plot(self):
        """ 实时更新曲线 """
        data_list = [self.linear_x, self.linear_y, self.angular_z]
        
        for i, line in enumerate(self.lines):
            line.set_xdata(self.time_steps)
            line.set_ydata(data_list[i])
            self.ax[i].relim()
            self.ax[i].autoscale_view()

        plt.draw()
        plt.pause(0.01)  

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CmdVelPlotter")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()  

if __name__ == '__main__':
    main()

