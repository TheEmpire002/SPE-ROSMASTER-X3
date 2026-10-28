import time, rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node("kurzcheck"); c = {"volt": 0, "scan": 0, "imu": 0, "vel": 0}; last = {}
def cb(k):
    def f(m): c[k] += 1; last[k] = m
    return f
n.create_subscription(Float32, "/voltage", cb("volt"), 10)
n.create_subscription(LaserScan, "/scan", cb("scan"), qos_profile_sensor_data)
n.create_subscription(Imu, "/imu/data_raw", cb("imu"), 10)
n.create_subscription(Twist, "/vel_raw", cb("vel"), 10)
t = time.time()
while time.time() - t < 8: rclpy.spin_once(n, timeout_sec=0.1)
print({k: round(v / 8, 1) for k, v in c.items()}, "Akku", last.get("volt") and round(last["volt"].data, 2))
