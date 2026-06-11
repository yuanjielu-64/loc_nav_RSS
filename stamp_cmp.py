import rclpy, time, statistics
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
n = Node('stamp_cmp')

latest_tf = {'stamp': None, 'wall': None}
scan_vs_tf = []     # scan_stamp - latest_tf_stamp
scan_vs_wall = []   # wall - scan_stamp
tf_vs_wall = []     # wall - tf_stamp

def tf_cb(m):
    wall = time.time()
    for tr in m.transforms:
        if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
            st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            latest_tf['stamp'] = st
            latest_tf['wall'] = wall
            tf_vs_wall.append((wall - st) * 1000)

def scan_cb(m):
    wall = time.time()
    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    scan_vs_wall.append((wall - st) * 1000)
    if latest_tf['stamp'] is not None:
        scan_vs_tf.append((st - latest_tf['stamp']) * 1000)

n.create_subscription(TFMessage, '/tf', tf_cb, 50)
n.create_subscription(LaserScan, '/front/scan', scan_cb, qos_profile_sensor_data)

t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.05)

def show(name, arr, hint):
    if arr:
        print(f"{name}: 平均 {statistics.mean(arr):7.1f} ms  范围[{min(arr):7.1f}, {max(arr):7.1f}]  ({len(arr)}个)  {hint}")
    else:
        print(f"{name}: 无数据  {hint}")

print("=== 时间戳对比 ===")
show("墙钟 - TF戳     ", tf_vs_wall,   "(TF 延迟)")
show("墙钟 - 激光戳   ", scan_vs_wall, "(激光延迟)")
show("激光戳 - TF戳   ", scan_vs_tf,   "<<< 若为负且|值|大，说明激光戳落后于TF, 正是RViz报错根源")

rclpy.shutdown()

