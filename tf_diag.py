import rclpy, time, statistics
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

rclpy.init()
n = Node('tf_diag')

edge_stamps = {}
edge_count = {}

def cb(m):
    wall = time.time()
    for tr in m.transforms:
        key = (tr.header.frame_id, tr.child_frame_id)
        st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        edge_stamps.setdefault(key, []).append((wall, st))
        edge_count[key] = edge_count.get(key, 0) + 1

n.create_subscription(TFMessage, '/tf', cb, 50)
n.create_subscription(TFMessage, '/tf_static', cb, 50)

t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.1)

print("=== 所有 TF 边（parent -> child）及频率 ===")
for key, cnt in sorted(edge_count.items()):
    rate = cnt / 5.0
    print(f"  {key[0]:>20s} -> {key[1]:<20s}  {rate:6.1f} Hz  ({cnt})")

print("\n=== odom->base_link 详细分析 ===")
key = ('odom', 'base_link')
if key in edge_stamps:
    data = edge_stamps[key]
    stamps = [d[1] for d in data]
    skews = [(d[0]-d[1])*1000 for d in data]
    print(f"  条数: {len(data)}")
    print(f"  墙钟-时间戳(skew): 平均 {statistics.mean(skews):.1f} ms 范围[{min(skews):.1f}, {max(skews):.1f}]")
    non_mono = sum(1 for i in range(1,len(stamps)) if stamps[i] < stamps[i-1])
    print(f"  时间戳回退次数: {non_mono} / {len(stamps)-1}  (>0 说明多个时钟抢这条TF)")
    print(f"  时间戳跨度: {max(stamps)-min(stamps):.3f} s")
else:
    print("  没有直接 odom->base_link 边（多级链）")

rclpy.shutdown()

