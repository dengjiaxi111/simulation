"""Publish a SINGLE ground initial pose after LIO is ready; repeated yaw resets are unsafe."""
import time,argparse
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener,TransformException
from gains import require_isolation
p=argparse.ArgumentParser();p.add_argument('--x',type=float,default=0);p.add_argument('--y',type=float,default=0);a=p.parse_args();require_isolation();rclpy.init();n=Node('pid_tuning_initial_pose');n.set_parameters([Parameter('use_sim_time',value=True)]);b=Buffer();l=TransformListener(b,n);pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);start=time.monotonic();ready=False
while time.monotonic()-start<100:
 rclpy.spin_once(n,timeout_sec=.1)
 try:b.lookup_transform('odom','base_links',rclpy.time.Time());ready=pub.get_subscription_count()>0
 except TransformException:pass
 if ready:break
if not ready:raise RuntimeError('LIO ground TF was not ready; no pose sent')
m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.pose.position.x=float(a.x);m.pose.pose.position.y=float(a.y);m.pose.pose.orientation.w=1.;m.pose.covariance[0]=.05;m.pose.covariance[7]=.05;m.pose.covariance[35]=.1;pub.publish(m)
for _ in range(20):rclpy.spin_once(n,timeout_sec=.1)
print('One initial pose sent',a.x,a.y);n.destroy_node();rclpy.shutdown()
