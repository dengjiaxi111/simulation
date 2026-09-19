"""Single TF/odometry fixture for ISOLATED Nav2 plant tests, not LIO validation.
Never run beside LIO: it would duplicate odom->base_link ownership.
"""
import copy,math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from gains import require_isolation
from trial import yaw
class Fixture(Node):
 def __init__(s):
  super().__init__('pid_tuning_truth_fixture');s.set_parameters([Parameter('use_sim_time',value=True)]);s.j=None;s.b=TransformBroadcaster(s);s.static=StaticTransformBroadcaster(s);s.pub=s.create_publisher(Odometry,'/Odometry',10)
  s.create_subscription(JointState,'/joint_states',s.js,qos_profile_sensor_data);s.create_subscription(Odometry,'/wheel/odometry',s.odom,qos_profile_sensor_data)
  t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;s.static.sendTransform(t)
 def js(s,m):
  if 'gimbal_yaw_joint' in m.name:s.j=m.position[m.name.index('gimbal_yaw_joint')]
 def odom(s,m):
  if s.j is None:return
  angle=yaw(m.pose.pose.orientation)+1.6032+s.j;out=[]
  for parent,child,a in [('odom','base_link',angle),('base_link','base_link_static',-angle)]:
   t=TransformStamped();t.header=copy.deepcopy(m.header);t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.z=math.sin(a/2);t.transform.rotation.w=math.cos(a/2)
   if parent=='odom':t.transform.translation.x=m.pose.pose.position.x;t.transform.translation.y=m.pose.pose.position.y;t.transform.translation.z=.30235
   out.append(t)
  s.b.sendTransform(out)
  o=copy.deepcopy(m);o.child_frame_id='base_link';o.pose.pose.position.z=.30235;o.pose.pose.orientation=copy.deepcopy(out[0].transform.rotation);delta=1.6032+s.j;c=math.cos(delta);sn=math.sin(delta);x=m.twist.twist.linear.x;y=m.twist.twist.linear.y;o.twist.twist.linear.x=c*x+sn*y;o.twist.twist.linear.y=-sn*x+c*y;s.pub.publish(o)
require_isolation();rclpy.init();n=Fixture()
try:rclpy.spin(n)
finally:n.destroy_node();rclpy.shutdown()
