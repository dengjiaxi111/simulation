"""Score actual Nav2 initial plans against Gazebo truth; optional full LIO chain."""
import argparse,json,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy, ReliabilityPolicy
LATEST=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
from nav2_msgs.action import ComputePathToPose,FollowPath,NavigateToPose
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Path as ROSPath,Odometry,OccupancyGrid
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64MultiArray,Float64
from tf2_ros import Buffer,TransformListener,TransformException
from routes import route,distances
from gains import require_isolation,apply,source_gains
from trial import yaw
class Trial(Node):
 def __init__(s):
  super().__init__('isolated_nav2_trial');s.set_parameters([Parameter('use_sim_time',value=True)]);s.last_record=-1.;s.targets={};s.telemetry=None;s.chain={};s.live=False;s.map_rotation=np.eye(2);s.map_offset=np.zeros(2);s.o=None;s.path=None;s.original=None;s.rows=[];s.js={};s.cmd=None;s.map=None;s.replans=0;s.tf=Buffer();s.listener=TransformListener(s.tf,s)
  s.create_subscription(Odometry,'/wheel/odometry',s.odom,LATEST);s.create_subscription(ROSPath,'/plan',s.plan,10);s.create_subscription(JointState,'/joint_states',s.joints,LATEST);s.create_subscription(Twist,'/cmd_vel_chassis',lambda m:setattr(s,'cmd',[m.linear.x,m.linear.y,m.angular.z]),10)
  s.create_subscription(OccupancyGrid,'/map',lambda m:setattr(s,'map',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));s.mode=s.create_publisher(String,'/control_mode',10);s.reference=s.create_publisher(ROSPath,'/pid_tuning/reference_path',10)
  s.create_subscription(Float64MultiArray,'/pid_tuning/motor_telemetry',lambda m:setattr(s,'telemetry',list(m.data)),LATEST)
  for topic in ['/cmd_vel','/navigation/cmd_vel_base_link','/navigation/cmd_vel']:
   s.create_subscription(Twist,topic,lambda m,k=topic:s.chain.update({k:[m.linear.x,m.linear.y,m.angular.z]}),10)
  for side in ['front_left','front_right','rear_left','rear_right']:
   for kind,term in [('wheel','target_speed'),('steer','target_angle')]:
    k=side+'_'+kind+'_joint';s.create_subscription(Float64,'/swerve/'+k+'/'+term,lambda m,k=k:s.targets.update({k:m.data}),LATEST)
  s.planner=ActionClient(s,ComputePathToPose,'compute_path_to_pose');s.follow=ActionClient(s,FollowPath,'follow_path');s.nav=ActionClient(s,NavigateToPose,'navigate_to_pose')
 def joints(s,m):s.js={k:[m.position[i],m.velocity[i]] for i,k in enumerate(m.name) if i<len(m.position) and i<len(m.velocity)}
 def plan(s,m):
  s.path=m
  if s.original is None and m.poses:s.original=m
  else:s.replans+=1
 def odom(s,m):
  s.o=m
  if s.original is None:return
  p=m.pose.pose.position;t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;estimated=None;age=None
  if t-s.last_record<.02:return
  s.last_record=t
  try:
   tf=s.tf.lookup_transform('map','chassis',rclpy.time.Time());estimated=[tf.transform.translation.x,tf.transform.translation.y];age=t-tf.header.stamp.sec-tf.header.stamp.nanosec*1e-9
  except TransformException:pass
  truth=s.map_rotation@np.array([p.x,p.y])+s.map_offset
  s.rows.append(dict(sim_time=t,pose=truth.tolist(),world_pose=[p.x,p.y],estimated=estimated,tf_age=age,tf_time=t-age if age is not None else None,velocity=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z],joints=s.js.copy(),cmd=s.cmd,command_chain=s.chain.copy(),motor_telemetry=s.telemetry,targets=s.targets.copy()))
 def pose(s,x,y,yaw=0.):
  m=PoseStamped();m.header.frame_id='map';m.header.stamp=s.get_clock().now().to_msg();m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.orientation.z=math.sin(yaw/2);m.pose.orientation.w=math.cos(yaw/2);return m
 def validate(s,path):
  if s.map is None:raise RuntimeError('No map: cannot validate route')
  m=s.map;r=m.info.resolution;a=np.asarray(m.data).reshape(m.info.height,m.info.width);rad=math.ceil(.4/r);yy,xx=np.mgrid[-rad:rad+1,-rad:rad+1];mask=(xx*xx+yy*yy)*r*r<=.4*.4
  for p in path.poses:
   x=int((p.pose.position.x-m.info.origin.position.x)/r);y=int((p.pose.position.y-m.info.origin.position.y)/r)
   if x-rad<0 or y-rad<0 or x+rad>=m.info.width or y+rad>=m.info.height:raise RuntimeError('Route footprint outside map')
   window=a[y-rad:y+rad+1,x-rad:x+rad+1]
   if np.any(window[mask]<0) or np.any(window[mask]>=65):raise RuntimeError('Route conflicts with obstacle/unknown area; not executed')
def wait(n,f,timeout=20):
 t=time.monotonic()
 while not f.done() and time.monotonic()-t<timeout:rclpy.spin_once(n,timeout_sec=.05)
 if not f.done():raise RuntimeError('Action timeout')
 return f.result()
def main():
 p=argparse.ArgumentParser();p.add_argument('--live-localization',action='store_true');p.add_argument('--wall-timeout',type=float,default=60);p.add_argument('--speed',type=float);p.add_argument('--shape',default='straight');p.add_argument('--size',type=float,default=12);p.add_argument('--goal',type=float,nargs=2);p.add_argument('--navigate',action='store_true');p.add_argument('--gains',type=float,nargs=6);p.add_argument('--output',required=True);a=p.parse_args();require_isolation();rclpy.init();n=Trial();n.live=a.live_localization;old=source_gains();handle=None;result=None;start=time.monotonic()
 try:
  while (n.o is None or n.map is None) and time.monotonic()-start<20:rclpy.spin_once(n,timeout_sec=.1)
  if n.o is None or n.map is None:raise RuntimeError('No map/ground truth feedback')
  if a.live_localization:
   tf=None;deadline=time.monotonic()+10
   while tf is None and time.monotonic()<deadline:
    rclpy.spin_once(n,timeout_sec=.1)
    try:tf=n.tf.lookup_transform('map','chassis',rclpy.time.Time())
    except TransformException:pass
   if tf is None:raise RuntimeError('No map->chassis transform')
   angle=yaw(tf.transform.rotation)-yaw(n.o.pose.pose.orientation);c=math.cos(angle);sn=math.sin(angle);n.map_rotation=np.array([[c,-sn],[sn,c]]);n.map_offset=np.array([tf.transform.translation.x,tf.transform.translation.y])-n.map_rotation@np.array([n.o.pose.pose.position.x,n.o.pose.pose.position.y])
  n.mode.publish(String(data='navigation'));apply(a.gains or old)
  if a.goal:
   if not n.planner.wait_for_server(timeout_sec=10):raise RuntimeError('Planner inactive')
   g=ComputePathToPose.Goal();g.goal=n.pose(*a.goal);g.use_start=False;g.planner_id='GridBased';h=wait(n,n.planner.send_goal_async(g))
   if not h.accepted:raise RuntimeError('Planner rejected goal')
   planned=wait(n,h.get_result_async());path=planned.result.path
   if not path.poses:raise RuntimeError('Planner returned empty path')
  else:
   xy=route(a.shape,a.size)+n.map_rotation@np.array([n.o.pose.pose.position.x,n.o.pose.pose.position.y])+n.map_offset;path=ROSPath();path.header.frame_id='map';path.header.stamp=n.get_clock().now().to_msg()
   for i,q in enumerate(xy):
    v=xy[min(i+1,len(xy)-1)]-xy[max(0,i-1)];path.poses.append(n.pose(*q,math.atan2(v[1],v[0])))
  n.original=path;n.rows=[]
  if not a.goal:n.validate(path)
  n.reference.publish(path)
  if a.navigate:
   g=NavigateToPose.Goal();g.pose=path.poses[-1];client=n.nav
  else:
   g=FollowPath.Goal();g.path=path;g.controller_id='PurePursuit';g.goal_checker_id='goal_checker';client=n.follow
  if not client.wait_for_server(timeout_sec=10):raise RuntimeError('Controller inactive')
  handle=wait(n,client.send_goal_async(g))
  if not handle.accepted:raise RuntimeError('Navigation rejected goal')
  future=handle.get_result_async();start=time.monotonic()
  while not future.done() and time.monotonic()-start<a.wall_timeout:rclpy.spin_once(n,timeout_sec=.05)
  if future.done():result=future.result().status
  else:wait(n,handle.cancel_goal_async());result='TIMEOUT'
 finally:
  if handle and result is None:wait(n,handle.cancel_goal_async())
  if n.rows:
   settle=time.monotonic()
   while time.monotonic()-settle<3:rclpy.spin_once(n,timeout_sec=.05)
  apply(old)
  if n.rows and n.original:
   xy=np.array([[p.pose.position.x,p.pose.position.y] for p in n.original.poses]);err=distances(np.array([r['pose'] for r in n.rows]),xy)
   times=np.array([r['sim_time'] for r in n.rows]);poses=np.array([r['pose'] for r in n.rows]);drift=[math.dist([np.interp(r['tf_time'],times,poses[:,i]) for i in range(2)],r['estimated']) for r in n.rows if r['estimated'] is not None and times[0]<=r['tf_time']<=times[-1]];missing=sum(r['estimated'] is None for r in n.rows)
   s=dict(stage='full_navigationros2_with_LIO' if a.live_localization else 'Nav2_with_ground_truth_fixture',action_status=result,path_max=float(max(err)),path_p95=float(np.quantile(err,.95)),path_rmse=float(np.sqrt(np.mean(err**2))),final_error=math.dist(n.rows[-1]['pose'],xy[-1]),localization_max_disagreement=max(drift,default=None),missing_tf_samples=missing,replans=n.replans,tf_age_p95=float(np.quantile([r['tf_age'] for r in n.rows if r['tf_age'] is not None],.95)),actual_speed_max=max(math.hypot(*r['velocity'][:2]) for r in n.rows),gains=a.gains or old)
   s['passed']=bool(result==4 and s['path_max']<=.1 and s['final_error']<=.1 and missing==0 and max(drift,default=1e6)<=.1)
   Path(a.output).write_text(json.dumps(dict(summary=s,path=xy.tolist(),rows=n.rows,world_to_map_rotation=n.map_rotation.tolist(),world_to_map_offset=n.map_offset.tolist())));print(json.dumps(s,indent=2),flush=True)
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
