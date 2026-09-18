#!/usr/bin/env python3
"""Run actual fake-heading -> gimbal -> chassis -> mux chain in navigation mode.
Ground-truth TF is a test fixture only. No navigation source code is altered.
"""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

class Trial(Node):
 def __init__(self):
  super().__init__('navigation_pid_trial');self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
  self.state={};self.rows=[];self.origin=None;self.fake=math.pi/4;self.first=None;self.last_send=-1;self.ref=None;self.prior=None
  self.tf=TransformBroadcaster(self)
  self.cmd=self.create_publisher(Twist,'/cmd_vel',10);self.mode=self.create_publisher(String,'/control_mode',10)
  self.subs=[self.create_subscription(Odometry,'/wheel/odometry',self.odom,10),
   self.create_subscription(JointState,'/joint_states',self.joints,10),
   self.create_subscription(TFMessage,'/tf',self.tf_msg,100)]
  for topic in ['/cmd_vel_chassis','/navigation/cmd_vel','/navigation/cmd_vel_base_link']:
   self.subs.append(self.create_subscription(Twist,topic,lambda m,t=topic:self.state.__setitem__(t,[m.linear.x,m.linear.y,m.angular.z]),10))
  self.subs.append(self.create_subscription(Float64,'/model/sentry/joint/gimbal_yaw_joint/cmd_vel',lambda m:self.state.__setitem__('gimbal_cmd',m.data),10))
  self.subs.append(self.create_subscription(Odometry,'/Odometry/EC',lambda m:self.state.__setitem__('ec',[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]),10))
  for side in ['front_left','front_right','rear_left','rear_right']:
   for kind,topic in [('wheel','target_speed'),('steer','target_angle')]:
    name=side+'_'+kind+'_joint'
    self.subs.append(self.create_subscription(Float64,'/swerve/'+name+'/'+topic,lambda m,k=name:self.state.__setitem__(k+'_target',m.data),10))
 def joints(self,m):self.state['joints']={k:[m.position[i],m.velocity[i]] for i,k in enumerate(m.name) if i<len(m.position) and i<len(m.velocity)}
 def tf_msg(self,m):
  for t in m.transforms:
   if t.child_frame_id=='base_link_fake':self.fake=yaw(t.transform.rotation)
 def odom(self,m):
  if 'joints' not in self.state:return
  js=self.state['joints'];joint=js.get('gimbal_yaw_joint')
  if joint is None:return
  now=stamp(m)
  if self.first is None:self.first=now;self.origin=[m.pose.pose.position.x,m.pose.pose.position.y];self.ref=self.origin.copy()
  elapsed=now-self.first;body=yaw(m.pose.pose.orientation);cloud=body+1.6032+joint[0]
  transforms=[]
  for parent,child,angle in [('odom','base_link',cloud),('base_link','base_link_static',-cloud)]:
   tf=TransformStamped();tf.header=m.header;tf.header.frame_id=parent;tf.child_frame_id=child
   tf.transform.rotation.z=math.sin(angle/2);tf.transform.rotation.w=math.cos(angle/2)
   if parent=='odom':tf.transform.translation.x=m.pose.pose.position.x;tf.transform.translation.y=m.pose.pose.position.y;tf.transform.translation.z=.3
   transforms.append(tf)
  self.tf.sendTransform(transforms)
  if now-self.last_send<.019:return
  self.last_send=now
  heading=.8*math.sin(max(0,elapsed-12)*.45) if elapsed<28 else .8*math.sin(16*.45)
  if elapsed<5:speed=0
  elif elapsed<8:speed=2*(elapsed-5)/3
  elif elapsed<12:speed=2
  elif elapsed<18:speed=2.5
  elif elapsed<26:speed=3
  elif elapsed<28:speed=3*(28-elapsed)/2
  else:speed=0
  command=Twist();command.linear.x=float(speed);command.angular.z=max(-1.5,min(1.5,3*wrap(heading-self.fake))) if elapsed<28 else 0.
  self.mode.publish(String(data='navigation'));self.cmd.publish(command)
  vx=math.cos(body)*m.twist.twist.linear.x-math.sin(body)*m.twist.twist.linear.y
  vy=math.sin(body)*m.twist.twist.linear.x+math.cos(body)*m.twist.twist.linear.y
  desired=[speed*math.cos(self.fake),speed*math.sin(self.fake)]
  if self.prior is not None:
   dt=now-self.prior
   self.ref=[self.ref[i]+desired[i]*dt for i in range(2)]
  self.prior=now
  self.rows.append({'t':elapsed,'sim_time':now,'body_yaw':body,'cloud_yaw':cloud,'pose':[m.pose.pose.position.x,m.pose.pose.position.y],
   'actual':[vx,vy,m.twist.twist.angular.z],'desired':desired,'ref':self.ref.copy(),'fake_yaw':self.fake,'speed':speed,**self.state})

parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
rclpy.init();node=Trial();wall=time.monotonic()
try:
 while (not node.rows or node.rows[-1]['t']<38) and time.monotonic()-wall<120:
  rclpy.spin_once(node,timeout_sec=.05)
 node.cmd.publish(Twist())
finally:
 Path(args.output).write_text(json.dumps(node.rows));node.destroy_node();rclpy.shutdown()
rows=node.rows
if not rows:raise RuntimeError('No simulation feedback')
summary={'samples':len(rows),'sim_duration':rows[-1]['t']}
for label,lo,hi in [('straight_2',9,12),('curve_2_5',14,18),('curve_3',20,26),('park',33,38)]:
 r=[x for x in rows if lo<x['t']<hi]
 if not r:continue
 summary[label]={'speed_mean':sum(math.hypot(*x['actual'][:2]) for x in r)/len(r),
  'vector_rmse':math.sqrt(sum(sum((x['actual'][i]-x['desired'][i])**2 for i in range(2)) for x in r)/len(r)),
  'max_body_yaw_rate':max(abs(x['actual'][2]) for x in r),
  'max_wheel_speed_error':max(abs(v[1]-x.get(k+'_target',v[1])) for x in r for k,v in x['joints'].items() if 'wheel' in k),
  'max_steer_angle_error':max(abs(wrap(v[0]-x.get(k+'_target',v[0]))) for x in r for k,v in x['joints'].items() if 'steer' in k),
  'world_gimbal_mean_dps':sum(math.degrees(x['actual'][2]+x['joints']['gimbal_yaw_joint'][1]) for x in r)/len(r),
  'world_gimbal_max_error_dps':max(abs(math.degrees(x['actual'][2]+x['joints']['gimbal_yaw_joint'][1])-100) for x in r)}
summary['final_path_error']=math.dist(rows[-1]['pose'],rows[-1]['ref'])
Path(args.output+'.summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
