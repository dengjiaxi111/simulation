"""Isolated fixed-path plant test. Navigation mode, real physics feedback, no TF fixture.
Input is already chassis-frame velocity at the existing navigation/mux boundary.
This stage does NOT test Nav2 or fake->gimbal conversion. All changes are transient.
"""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Float64,Float64MultiArray
from routes import route,cumulative,reference,distances
from gains import apply,source_gains,require_isolation,reset_pose

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
class Trial(Node):
    def __init__(self,args):
        super().__init__('isolated_pid_trial');self.set_parameters([Parameter('use_sim_time',value=True)])
        self.args=args;self.rows=[];self.js={};self.targets={};self.first=None;self.origin=None;self.done=False;self.actual_cmd=None;self.motor=None
        self.path=route(args.shape,args.size,width=getattr(args,'width',None));self.s=cumulative(self.path);self.last=-1
        self.pub=self.create_publisher(Twist,'/navigation/cmd_vel',10);self.mode=self.create_publisher(String,'/control_mode',10)
        self.create_subscription(Odometry,'/wheel/odometry',self.odom,qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray,'/pid_tuning/motor_telemetry',lambda m:setattr(self,'motor',list(m.data)),qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel_chassis',lambda m:setattr(self,'actual_cmd',[m.linear.x,m.linear.y,m.angular.z]),10)
        for side in ['front_left','front_right','rear_left','rear_right']:
            for kind,term in [('wheel','target_speed'),('steer','target_angle')]:
                k=side+'_'+kind+'_joint';self.create_subscription(Float64,'/swerve/'+k+'/'+term,lambda m,k=k:self.targets.__setitem__(k,m.data),10)
    def joints(self,m):
        self.js={k:[float(m.position[i]),float(m.velocity[i])] for i,k in enumerate(m.name) if i<len(m.position) and i<len(m.velocity)}
    def send(self,vx=0.,vy=0.):
        self.mode.publish(String(data='navigation'));m=Twist();m.linear.x=float(vx);m.linear.y=float(vy);self.pub.publish(m)
    def odom(self,m):
        if self.done or not self.js:return
        now=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.first is None:self.first=now;self.origin=np.array([m.pose.pose.position.x,m.pose.pose.position.y]);self.path+=self.origin
        if now-self.last<.019:return
        self.last=now;t=now-self.first;pos=np.array([m.pose.pose.position.x,m.pose.pose.position.y]);angle=yaw(m.pose.pose.orientation)
        # Acceleration and braking are part of the scored moving trajectory.
        tau=max(0,t-2);v=self.args.speed;acc=self.args.acceleration;ramp=v/acc;length=self.s[-1]
        cruise=max(0,(length-v*ramp)/v);duration=2*ramp+cruise
        if tau<ramp: speed=acc*tau;d=.5*acc*tau*tau
        elif tau<ramp+cruise:speed=v;d=.5*v*ramp+v*(tau-ramp)
        elif tau<duration:
            z=tau-ramp-cruise;speed=max(0,v-acc*z);d=.5*v*ramp+v*cruise+v*z-.5*acc*z*z
        else:speed=0.;d=length
        ref,tan=reference(self.path,self.s,d);desired=speed*tan
        # Keep the existing mux sign convention. Positive kinematic wheel speed
        # produces negative physical velocity with the authored wheel axes.
        body=np.array([[math.cos(angle),math.sin(angle)],[-math.sin(angle),math.cos(angle)]])@desired
        self.send(body[0],body[1])
        # wheel/odometry twist is documented by the installed OdometryPublisher as child-frame.
        bv=np.array([m.twist.twist.linear.x,m.twist.twist.linear.y]);world=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])@bv
        self.rows.append(dict(t=t,sim_time=now,pose=pos.tolist(),yaw=angle,actual=world.tolist(),desired=desired.tolist(),speed=speed,reference=ref.tolist(),angular_velocity=m.twist.twist.angular.z,joints=self.js.copy(),targets=self.targets.copy(),cmd_chassis=self.actual_cmd,motor_telemetry=self.motor))
        if t>duration+5:self.done=True;self.send()
    def finish(self):
        a=self.args
        if not self.rows:raise RuntimeError('No odometry + joint feedback')
        moving=[r for r in self.rows if r['t']>=2 and r['speed']>.05]
        if not moving:raise RuntimeError('No moving samples')
        err=distances(np.array([r['pose'] for r in moving]),self.path)
        timed=np.array([math.dist(r['pose'],r['reference']) for r in moving])
        vector=np.array([math.dist(r['actual'],r['desired']) for r in moving])
        steer=[];wheel=[];gimbal=[]
        for r in moving:
            for k,v in r['joints'].items():
                if k in r['targets']:
                    (steer if 'steer' in k else wheel).append(abs(wrap(r['targets'][k]-v[0])) if 'steer' in k else abs(r['targets'][k]-v[1]))
            if 'gimbal_yaw_joint' in r['joints']:gimbal.append(math.degrees(r['angular_velocity']+r['joints']['gimbal_yaw_joint'][1]))
        summary=dict(stage='fixed_path_chassis_boundary',shape=a.shape,size=a.size,width=a.width,speed=a.speed,gains=a.gains,completed=self.done,samples=len(self.rows),path_max=float(max(err)),path_p95=float(np.quantile(err,.95)),path_rmse=float(np.sqrt(np.mean(err**2))),time_position_max=float(max(timed)),velocity_vector_rmse=float(np.sqrt(np.mean(vector**2))),final_error=math.dist(self.rows[-1]['pose'],self.path[-1]),steer_error_max=max(steer,default=None),wheel_error_max=max(wheel,default=None),gimbal_world_dps_mean=float(np.mean(gimbal)) if gimbal else None,torque_command_observation_available=any(r.get('motor_telemetry') for r in moving),gain_ack_available=False)
        telemetry=[r['motor_telemetry'] for r in moving if r.get('motor_telemetry') and len(r['motor_telemetry'])==25]
        summary['torque_command_saturation_fraction']={str(i):sum(abs(m[3+3*i])>=(4.95 if i%2==0 else 1.98) for m in telemetry)/len(telemetry) for i in range(8)} if telemetry else None
        summary['passed']=bool(self.done and summary['path_max']<=.1 and summary['final_error']<=.1 and summary['velocity_vector_rmse']<=.15)
        Path(a.output).parent.mkdir(parents=True,exist_ok=True)
        Path(a.output).write_text(json.dumps(dict(summary=summary,path=self.path.tolist(),rows=self.rows)))
        print(json.dumps(summary,indent=2),flush=True)
def main():
    p=argparse.ArgumentParser();p.add_argument('--shape',choices=['straight','S','J','U','C','circle','ellipse'],default='straight');p.add_argument('--size',type=float,default=8);p.add_argument('--width',type=float);p.add_argument('--speed',type=float,default=2);p.add_argument('--acceleration',type=float,default=1);p.add_argument('--gains',type=float,nargs=6);p.add_argument('--output',required=True);p.add_argument('--wall-timeout',type=float,default=120);a=p.parse_args()
    if a.speed<=0 or a.acceleration<=0 or a.size<=0:raise ValueError('positive speed, acceleration, size required')
    if cumulative(route(a.shape,a.size,width=a.width))[-1]<a.speed*a.speed/a.acceleration:raise ValueError('route too short to reach requested speed with acceleration/braking')
    require_isolation();reset_pose();old=source_gains();a.gains=a.gains or old
    rclpy.init();n=Trial(a);start=time.monotonic()
    try:
        for _ in range(20):rclpy.spin_once(n,timeout_sec=.05)
        apply(a.gains)
        while not n.done and time.monotonic()-start<a.wall_timeout:rclpy.spin_once(n,timeout_sec=.05)
    finally:
        for _ in range(10):n.send();rclpy.spin_once(n,timeout_sec=.02)
        apply(old);n.finish();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
