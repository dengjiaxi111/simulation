"""Record step, ramp, braking, lateral direction reversal without changing controller."""
import argparse,time,json,math
from pathlib import Path
import rclpy
from trial import Trial,yaw
from gains import apply,source_gains,require_isolation,reset_pose
class Basic(Trial):
    def odom(self,m):
        if not self.js or self.done:return
        now=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.first is None:self.first=now
        if now-self.last<.019:return
        self.last=now;t=now-self.first
        if t<2:label='settle';vx=vy=0.
        elif t<5:label='forward_step';vx=self.args.speed;vy=0.
        elif t<7:label='brake';vx=vy=0.
        elif t<10:label='forward_ramp';vx=self.args.speed*(t-7)/3;vy=0.
        elif t<13:label='left_step';vx=0.;vy=self.args.speed
        elif t<16:label='right_reverse';vx=0.;vy=-self.args.speed
        else:label='stop';vx=vy=0.
        self.send(vx,vy);self.rows.append(dict(t=t,phase=label,pose=[m.pose.pose.position.x,m.pose.pose.position.y],yaw=yaw(m.pose.pose.orientation),velocity=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z],desired_body=[vx,vy],joints=self.js.copy(),targets=self.targets.copy(),cmd_chassis=self.actual_cmd))
        if t>=20:self.done=True
    def finish(self):
        if not self.rows:raise RuntimeError('No joint/odometry feedback')
        Path(self.args.output).write_text(json.dumps(dict(gains=self.args.gains,completed=self.done,rows=self.rows,torque_available=False,gain_ack_available=False)))
        print('Basic response recorded',len(self.rows),'completed',self.done,flush=True)
def main():
    p=argparse.ArgumentParser();p.add_argument('--speed',type=float,default=2);p.add_argument('--gains',type=float,nargs=6);p.add_argument('--output',required=True);a=p.parse_args();require_isolation();reset_pose();a.shape='straight';a.size=20;old=source_gains();a.gains=a.gains or old
    rclpy.init();n=Basic(a);t=time.monotonic()
    try:
        apply(a.gains)
        while not n.done and time.monotonic()-t<120:rclpy.spin_once(n,timeout_sec=.05)
    finally:
        for _ in range(10):n.send();rclpy.spin_once(n,timeout_sec=.02)
        apply(old);n.finish();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
