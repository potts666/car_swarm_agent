#!/usr/bin/env python3
"""RViz2 markers from accepted actual logs. Publishes no vehicle controls."""
import argparse
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from geometry_msgs.msg import Point,TransformStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy,QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker,MarkerArray
from replay_log import Recording,CARS,DEFAULT_RUN

COLORS=((.2,.65,1.),(1.,.55,.1),(.3,.85,.35),(.85,.35,.9))


class Replay(Node):
    def __init__(self,args):
        super().__init__('dmpc_actual_replay');self.recording=Recording(args.run);self.rate=args.rate;self.once=args.once;self.started=time.monotonic()
        self.publisher=self.create_publisher(MarkerArray,'/dmpc_replay/markers',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf=TransformBroadcaster(self);self.create_timer(.05,self.tick);self.tick()
        self.get_logger().info(f'Actual log replay: {self.recording.directory}; {self.recording.duration:.1f}s; {self.rate:g}x; full task={self.recording.full_task}; no control output')

    def marker(self,ns,index,kind,color,x=0.,y=0.,z=0.):
        m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.ns=ns;m.id=index;m.type=kind;m.pose.orientation.w=1.
        m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.position.z=float(z)
        m.color.r,m.color.g,m.color.b=map(float,color[:3]);m.color.a=float(color[3] if len(color)>3 else 1.)
        return m

    def line(self,ns,index,color,points,width=.1,z=.05):
        m=self.marker(ns,index,Marker.LINE_STRIP,color);m.scale.x=float(width);m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y in points]
        if len(points)<2:m.action=Marker.DELETE
        return m

    def text(self,index,text,x,y,height=1.4):
        m=self.marker('status',index,Marker.TEXT_VIEW_FACING,(.95,.95,.95),x,y,1.);m.scale.z=float(height);m.text=text;return m

    def tick(self):
        r=self.recording;elapsed=(time.monotonic()-self.started)*self.rate;local=elapsed if self.once else elapsed%(r.duration+3.)
        seconds=min(local,r.duration);index,frame=r.sample(seconds);markers=[]
        for i,(x0,x1,y0,y1) in enumerate(r.config['road_rectangles']):
            m=self.marker('road',i,Marker.CUBE,(.23,.26,.29), (x0+x1)/2,(y0+y1)/2,-.1)
            m.scale.x=float(x1-x0);m.scale.y=float(y1-y0);m.scale.z=.1;markers.append(m)
        for i,x in enumerate(r.config.get('resource_boundaries',(-26.5,-18,18,26.5))):
            markers.append(self.line('resource',i,(.7,.6,.2,.6),[(x,-5.5),(x,5.5)],.06))
        for i,car in enumerate(CARS):
            state=frame['states'][car];x,y,yaw,v,delta=state;color=COLORS[i];cx,cy=r.body_center(state)
            body=self.marker(car,0,Marker.CUBE,(*color,1. if car in frame.get('active',[frame['actor']]) else .65),cx,cy,.3)
            body.scale.x=float(r.config['vehicle']['length']);body.scale.y=float(r.config['vehicle']['width']);body.scale.z=.5
            body.pose.orientation.z=math.sin(yaw/2);body.pose.orientation.w=math.cos(yaw/2);markers.append(body)
            markers.append(self.line(car,1,(*color,.9),r.trail(car,index),.1))
            markers.append(self.line(car,2,(*color,.25),r.paths[car],.045,.025))
            for axle,distance,angle in ((0,0.,yaw),(1,float(r.config['vehicle']['L']),yaw+delta)):
                for side in (-1,1):
                    wx=x+distance*math.cos(yaw)-side*.72*math.sin(yaw);wy=y+distance*math.sin(yaw)+side*.72*math.cos(yaw)
                    points=[(wx+s*.4*math.cos(angle),wy+s*.4*math.sin(angle)) for s in (-1,1)]
                    markers.append(self.line(car,10+axle*2+(side+1)//2,(.05,.05,.05),points,.15,.6))
            arrow=self.marker(car,20,Marker.ARROW,(1.,1.,1.),x,y,.64);arrow.points=[Point(x=float(x),y=float(y),z=.64),Point(x=x+1.8*math.cos(yaw),y=y+1.8*math.sin(yaw),z=.64)];arrow.pose.position.x=arrow.pose.position.y=arrow.pose.position.z=0.;arrow.scale.x=.12;arrow.scale.y=.3;arrow.scale.z=.3;markers.append(arrow)
            label=self.marker(car,30,Marker.TEXT_VIEW_FACING,color,cx,cy+1.8,1.);label.scale.z=1.2;label.text=f'{car}  v={v:.2f}  delta={delta:+.3f}';markers.append(label)
            goal=r.config['goals'][car];gx,gy=goal[:2];markers.append(self.line('goals',i,(*color,.7),[(gx-.5,gy),(gx+.5,gy),(gx,gy),(gx,gy-.5),(gx,gy+.5)],.08))
            transform=TransformStamped();transform.header.frame_id='map';transform.header.stamp=self.get_clock().now().to_msg();transform.child_frame_id='dmpc_'+car
            transform.transform.translation.x=float(x);transform.transform.translation.y=float(y);transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2);self.tf.sendTransform(transform)
        status='FULL FOUR-CAR TASK' if r.full_task else 'PARTIAL INTERFACE RUN'
        markers.append(self.text(0,f'{r.method} | t={index*r.dt:.1f}/{r.duration:.1f}s | {self.rate:g}x | {status}',0,10.))
        detail=f"active={frame.get('active',frame['actor'])} | owner={frame.get('owner','-')} | budget={frame['budget']} ticks"
        markers.append(self.text(1,detail,0,8.7,1.1))
        message=r.stage(index) if hasattr(r,'stage') else 'actual accepted execution'
        markers.append(self.text(2,message+' | thin: accepted full path; thick: executed',0,-9.,.95))
        if local>=r.duration:markers.append(self.text(3,'END: all goals reached' if r.full_task else 'END: interface verification only',0,-10.3,1.1))
        else:
            m=self.text(3,'',0,0);m.action=Marker.DELETE;markers.append(m)
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,default=DEFAULT_RUN);p.add_argument('--rate',type=float,default=4.);p.add_argument('--once',action='store_true');p.add_argument('--no-rviz',action='store_true');args=p.parse_args()
    if not math.isfinite(args.rate) or args.rate<=0:p.error('--rate must be positive and finite')
    rclpy.init();node=None;rviz=None
    try:
        node=Replay(args)
        if not args.no_rviz:
            rviz=subprocess.Popen(['rviz2','-d',str(Path(__file__).with_name('fourcar.rviz'))])
            def window_closed():
                if rviz.poll() is not None and rclpy.ok():rclpy.shutdown()
            node.create_timer(.25,window_closed)
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if rviz is not None and rviz.poll() is None:
            rviz.terminate()
            try:rviz.wait(timeout=5)
            except subprocess.TimeoutExpired:rviz.kill();rviz.wait()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
