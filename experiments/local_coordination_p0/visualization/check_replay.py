#!/usr/bin/env python3
"""Check actual-log decoding and the live four-car ROS MarkerArray."""
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import DurabilityPolicy,QoSProfile
from visualization_msgs.msg import Marker,MarkerArray
from replay_log import Recording,DEFAULT_RUN,LEGACY_RUN,CARS

r=Recording(DEFAULT_RUN);assert r.full_task and len(r.frames)==1754 and abs(r.duration-350.6)<1e-9
legacy=Recording(LEGACY_RUN);assert legacy.full_task and len(legacy.frames)==1199
assert r.frames[406]['owner']=='B1' and r.frames[36]['states']['B1'][3]==0 and r.frames[412]['states']['B1'][3]>0
for car,z in r.frames[-1]['states'].items():
    assert abs(z[0]-r.config['goals'][car][0])<.08 and z[3]==z[4]==0
    cx,cy=r.body_center(z);assert abs(cx-z[0]-1.25*math.cos(z[2]))<1e-12
assert all(r.frames[1]['states'][c]!=r.frames[0]['states'][c] for c in CARS)
assert abs(r.frames[1]['states']['B1'][2]-math.pi)<1e-12
b=Recording(Path(__file__).resolve().parents[1]/'stage_b/results/comparison_002/curve_B-ZG')
assert not b.full_task and len(b.frames)==241 and b.duration==48
rclpy.init();node=rclpy.create_node('check_dmpc_actual_replay');received=[]
node.create_subscription(MarkerArray,'/dmpc_replay/markers',lambda msg:received.append(msg),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
started=time.monotonic()
try:
    while (len(received)<6 or len({tuple((m.pose.position.x,m.pose.position.y) for m in msg.markers if m.ns in CARS and m.id==0) for msg in received})<2) and time.monotonic()-started<15:rclpy.spin_once(node,timeout_sec=.2)
    assert len(received)>=2,'No live replay messages; start start_replay.sh in the same ROS_DOMAIN_ID'
    positions=[]
    for msg in received:
        markers={(m.ns,m.id):m for m in msg.markers}
        for car in CARS:
            body=markers[(car,0)];arrow=markers[(car,20)]
            assert body.type==Marker.CUBE and body.header.frame_id=='map'
            assert body.scale.x==4.5 and body.scale.y==1.8
            yaw=2*math.atan2(body.pose.orientation.z,body.pose.orientation.w);rear=arrow.points[0]
            assert abs(body.pose.position.x-rear.x-1.25*math.cos(yaw))<1e-9
            assert abs(body.pose.position.y-rear.y-1.25*math.sin(yaw))<1e-9
        positions.append(tuple((markers[(c,0)].pose.position.x,markers[(c,0)].pose.position.y) for c in CARS))
    assert len(set(positions))>1,'Received replay did not move'
    topics=node.get_publisher_names_and_types_by_node('dmpc_actual_replay','/')
    assert {name for name,types in topics}<={'/dmpc_replay/markers','/tf','/rosout','/parameter_events'}
    result=dict(passed=True,actual_full_task_s=r.duration,full_trace_frames=len(r.frames),partial_interface_s=b.duration,live_messages=len(received),all_four_bodies_correct=True,B_yaw_and_rear_axle_offset_checked=True,live_motion_observed=True,published_topics=[name for name,types in topics],no_control_topics=True)
    path=Path(__file__).with_name('verification.json');path.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
finally:
    node.destroy_node();rclpy.shutdown()
