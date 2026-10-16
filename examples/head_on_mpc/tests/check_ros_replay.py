#!/usr/bin/env python3
"""Integration check of the installed standalone ROS package (no display needed)."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=Path(__file__).resolve().parents[1]/'results/run_001')
    args = parser.parse_args()
    run = args.run.resolve()
    rclpy.init()
    node = rclpy.create_node('head_on_replay_check')
    latest,frames = [None],[]
    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MarkerArray,'/mpc_replay/markers',lambda m:latest.__setitem__(0,m),qos)
    node.create_subscription(TFMessage,'/tf',lambda m:frames.extend(t.child_frame_id for t in m.transforms),10)
    results = []
    try:
        for case in ('head_on_inline_stop','head_on_pause_bay','head_on_reverse'):
            latest[0] = None
            frames.clear()
            expected = json.loads((run/case/'summary.json').read_text())['status']
            command = ['ros2','launch','head_on_mpc','replay.launch.py',f'case:={case}',
                       f'study_dir:={run}','rviz:=false','loop:=false','playback_rate:=30.0']
            with (run/f'ros_{case}.log').open('w') as stream:
                process = subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    deadline = time.monotonic()+12
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.1)
                        if process.poll() is not None:
                            raise RuntimeError(f'{case}: launch failed')
                        message = latest[0]
                        titles = [] if message is None else [m.text for m in message.markers if m.ns=='status' and m.id==0]
                        if (titles and case in titles[0] and titles[0].endswith(expected)
                                and 'case_view' in frames):
                            break
                    else:
                        raise AssertionError(f'{case}: final status not received')
                    assert 'case_view' in frames
                    assert all(m.header.frame_id=='map' for m in message.markers)
                    with (run/case/'actual.csv').open() as recording:
                        final = {row['car']:row for row in csv.DictReader(recording)}
                    bodies = [m for m in message.markers if m.id==0 and m.ns in final]
                    assert len(bodies)==2
                    for body in bodies:
                        assert abs(body.pose.position.x-float(final[body.ns]['x']))<1e-6
                        assert abs(body.pose.position.y-float(final[body.ns]['y']))<1e-6
                    assert any(m.id==3 and len(m.points)>10 for m in message.markers)
                    assert any(m.id==4 and len(m.points)>2 for m in message.markers)
                    results.append({'case':case,'passed':True,'final_positions_match_log':True,
                                    'camera_tf_received':True,'status':titles[0],
                                    'gui_rendering_tested':False})
                    print(results[-1],flush=True)
                finally:
                    os.killpg(process.pid,signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL)
                        process.wait()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    (run/'rviz_validation.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
