#!/usr/bin/env python3
"""Two independent ROS 2 MPC processes; offline lockstep plant and fault injection."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from car_swarm_agent.msg import PredictedTrajectory, TrajectoryPoint
from decentralized_mpc import DT, EPOCH, N, SUBSTEPS, packet, receive, rollout, run, solve

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
MAX_AGE = DT + .05


def encode(vehicle_id, pred, sequence=0):
    msg = PredictedTrajectory()
    msg.vehicle_id = vehicle_id
    msg.prediction_sequence = sequence
    msg.header.frame_id = 'map'
    ns = round(pred['stamp'] * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
    for t, state in zip(pred['times'], pred['states']):
        point = TrajectoryPoint()
        ns = round((t - pred['stamp']) * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
        point.pose.position.x, point.pose.position.y = map(float, state[:2])
        point.pose.orientation.z = math.sin(state[2] / 2)
        point.pose.orientation.w = math.cos(state[2] / 2)
        point.speed, point.steering = map(float, state[3:])
        point.direction = 1 if state[3] > 1e-8 else 0
        msg.points.append(point)
    return msg


def decode(msg):
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    states, times = [], []
    for p in msg.points:
        q = p.pose.orientation
        states.append([p.pose.position.x, p.pose.position.y,
                       math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z), p.speed, p.steering])
        times.append(stamp + p.time_from_start.sec + p.time_from_start.nanosec * 1e-9)
    return {'stamp': stamp, 'times': np.array(times), 'states': np.array(states)}


def prediction_status(pred, now, expected_stamp):
    if pred is None:
        return 'missing'
    age = now - pred['stamp']
    if age < -1e-8:
        return 'future'
    if age > MAX_AGE:
        return 'expired'
    if abs(pred['stamp'] - expected_stamp) > 1e-8:
        return 'missing_round'
    try:
        receive(pred, now + np.arange(1, N * SUBSTEPS + 1) * DT / SUBSTEPS)
    except (ValueError, IndexError, TypeError):
        return 'invalid_or_uncovered'
    return 'fresh'


def select_previous(inbox, cycle):
    eligible = [key for key in inbox if key < cycle]
    return inbox[max(eligible)] if eligible else (None, None)


def braking(state):
    u = np.zeros((N, 2))
    u[:, 1] = state[4]
    speed = max(0., state[3])
    for j in range(N):
        u[j, 0] = -min(1., speed / DT)
        speed = max(0., speed + u[j, 0] * DT)
    return u


class MPCNode(Node):
    def __init__(self, car, root, directory, fault):
        super().__init__(f'car_{car}_mpc')
        self.car, self.peer = f'car_{car}', f'car_{3-car}'
        self.now = EPOCH
        self.cycle = 0
        self.warm, self.pending = np.zeros((N, 2)), None
        self.fault = fault
        self.inbox = {}
        self.events = (directory / f'{self.car}_exchange.jsonl').open('w')
        self.pub = self.create_publisher(PredictedTrajectory, f'{root}/{self.car}/prediction', QOS)
        self.out = self.create_publisher(String, f'{root}/{self.car}/control', QOS)
        self.create_subscription(PredictedTrajectory, f'{root}/{self.peer}/prediction', self.on_prediction, QOS)
        self.create_subscription(String, f'{root}/{self.car}/request', self.on_request, QOS)
        self.create_timer(.01, self.process)

    def log(self, event):
        self.events.write(json.dumps(event, allow_nan=False) + '\n')
        self.events.flush()

    def on_prediction(self, msg):
        at = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        valid = msg.vehicle_id == self.peer and msg.header.frame_id == 'map' and len(msg.points) >= 2
        self.log({'event': 'receive', 'vehicle_id': msg.vehicle_id,
                  'stamp_absolute_s': stamp, 'prediction_sequence': msg.prediction_sequence, 'receiver_absolute_s': self.now,
                  'message_age_s': self.now - stamp, 'point_count': len(msg.points),
                  'accepted_identity_frame_count': valid, 'receive_monotonic_s': at})
        if valid:
            pred = decode(msg)
            pred['sequence'] = msg.prediction_sequence
            if np.all(np.isfinite(pred['states'])) and np.all(np.diff(pred['times']) > 0):
                self.inbox[msg.prediction_sequence] = (pred, at)
                for key in list(self.inbox):
                    if key < self.cycle - 3:
                        del self.inbox[key]

    def send(self, data):
        self.out.publish(String(data=json.dumps(data)))

    def publish(self, pred, cycle):
        if self.car == 'car_2' and ((cycle == 2 and self.fault == 'drop') or (cycle == -1 and self.fault == 'missing')):
            self.log({'event': 'injected_drop', 'cycle': cycle, 'vehicle_id': self.car})
            return
        if self.car == 'car_2' and cycle == 2 and self.fault == 'stale':
            pred = dict(pred, stamp=pred['stamp'] - 1., times=pred['times'] - 1.)
        self.pub.publish(encode(self.car, pred, cycle))
        self.log({'event': 'publish', 'cycle': cycle, 'vehicle_id': self.car,
                  'stamp_absolute_s': pred['stamp'], 'point_count': len(pred['states'])})

    def on_request(self, msg):
        data = json.loads(msg.data)
        if data['kind'] == 'init':
            names = ('t', 'x', 'y', 'yaw', 'speed')
            rows = data['reference']
            self.ref = np.zeros(len(rows), dtype=[(k, float) for k in names])
            for j, k in enumerate(names):
                self.ref[k] = np.array(rows)[:, j]
            self.obstacles = np.array(data['obstacles'])
            state = np.array(data['state'])
            self.publish(packet(EPOCH, rollout(state, self.warm, tail=True)), -1)
            self.send({'kind': 'ready'})
        elif data['kind'] == 'step':
            self.now = data['now']
            self.cycle = data['cycle']
            self.pending = (data, time.perf_counter())

    def process(self):
        if self.pending is None:
            return
        data, request_at = self.pending
        cycle, now = data['cycle'], data['now']
        expected = EPOCH if cycle == 0 else now - DT
        pred, received_at = select_previous(self.inbox, cycle)
        status = prediction_status(pred, now, expected)
        if status == 'fresh' and cycle - 1 not in self.inbox:
            status = 'missing_round'
        if status != 'fresh' and time.perf_counter() - request_at < .15:
            return
        self.pending = None
        state = np.array(data['state'])
        if status == 'fresh':
            u, report = solve(state, now, pred, self.ref, self.obstacles, self.warm)
        else:
            u = braking(state)
            report = {'accepted': False, 'solver_success': False, 'solve_seconds': 0.,
                      'prediction_constraint_violation': 0., 'attempts': []}
        conservative = status != 'fresh' or not report['accepted']
        if conservative:
            u = braking(state)
        # accepted means a finite bounded command is output; solver acceptance
        # remains separately recorded. Braking is not certified collision-free.
        report['optimizer_accepted'] = report['accepted']
        report['accepted'] = True
        output_at = time.perf_counter()
        report.update({'conservative': conservative, 'peer_status': status, 'peer_expected_sequence': cycle - 1,
                       'peer_sequence': pred.get('sequence') if pred else None,
                       'fallback_reason': status if status != 'fresh' else ('solver_rejected' if conservative else None),
                       'peer_vehicle_id': self.peer,
                       'peer_stamp_absolute_s': pred['stamp'] if pred else None,
                       'message_age_s': now - pred['stamp'] if pred else None,
                       'point_count': len(pred['states']) if pred else 0,
                       'request_to_control_seconds': output_at - request_at,
                       'receive_to_control_seconds': output_at - received_at if received_at else None,
                       'receive_available': received_at is not None,
                       'control_output_monotonic_s': output_at,
                       'worker_pid': os.getpid()})
        self.log({'event': 'control', 'cycle': cycle, 'absolute_time_s': now, **report})
        self.send({'kind': 'control', 'cycle': cycle, 'controls': u.tolist(), 'report': report})
        self.publish(packet(now, rollout(state, u, tail=True)), cycle)
        self.warm = np.vstack((u[1:], u[-1]))


class ROSTransport:
    def __init__(self, directory, fault='none'):
        rclpy.init()
        self.node = Node('offline_two_car_plant')
        root = f'/mpc_benchmark_{os.getpid()}'
        self.responses, self.processes, self.logs = {}, [], []
        self.requests = [self.node.create_publisher(String, f'{root}/car_{i}/request', QOS) for i in (1, 2)]
        for i in (1, 2):
            self.node.create_subscription(String, f'{root}/car_{i}/control',
                lambda msg, i=i: self.responses.__setitem__(i, json.loads(msg.data)), QOS)
            log = (directory / f'car_{i}_process.log').open('w')
            self.logs.append(log)
            self.processes.append(subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
                '--worker', str(i), '--root', root, '--output', str(directory.resolve()), '--fault', fault],
                stdout=log, stderr=subprocess.STDOUT))
        try:
            self.wait(lambda: all(p.get_subscription_count() for p in self.requests), 20)
        except Exception:
            self.close()
            raise

    def wait(self, condition, timeout=30):
        end = time.perf_counter() + timeout
        while not condition():
            if any(p.poll() is not None for p in self.processes):
                raise RuntimeError('MPC worker exited; inspect car_*_process.log')
            if time.perf_counter() > end:
                raise TimeoutError('ROS handshake/control timed out')
            rclpy.spin_once(self.node, timeout_sec=.01)

    def initialize(self, refs, obstacles, states, predictions):
        for i in range(2):
            rows = np.column_stack([refs[i][k] for k in ('t', 'x', 'y', 'yaw', 'speed')])
            self.requests[i].publish(String(data=json.dumps({'kind': 'init',
                'reference': rows.tolist(), 'obstacles': obstacles.tolist(), 'state': states[i].tolist()})))
        self.wait(lambda: all(self.responses.get(i, {}).get('kind') == 'ready' for i in (1, 2)))

    def step(self, cycle, now, states):
        self.responses.clear()
        start = time.perf_counter()
        for i in range(2):
            self.requests[i].publish(String(data=json.dumps({'kind': 'step', 'cycle': cycle,
                                                            'now': now, 'state': states[i].tolist()})))
        self.wait(lambda: all(self.responses.get(i, {}).get('cycle') == cycle for i in (1, 2)))
        elapsed = time.perf_counter() - start
        return [(np.array(self.responses[i]['controls']),
                 dict(self.responses[i]['report'], round_request_to_outputs_seconds=elapsed)) for i in (1, 2)]

    def close(self):
        for p in self.processes:
            if p.poll() is None:
                p.terminate()
        for p in self.processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill(); p.wait()
        for log in self.logs:
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', type=int, choices=(1, 2))
    parser.add_argument('--root')
    parser.add_argument('--output', type=Path, default=Path('log/ros_decentralized_mpc'))
    parser.add_argument('--binary', type=Path, default=Path('build/car_swarm_agent/compare_spatial_temporal'))
    parser.add_argument('--max-seconds', type=float, default=60.)
    parser.add_argument('--fault', choices=('none', 'missing', 'drop', 'stale'), default='none')
    args = parser.parse_args()
    if not math.isfinite(args.max_seconds) or args.max_seconds <= 0:
        parser.error('--max-seconds must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=True)
    if args.worker:
        rclpy.init()
        node = MPCNode(args.worker, args.root, args.output, args.fault)
        try:
            rclpy.spin(node)
        finally:
            node.events.close()
            node.destroy_node()
            rclpy.shutdown()
    else:
        transport = ROSTransport(args.output, args.fault)
        try:
            result = run(args.output, args.binary, args.max_seconds, transport=transport)
        finally:
            transport.close()
        raise SystemExit(0 if result['success'] else 1)
