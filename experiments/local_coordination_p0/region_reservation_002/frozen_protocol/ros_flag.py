"""Three ROS 2 nodes, reliable DDS topics, cycle-synchronous ideal communication."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from flag_constraints import construct, accepts


class Channel:
    def __init__(self):
        self.owner = None
        self.pending = {}
        self.ready = set()
        self.released = set()
        self.first = True

    def request(self, car, cycle):
        if car not in ('A', 'B') or car in self.released:
            raise ValueError('invalid requesting vehicle')
        self.pending.setdefault(car, cycle)

    def release(self, car, checks):
        if car != self.owner or checks.get('passed') is not True or checks.get('both_future_goals') is not True:
            raise ValueError('release requires owner and accepted full future check')
        self.released.add(car)
        self.owner = None

    def grant(self):
        eligible = [c for c in self.pending if self.first or c in self.ready]
        if self.owner is None and eligible:
            self.owner = min(eligible, key=lambda c: (self.pending[c], c))
            del self.pending[self.owner]
            self.first = False
        return self.owner


class RosFlag:
    def __init__(self):
        rclpy.init()
        self.cycle = 0
        self.log = []
        self.constraint_log = []
        self.channel = Channel()
        self.received = {c: None for c in ('A', 'B')}
        self.version = 0
        self.manager = Node('e1_channel')
        self.cars = {c: Node('e1_vehicle_'+c.lower()) for c in ('A', 'B')}
        self.executor = SingleThreadedExecutor()
        for node in [self.manager, *self.cars.values()]:
            self.executor.add_node(node)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.manager.create_publisher(String, '/e1/channel/state', state_qos)
        self.subs = []
        self.publishers = {}
        self.count = 0
        for kind in ('request', 'ready', 'release'):
            self.subs.append(self.manager.create_subscription(String, '/e1/channel/'+kind,
                lambda msg, k=kind: self.on_channel(k, msg), qos))
            for car, node in self.cars.items():
                self.publishers[car, kind] = node.create_publisher(String, '/e1/channel/'+kind, qos)
        for car, node in self.cars.items():
            self.subs.append(node.create_subscription(String, '/e1/channel/state',
                lambda msg, c=car: self.on_state(c, msg), state_qos))
        self.wait(lambda: self.pub_state.get_subscription_count()==2 and
                  all(p.get_subscription_count()==1 for p in self.publishers.values()))
        self.publish_state()

    def record(self, kind, **fields):
        self.log.append(dict(sequence=len(self.log), cycle=self.cycle,
                             wall_monotonic_ns=time.monotonic_ns(), kind=kind, **fields))

    def wait(self, predicate):
        end = time.monotonic()+10
        while not predicate():
            if time.monotonic() > end:
                raise TimeoutError('ROS 2 discovery/delivery timeout')
            self.executor.spin_once(timeout_sec=0.01)

    def send(self, car, kind, payload):
        self.record('publish_'+kind, vehicle_id=car, payload=payload)
        self.publishers[car, kind].publish(String(data=json.dumps(payload)))

    def on_channel(self, kind, msg):
        packet = json.loads(msg.data)
        car = packet['vehicle_id']
        self.record('receive_'+kind, vehicle_id=car, payload=packet)
        if kind == 'request':
            self.channel.request(car, self.cycle)
        elif kind == 'ready':
            if car not in self.channel.pending:
                raise ValueError('ready without pending request')
            self.channel.ready.add(car)
        else:
            self.channel.release(car, packet['checks'])
        self.count += 1

    def on_state(self, car, msg):
        packet = json.loads(msg.data)
        if type(packet['occupied']) is not bool or packet['occupied'] != (packet['owner_id'] in ('A', 'B')):
            raise ValueError('inconsistent channel flag')
        self.received[car] = dict(**packet, receive_cycle=self.cycle)
        self.record('receive_state', vehicle_id=car, **self.received[car])

    def publish_state(self):
        self.channel.grant()
        self.version += 1
        packet = dict(occupied=self.channel.owner is not None, owner_id=self.channel.owner,
                      version=self.version, publish_cycle=self.cycle)
        self.record('publish_state', **packet)
        self.pub_state.publish(String(data=json.dumps(packet)))
        self.wait(lambda: all(v and v['version']==self.version for v in self.received.values()))

    def request(self, requests):
        target = self.count+len(requests)
        for _, car in requests:
            self.send(car, 'request', dict(vehicle_id=car))
        self.wait(lambda: self.count==target)
        self.publish_state()
        owner = self.received['A']['owner_id']
        return owner, {car: self.received[car]['owner_id']==car for _, car in requests}

    def ready(self, car):
        target = self.count+1
        self.send(car, 'ready', dict(vehicle_id=car))
        self.wait(lambda: self.count==target)
        self.publish_state()

    def release(self, car, checks):
        target = self.count+1
        self.send(car, 'release', dict(vehicle_id=car, checks=checks))
        self.wait(lambda: self.count==target)
        self.publish_state()
        return self.received[car]['owner_id']

    def constraints(self, states, queues, forecast, status, scene):
        for car in ('A', 'B'):
            received = self.received[car]
            g = int(received['owner_id']==car)
            params = construct(car, g, states[car], queues[car], status[car], scene)
            accepted = accepts(params, forecast[car], scene)
            self.constraint_log.append(dict(car=car, effective_cycle=self.cycle,
                receive_cycle=received['receive_cycle'], state_version=received['version'],
                prediction_accepted=accepted, **params))
            if not accepted:
                raise ValueError('received permission constraint rejected prediction: '+car)

    def close(self):
        self.executor.shutdown()
        for node in [self.manager, *self.cars.values()]:
            node.destroy_node()
        rclpy.shutdown()
