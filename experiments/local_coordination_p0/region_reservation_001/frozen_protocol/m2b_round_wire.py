"""Directed reliable DDS envelopes collected as a cohort, not send/wait per edge."""
import copy
import json
import time
from m2b_protocol import canonical


class RoundWire:
    def __init__(self,cars):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from std_msgs.msg import String
        self.rclpy=rclpy;self.String=String;rclpy.init()
        self.members=tuple(cars)+('channel',);self.nodes={c:Node('m2b_round_endpoint_'+str(i)) for i,c in enumerate(self.members)}
        self.executor=SingleThreadedExecutor();self.pubs={};self.subs=[];self.received={};self.seen=set();self.log=[];self.sequence=0;self.cycle=0
        qos=QoSProfile(depth=128,reliability=ReliabilityPolicy.RELIABLE)
        for receiver,node in self.nodes.items():
            self.executor.add_node(node);topic='/m2b_round/to_'+str(self.members.index(receiver))
            self.subs.append(node.create_subscription(String,topic,lambda msg,r=receiver:self.receive(r,msg),qos))
            for sender,snode in self.nodes.items():self.pubs[sender,receiver]=snode.create_publisher(String,topic,qos)
        self.spin_until(lambda:all(p.get_subscription_count()==1 for p in self.pubs.values()))

    def spin_until(self,test):
        deadline=time.monotonic()+10.
        while not test():
            if time.monotonic()>deadline:raise TimeoutError('bounded DDS cohort delivery timeout')
            self.executor.spin_once(timeout_sec=.001)

    def receive(self,receiver,msg):
        envelope=json.loads(msg.data);seq=envelope['sequence']
        if envelope['receiver']!=receiver or envelope['cycle']!=self.cycle:raise ValueError('misrouted/stale DDS envelope')
        if seq in self.seen:raise ValueError('duplicate DDS delivery')
        self.seen.add(seq)
        self.received[seq]=envelope
        self.log.append(dict(event='receive',cycle=self.cycle,sequence=seq,wall_ns=time.monotonic_ns(),receiver=receiver))

    def exchange(self,batch):
        expected=[]
        for sender,receiver,kind,payload in batch:
            if sender not in self.nodes or receiver not in self.nodes:raise ValueError('unknown DDS sender/receiver')
            envelope=dict(sender=sender,receiver=receiver,kind=kind,cycle=self.cycle,sequence=self.sequence,payload=payload)
            self.sequence+=1;raw=canonical(envelope);expected.append(envelope['sequence'])
            self.log.append(dict(event='send',wall_ns=time.monotonic_ns(),application_bytes=len(raw.encode()),**copy.deepcopy(envelope)))
            self.pubs[sender,receiver].publish(self.String(data=raw))
        self.spin_until(lambda:all(seq in self.received for seq in expected))
        result=[self.received.pop(seq) for seq in expected]
        return result

    def close(self):
        self.executor.shutdown()
        for node in self.nodes.values():node.destroy_node()
        self.rclpy.shutdown()
