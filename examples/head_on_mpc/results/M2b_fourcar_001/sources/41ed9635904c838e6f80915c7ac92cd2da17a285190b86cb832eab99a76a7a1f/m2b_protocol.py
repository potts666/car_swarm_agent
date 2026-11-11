"""Four-car resource transactions and final-version barrier, without motion optimization."""
import copy
import json
import math
import time
import numpy as np
from bicycle_core import corners,clipped_area,rollout
from m2a_logs import plan_id
from m2a_protocol import canonical
from m2b_scene import CARS,SIGMA,audit

CONTRACT='M2b-release-v1'


def cleared(car,z,scene):return float(np.min(SIGMA[car]*corners([z],scene['vehicle'])[0,:,0]))>=26.5-scene['tolerance']

def ready_report(r,scene):
    c=r['car'];z=np.array(r['z']);head=np.array(scene['queue_heads'][c[0]]);yaw=math.atan2(math.sin(z[2]-(0 if SIGMA[c]==1 else math.pi)),math.cos(z[2]-(0 if SIGMA[c]==1 else math.pi)))
    protection=[-18,18,-1.4,1.4]
    return bool(r['mode']=='READY' and r['steering_complete'] and abs(z[3])<=1e-12 and abs(z[4])<=1e-7 and np.linalg.norm(z[:2]-head)<=.005 and abs(yaw)<=.003 and clipped_area(corners([z],scene['vehicle'])[0],protection)<=scene['tolerance'])

def select_next(candidates,requests,policy,last_direction):
    if not candidates:return None
    choices=list(candidates)
    if policy=='alternating-ready' and last_direction is not None:
        opposite=[c for c in choices if SIGMA[c]!=last_direction]
        if opposite:choices=opposite
    elif policy!='fifo-ready' and policy!='alternating-ready':raise ValueError('unknown policy')
    return min(choices,key=lambda c:(requests[c],CARS.index(c)))


def public_report(agent,context):
    c=agent.car;z=agent.z
    left=bool(float(np.min(SIGMA[c]*corners([z],agent.scene['vehicle'])[0,:,0]))>=-16-agent.scene['tolerance'] and agent.queue_arrived)
    return dict(car=c,z=z.tolist(),mode=agent.mode,index=agent.index,progress_m=agent.s,queue_arrived=agent.queue_arrived,left_queue=left,steering_complete=not agent.steering,context=copy.deepcopy(context))


def context_plan(report,commands,states,kind,origin,shifted_from=None):
    p=dict(car=report['car'],context=copy.deepcopy(report['context']),report=copy.deepcopy(report),commands=[list(u) for u in commands],states=np.asarray(states).tolist(),kind=kind,origin=origin,shifted_from=shifted_from)
    p['id']=plan_id(p);return p


def check_plan(plan,report,context,scene):
    if plan['id']!=plan_id(plan) or plan['context']!=context or plan['report']!=report or plan['car']!=report['car']:raise ValueError('plan content/state/context mismatch')
    if len(plan['commands'])!=scene['N']:raise ValueError('wrong horizon')
    z=rollout(np.array(report['z']),plan['commands'],scene)
    if np.asarray(plan['states']).shape!=z.shape or not np.isfinite(plan['states']).all() or np.max(abs(z-np.asarray(plan['states'])))>1e-12:raise ValueError('plan inputs were repaired without new prediction/ID')
    if not audit({plan['car']:z},scene)['passed']:raise ValueError('plan road/actuator state violation')
    body=corners(z,scene['vehicle']);c=plan['car']
    if context['owner']!=c and any(clipped_area(b,[-18,18,-1.4,1.4])>scene['tolerance'] for b in body):raise ValueError('unauthorized protection entry')
    if c in context['released'] and float(np.min(SIGMA[c]*body[:,:,0]))<26.5-scene['tolerance']:raise ValueError('released plan reenters boundary')
    return z


def acknowledge(car,plans,reports,context,scene):
    if set(plans)!=set(CARS):raise ValueError('missing final prediction')
    states={c:check_plan(p,reports[c],context,scene) for c,p in plans.items()}
    if any(p['kind']!='final' for p in plans.values()):raise ValueError('acknowledgement requires final plans')
    if not audit(states,scene)['passed']:raise ValueError('four-final compatibility rejected')
    return dict(car=car,context=copy.deepcopy(context),plans={c:plans[c]['id'] for c in CARS},accepted=True)


class Barrier:
    def __init__(self,context):self.context=copy.deepcopy(context);self.plans={};self.acks={}
    def publish(self,p):
        if p['context']!=self.context or p['car'] not in CARS or p['kind']!='final':raise ValueError('wrong final context')
        if self.plans.get(p['car'])!=p['id']:self.acks={}
        self.plans[p['car']]=p['id']
    def accept(self,a):
        if a['car'] not in CARS or a['context']!=self.context or a['plans']!=self.plans or set(self.plans)!=set(CARS) or not a['accepted']:raise ValueError('stale or mixed acknowledgement')
        self.acks[a['car']]=copy.deepcopy(a)
    def token(self):
        if set(self.plans)!=set(CARS) or set(self.acks)!=set(CARS):raise ValueError('missing four-plan acknowledgement')
        return dict(context=copy.deepcopy(self.context),plans=copy.deepcopy(self.plans))


class Channel:
    def __init__(self,scene,policy):
        self.scene=scene;self.policy=policy;self.owner=None;self.version=0;self.requests={};self.served=set();self.released=set();self.last_direction=None;self.proposal=None;self.events=[]
    def event(self,cycle,kind,**data):self.events.append(dict(cycle=cycle,kind=kind,**data))
    def request(self,car,cycle):
        if car not in CARS or car in self.served:raise ValueError('invalid request')
        if car in self.requests:return
        self.requests[car]=cycle;self.event(cycle,'request',car=car,first_request_cycle=cycle)
    def context(self,cycle):return dict(cycle=cycle,state_time_s=round(self.scene['epoch_s']+cycle*self.scene['h'],10),owner=self.owner,permission_version=self.version,proposal_id=None,conditional=False,released=sorted(self.released))
    def eligible(self,reports):
        result=[]
        for side in ('A','B'):
            head=next((c for c in CARS if c[0]==side and c not in self.served),None)
            if head and head in self.requests and ready_report(reports[head],self.scene):result.append(head)
        return result
    def prepare(self,reporter,version,reports,cycle):
        active=self.context(cycle)
        if set(reports)!=set(CARS):raise ValueError('missing current vehicle report')
        if version!=self.version or reporter!=self.owner:raise ValueError('nonowner or stale release')
        if any(r['context']!=active for r in reports.values()):raise ValueError('stale state report')
        if self.owner and not cleared(self.owner,reports[self.owner]['z'],self.scene):raise ValueError('actual rotated body not clear')
        candidates=self.eligible(reports);new=select_next(candidates,self.requests,self.policy,self.last_direction)
        if self.owner is None and new is None:return active
        released=self.released|({self.owner} if self.owner else set())
        proposed={**active,'owner':new,'permission_version':self.version+1,'proposal_id':f'{cycle}:{self.version+1}','conditional':True,'released':sorted(released)}
        self.proposal=dict(active=active,context=proposed,candidates=candidates,reports=copy.deepcopy(reports),previous_direction=self.last_direction)
        self.event(cycle,'prepare',old_owner=self.owner,proposed_owner=new,version=self.version+1,proposal_id=proposed['proposal_id'],eligible=candidates,policy=self.policy)
        return copy.deepcopy(proposed)
    def commit(self,barrier):
        token=barrier.token()
        if self.proposal is None or token['context']!=self.proposal['context']:raise ValueError('no active proposal')
        p=self.proposal;ctx=p['context'];old=self.owner;new=ctx['owner'];self.owner=new;self.version=ctx['permission_version'];self.released=set(ctx['released'])
        if new:self.served.add(new);self.last_direction=SIGMA[new]
        self.event(ctx['cycle'],'handoff',old_owner=old,owner=new,version=self.version,proposal_id=ctx['proposal_id'],plans=token['plans'],eligible=p['candidates'],previous_direction=p['previous_direction'])
        self.proposal=None;return self.context(ctx['cycle'])
    def cancel(self,cycle,reason):
        if self.proposal:self.event(cycle,'cancel',proposal_id=self.proposal['context']['proposal_id'],reason=reason)
        self.proposal=None


class Wire:
    """Five reliable ROS nodes in one process; only decoded public envelopes cross endpoints."""
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from std_msgs.msg import String
        self.rclpy=rclpy;self.String=String;rclpy.init();self.nodes={c:Node('m2b_'+c.lower()) for c in (*CARS,'channel')};self.executor=SingleThreadedExecutor();self.inbox={c:[] for c in self.nodes};self.log=[];self.cycle=0;self.sequence=0
        self.pubs={};self.subs=[];qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE)
        for name,node in self.nodes.items():
            self.executor.add_node(node);self.pubs[name]=node.create_publisher(String,'/m2b/envelope',qos);self.subs.append(node.create_subscription(String,'/m2b/envelope',lambda msg,n=name:self.receive(n,msg),qos))
        self.wait(lambda:all(p.get_subscription_count()==5 for p in self.pubs.values()))
    def wait(self,condition):
        end=time.monotonic()+10
        while not condition():
            if time.monotonic()>end:raise TimeoutError('DDS delivery timeout')
            self.executor.spin_once(timeout_sec=.001)
    def receive(self,node,msg):
        envelope=json.loads(msg.data)
        if envelope['receiver']==node:self.inbox[node].append(envelope);self.log.append(dict(event='receive',cycle=self.cycle,sequence=envelope['sequence'],wall_ns=time.monotonic_ns(),receiver=node))
    def send(self,sender,receiver,kind,payload):
        envelope=dict(sender=sender,receiver=receiver,kind=kind,cycle=self.cycle,sequence=self.sequence,payload=payload);self.sequence+=1;raw=canonical(envelope);target=len(self.inbox[receiver])+1
        self.log.append(dict(event='send',wall_ns=time.monotonic_ns(),application_bytes=len(raw.encode()),**envelope));self.pubs[sender].publish(self.String(data=raw));self.wait(lambda:len(self.inbox[receiver])==target)
        result=copy.deepcopy(self.inbox[receiver][-1]['payload']);self.inbox[receiver].clear();return result
    def close(self):
        self.executor.shutdown()
        for node in self.nodes.values():node.destroy_node()
        self.rclpy.shutdown()
