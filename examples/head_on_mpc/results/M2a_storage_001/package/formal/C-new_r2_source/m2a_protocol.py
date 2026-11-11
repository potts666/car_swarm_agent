"""M2a-release-v1 and final-plan agreement; no optimizer or future rollout in channel."""
import copy
import hashlib
import json
import time
import numpy as np
from bicycle_core import CARS,SIGN,corners,clipped_area,audit,rollout

CONTRACT='M2a-release-v1'
COORDINATION_ROUNDS=1


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def plan_id(plan):return hashlib.sha256(canonical({k:v for k,v in plan.items() if k!='id'}).encode()).hexdigest()
def protection(scene):return [-scene['admission_protection_abs_x'],scene['admission_protection_abs_x'],-scene['W']/2,scene['W']/2]
def cleared(car,z,scene):
    body=corners([z],scene['vehicle'])[0]
    return bool(np.min(SIGN[car]*body[:,0])>=scene['release_plane_abs_x']+scene['release_margin_m']-scene['tolerance'])


def public_report(agent,context):
    r=dict(car=agent.car,z=agent.z.tolist(),index=agent.index,mode=agent.mode,ready=agent.ready,
        steering_complete=not agent.steering,context=copy.deepcopy(context))
    if agent.car=='B':
        r.update(entry_envelope_xmin=min(p['body_xmin'] for p in agent.path[1:3]),return_envelope_xmax=max(p['body_xmax'] for p in agent.path[3:]))
    return r


def make_plan(report,commands,states,kind,revision,origin,shifted_from=None):
    plan=dict(car=report['car'],context=copy.deepcopy(report['context']),report=copy.deepcopy(report),
        commands=[list(u) for u in commands],states=np.asarray(states).tolist(),kind=kind,revision=revision,origin=origin,shifted_from=shifted_from)
    plan['id']=plan_id(plan);return plan


def check_plan(plan,report,context,scene):
    if set(plan)!={'car','context','report','commands','states','kind','revision','origin','shifted_from','id'}:raise ValueError('invalid public plan schema')
    if plan['id']!=plan_id(plan):raise ValueError('plan changed without new ID')
    if plan['context']!=context or report['context']!=context or plan['report']!=report or plan['car']!=report['car']:raise ValueError('stale plan/report context')
    z=np.asarray(plan['states'],float);commands=plan['commands'];N=scene['N'];n=round(scene['h']/scene['sample_s'])
    if z.shape!=(N*n+1,5) or len(commands)!=N or not np.isfinite(z).all():raise ValueError('bad prediction shape')
    if np.max(abs(z[0]-report['z']))>1e-12:raise ValueError('prediction not initialized at current state')
    replayed=rollout(np.asarray(report['z'],float),commands,scene)
    if np.max(abs(z-replayed))>1e-9:raise ValueError('final inputs/prediction mismatch')
    if not audit({plan['car']:z},scene)['passed']:raise ValueError('single vehicle physical rejection')
    bodies=corners(z,scene['vehicle'])
    if context['owner']!=plan['car'] and any(clipped_area(b,protection(scene))>scene['tolerance'] for b in bodies):raise ValueError('unauthorized predicted entry')
    if plan['car'] in context['released'] or plan['car']==context.get('releasing'):
        if any(not cleared(plan['car'],q,scene) for q in z):raise ValueError('released owner reentry')
    return z


def acknowledge(car,own,peer,own_report,peer_report,context,scene):
    if own['kind']!='final' or peer['kind']!='final':raise ValueError('seed cannot be acknowledged for commit')
    z=check_plan(own,own_report,context,scene);other=check_plan(peer,peer_report,context,scene)
    if not audit({car:z,peer['car']:other},scene)['passed']:raise ValueError('incompatible final predictions')
    return dict(car=car,context=copy.deepcopy(context),plans={car:own['id'],peer['car']:peer['id']},accepted=True)


class Barrier:
    def __init__(self,context):self.context=copy.deepcopy(context);self.plans={};self.acks={}
    def publish(self,plan):
        if plan['context']!=self.context or plan['id']!=plan_id(plan):raise ValueError('stale or modified final publication')
        if plan['kind']!='final':raise ValueError('only final plans enter barrier')
        if self.plans.get(plan['car'])!=plan['id']:self.acks={}
        self.plans[plan['car']]=plan['id']
    def accept(self,ack):
        if ack['context']!=self.context or ack['plans']!=self.plans or not ack['accepted']:raise ValueError('stale acknowledgement')
        if ack['car'] not in CARS:raise ValueError('unknown acknowledger')
        self.acks[ack['car']]=copy.deepcopy(ack)
    def token(self):
        if set(self.plans)!=set(CARS) or set(self.acks)!=set(CARS):raise ValueError('missing compatible final versions')
        return dict(context=copy.deepcopy(self.context),plans=copy.deepcopy(self.plans))


class Channel:
    def __init__(self,scene,waiting_pose):
        self.scene=scene;self.waiting_pose=np.asarray(waiting_pose);self.owner=None;self.version=0;self.pending={};self.released=[];self.requested=set();self.proposal=None;self.events=[]
    def event(self,cycle,kind,**data):self.events.append(dict(cycle=cycle,kind=kind,**data))
    def request(self,car,cycle):
        if car not in CARS or car in self.requested or car in self.released:raise ValueError('invalid/repeated request')
        self.requested.add(car);self.pending[car]=cycle;self.event(cycle,'request',car=car)
    def grant_initial(self,cycle):
        if self.owner is None and not self.released and self.pending:
            self.owner=min(self.pending,key=lambda c:(self.pending[c],c));del self.pending[self.owner];self.version+=1
            self.event(cycle,'grant',car=self.owner,version=self.version)
    def context(self,cycle):
        return dict(cycle=cycle,state_time_s=round(self.scene['epoch_s']+cycle*self.scene['h'],10),permission_version=self.version,
            owner=self.owner,released=self.released.copy(),proposal_id=None,releasing=None,conditional=False)
    def waiting_ready(self,report):
        z=np.asarray(report['z']);bodies=corners([z],self.scene['vehicle'])
        return bool(report['car']=='B' and report['mode']=='WAIT' and report['index']==3 and report['ready'] and report['steering_complete']
            and abs(z[3])<=1e-12 and abs(z[4])<1e-7 and np.linalg.norm(z[:2]-self.waiting_pose[:2])<=.005
            and abs(z[2]-self.waiting_pose[2])<=.003 and clipped_area(bodies[0],protection(self.scene))<=self.scene['tolerance'])
    def propose_release(self,reporter,authorization_version,reports,cycle):
        active=self.context(cycle)
        if reporter!=self.owner or authorization_version!=self.version or reporter is None:raise ValueError('nonowner/stale release')
        if self.proposal is not None:raise ValueError('release already prepared')
        if any(reports[c]['context']!=active for c in CARS):raise ValueError('release reports not from same active snapshot')
        if not cleared(reporter,reports[reporter]['z'],self.scene):raise ValueError('actual rotated body not clear')
        if reporter=='A' and not self.waiting_ready(reports['B']):raise ValueError('waiting vehicle not physically ready')
        eligible=[c for c in self.pending if c=='B' and self.waiting_ready(reports[c])]
        next_owner=min(eligible,key=lambda c:(self.pending[c],c)) if eligible else None
        if self.pending and next_owner is None:raise ValueError('pending next owner not ready')
        proposed=copy.deepcopy(active);proposed.update(permission_version=self.version+1,owner=next_owner,proposal_id=f'{cycle}:{self.version}:{reporter}',releasing=reporter,conditional=True)
        self.proposal=dict(active=active,reports=copy.deepcopy(reports),proposed=proposed,reporter=reporter)
        self.event(cycle,'release_prepared',old_owner=reporter,next_owner=next_owner,old_version=self.version,proposed=proposed)
        return copy.deepcopy(proposed)
    def commit_release(self,barrier):
        p=self.proposal
        if p is None:raise ValueError('no prepared release')
        token=barrier.token()
        if token['context']!=p['proposed'] or self.context(p['active']['cycle'])!=p['active']:raise ValueError('release transaction invalidated')
        if not cleared(p['reporter'],p['reports'][p['reporter']]['z'],self.scene):raise ValueError('clear evidence invalidated')
        old=self.owner;self.released.append(old);self.owner=p['proposed']['owner'];self.version=p['proposed']['permission_version']
        if self.owner is not None:del self.pending[self.owner]
        cycle=p['active']['cycle'];self.event(cycle,'handoff',old_owner=old,owner=self.owner,version=self.version,proposal_id=p['proposed']['proposal_id'],plans=token['plans'])
        self.proposal=None;return self.context(cycle)
    def cancel(self,cycle,reason):
        if self.proposal:self.event(cycle,'release_cancelled',reason=reason,proposal_id=self.proposal['proposed']['proposal_id'])
        self.proposal=None


class Wire:
    """Three actual ROS2 nodes; compact JSON envelope copied over reliable DDS."""
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from std_msgs.msg import String
        self.rclpy=rclpy;self.String=String;rclpy.init();self.nodes={c:Node('m2a_'+c.lower()) for c in ('A','B','channel')};self.executor=SingleThreadedExecutor();self.inbox={c:[] for c in self.nodes};self.log=[];self.cycle=0
        self.pubs={};self.subs=[];qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
        for name,node in self.nodes.items():
            self.executor.add_node(node);self.pubs[name]=node.create_publisher(String,'/m2a/envelope',qos)
            self.subs.append(node.create_subscription(String,'/m2a/envelope',lambda msg,n=name:self.receive(n,msg),qos))
        self.wait(lambda:all(p.get_subscription_count()==3 for p in self.pubs.values()))
    def wait(self,predicate):
        end=time.monotonic()+10
        while not predicate():
            if time.monotonic()>end:raise TimeoutError('M2a DDS delivery timeout')
            self.executor.spin_once(timeout_sec=.01)
    def receive(self,node,msg):
        envelope=json.loads(msg.data)
        if envelope['receiver']==node:
            self.inbox[node].append(envelope)
            self.log.append(dict(event='receive',cycle=self.cycle,sequence=envelope['sequence'],wall_ns=time.monotonic_ns(),receiver=node))
    def send(self,sender,receiver,kind,payload):
        seq=sum(r['event']=='send' for r in self.log)
        envelope=dict(sender=sender,receiver=receiver,kind=kind,cycle=self.cycle,sequence=seq,payload=payload)
        raw=canonical(envelope);target=len(self.inbox[receiver])+1
        self.log.append(dict(event='send',wall_ns=time.monotonic_ns(),application_bytes=len(raw.encode()),**envelope))
        self.pubs[sender].publish(self.String(data=raw));self.wait(lambda:len(self.inbox[receiver])==target)
        return copy.deepcopy(self.inbox[receiver][-1]['payload'])
    def close(self):
        self.executor.shutdown()
        for node in self.nodes.values():node.destroy_node()
        self.rclpy.shutdown()
