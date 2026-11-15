"""Four independent stop/steer routes and all-pair rotated-body checks."""
import copy
import itertools
import json
import math
from pathlib import Path
import numpy as np
from bicycle_core import corners,gaps,outside_area,rollout,steer,drive,completed
from longitudinal_joint import pose

CARS=('A1','A2','B1','B2')
ORDER=('A1','B1','A2','B2')
SIGMA={'A1':1,'A2':1,'B1':-1,'B2':-1}
PREDECESSOR={'A1':None,'A2':'A1','B1':None,'B2':'B1'}
ROOT=Path(__file__).resolve().parents[1]


def build_scene():return json.loads((ROOT/'config/m2b_fourcar.json').read_text())

def audit(motions,scene):
    if any(not np.isfinite(z).all() for z in motions.values()):return dict(passed=False,residuals={'non_finite':True},pair_gaps_m={},minimum_gap_m=None)
    residuals={};bodies={c:corners(z,scene['vehicle']) for c,z in motions.items()};p=scene['vehicle']
    for c,z in motions.items():
        residuals[c]=dict(outside_road_area_m2=float(outside_area(bodies[c],scene).max()),forward_speed_m_s=float(max(0.,z[:,3].max()-p['v_fwd'])),reverse_speed_m_s=float(max(0.,-z[:,3].min()-p['v_rev'])),steering_rad=float(max(0.,abs(z[:,4]).max()-p['delta_max'])))
    pairs={a+':'+b:float(gaps(bodies[a],bodies[b]).min()) for a,b in itertools.combinations(motions,2)}
    residuals['pair_gap_m']=max([0.]+[scene['s_min']-v for v in pairs.values()])
    values=[v for r in residuals.values() for v in (r.values() if isinstance(r,dict) else [r])]
    return dict(passed=bool(all(v<=scene['tolerance'] for v in values)),residuals=residuals,pair_gaps_m=pairs,minimum_gap_m=min(pairs.values()) if pairs else None)


def build_routes(scene):
    result={};r=scene['turn_radius_m'];v=scene['vehicle']
    for c in CARS:
        origin=np.array(scene['initial'][c],float);path=[];offset=0.;sigma=SIGMA[c];head=np.array(scene['queue_heads'][c[0]],float);tail=np.array(scene['queue_tails'][c[0]],float)
        def add(length,delta,stage):
            nonlocal origin,offset
            if length<1e-8:return
            samples=np.linspace(0,length,max(2,math.ceil(length/.02)+1));states=np.array([pose(origin,s,delta,1,v) for s in samples]);body=corners(states,v)
            path.append(dict(length=float(length),delta=float(delta),direction=1,stage=stage,start=origin.tolist(),end=states[-1].tolist(),offset=offset,speed_cap=1. if abs(delta)>1e-12 else 2.,admission_bound=float(length),body_xmin=float(body[:,:,0].min()),body_xmax=float(body[:,:,0].max())))
            offset+=length;origin=states[-1]
        def shift(world_dy,stage):
            if abs(world_dy)<1e-8:return
            angle=math.acos(1-abs(world_dy)/(2*r));delta=math.copysign(math.atan(v['L']/r),world_dy*sigma)
            add(r*angle,delta,stage);add(r*angle,-delta,stage)
        # A follower waits for its leader to reach the head before it merges.
        destination=tail if PREDECESSOR[c] else head
        shift(destination[1]-origin[1],'APPROACH_MERGE')
        add(sigma*(destination[0]-origin[0]),0.,'APPROACH_MERGE')
        advance_index=len(path)
        if PREDECESSOR[c]:add(sigma*(head[0]-origin[0]),0.,'QUEUE_ADVANCE')
        entry_index=len(path);shift(-origin[1],'ENTER')
        add(34.-sigma*origin[0],0.,'PASS')
        shift(scene['goals'][c][1]-origin[1],'EXIT_PARK')
        add(sigma*(scene['goals'][c][0]-origin[0]),0.,'EXIT_PARK')
        if not completed(origin,scene['goals'][c],scene):raise ValueError('route endpoint misses goal '+c)
        result[c]=dict(segments=path,advance_index=advance_index,entry_index=entry_index)
    return result


def nominal_route(car,route,scene,start=0,end=None):
    segments=route['segments'];end=len(segments) if end is None else end
    z=np.array(segments[start]['start']);z[4]=0.;commands=[]
    for segment in segments[start:end]:
        commands+=steer(z[4],segment['delta'],scene,segment['stage'])
        z[4]=segment['delta'];commands+=drive(segment['length'],scene,segment['stage'],segment['speed_cap'])
    commands+=steer(z[4],0.,scene,'STOP_STEER')
    return rollout(np.array(segments[start]['start']) if start else np.array(scene['initial'][car]),commands,scene),commands


def check_scenario(scene,routes):
    initial=audit({c:np.array([scene['initial'][c]]) for c in CARS},scene);goals=audit({c:np.array([scene['goals'][c]]) for c in CARS},scene);checks={};nominal={}
    for c in CARS:
        states,commands=nominal_route(c,routes[c],scene);nominal[c]=dict(duration_s=len(commands)*scene['h'],goal_passed=completed(states[-1],scene['goals'][c],scene),check=audit({c:states},scene))
        checks[c+'_route']=nominal[c]['check']['passed'] and nominal[c]['goal_passed']
        pred=PREDECESSOR[c]
        if pred:
            states,_=nominal_route(c,routes[c],scene,end=routes[c]['advance_index'])
            head=scene['queue_heads'][c[0]];peer=np.array([*head,0. if SIGMA[c]==1 else math.pi,0.,0.]);q=audit({c:states,pred:np.tile(peer,(len(states),1))},scene)
            checks[c+'_merge_leader_at_head']=q['passed']
        # Later traffic must also clear already parked vehicles.
        states,_=nominal_route(c,routes[c],scene,start=routes[c]['entry_index'])
        same=[x for x in CARS if x[0]==c[0] and x!=c]
        for p in same:
            checks[c+'_parked_'+p]=audit({c:states,p:np.tile(scene['goals'][p],(len(states),1))},scene)['passed']
    return dict(passed=bool(initial['passed'] and goals['passed'] and all(checks.values())),entry_distances_m={c:abs(scene['initial'][c][0])-10 for c in CARS},initial=initial,goals=goals,initial_minimum_gap_m=initial['minimum_gap_m'],goal_minimum_gap_m=goals['minimum_gap_m'],maneuver_checks=checks,nominal=nominal)


class Agent:
    def __init__(self,car,route,scene):
        self.car=car;self.route=route;self.path=route['segments'];self.scene=scene;self.z=np.array(scene['initial'][car],float);self.index=0;self.s=0.;self.previous=0.;self.stop_steps=None;self.steering=[];self.mode='WAIT_MERGE' if PREDECESSOR[car] else 'STEER';self.queue_arrived=False
    @property
    def segment(self):return self.path[min(self.index,len(self.path)-1)]
    def advance(self,permission,peers):
        if self.mode=='MOVE' and self.s>=self.segment['length']-1e-6 and abs(self.z[3])<1e-6:
            if abs(self.z[3])>1e-12:self.stop_steps=1;return
            self.index+=1;self.s=0.;self.previous=0.;self.stop_steps=None
            if self.index==self.route['entry_index']:self.queue_arrived=True;self.mode='READY'
            elif PREDECESSOR[self.car] and self.index==self.route['advance_index']:self.mode='QUEUE_WAIT'
            else:self.mode='STEER'
        pred=PREDECESSOR[self.car];peer=peers.get(pred,{})
        if self.mode=='WAIT_MERGE' and peer.get('queue_arrived'):self.mode='STEER'
        if self.mode=='QUEUE_WAIT' and peer.get('left_queue'):self.mode='STEER'
        if self.mode=='READY' and permission:self.mode='STEER'
        if self.mode=='STEER' or (self.mode in ('READY','QUEUE_WAIT') and not self.steering and abs(self.z[4])>1e-7):
            target=0. if self.index==len(self.path) or self.mode in ('READY','QUEUE_WAIT') else self.segment['delta']
            if not self.steering:self.steering=steer(self.z[4],target,self.scene,self.segment['stage'])
            if not self.steering and self.mode=='STEER':self.mode='DONE' if self.index==len(self.path) else 'MOVE'
    def parameters(self,permission):
        if self.mode!='MOVE':return dict(s0=self.s,v0=0.,end=self.s,upper=self.s,cap=0.,direction=1,stop_at_end=True,stop_index=0)
        p=self.segment;T=self.scene['h']*self.scene['N'];cap=p['speed_cap'];accel=min(self.scene['vehicle']['a_max'],-self.scene['vehicle']['a_min'])
        if p['length']-self.s<=cap*max(0.,T-2*cap/accel) and self.stop_steps is None:self.stop_steps=self.scene['N']
        return dict(s0=self.s,v0=self.z[3],end=p['length'],upper=p['length'],cap=cap,direction=1,stop_at_end=self.stop_steps is not None,stop_index=self.stop_steps or self.scene['N'])
