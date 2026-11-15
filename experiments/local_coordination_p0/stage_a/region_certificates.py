"""Stage A data identities, reset boxes and the current witnessed continuation."""
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from moving_objects import content_id,CARS
from pilot_control import H,pose_rows,raw_pose


def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,default=str,ensure_ascii=False,indent=2)+'\n')
def controls(cert):return [tuple(map(F,u)) for u in cert['inputs']]


def check_box(row,cert):
    d=row['domain'];k=row['phase'];c=F(3,2);Q=cert['scale']
    assert row['object_sha256']==cert['_object_id'] and 0<k<cert['ticks']
    assert d['kind']=='flow_union_five_state_reset_box' and d['next_phase']==k+1
    assert d['next_component']=='inherited_exact_flow'
    rp,rt,rv,rd=[F(d[n]) for n in ('position_l1_radius','yaw_radius','speed_radius','steering_radius')]
    assert min(rp,rt,rv,rd)>0
    v,v1,delta,d1=map(F,cert['knots'][k]);a,w=controls(cert)[k]
    assert F(d['center_v'])==v and F(d['center_delta'])==delta
    assert F(d['feedback_a_ref'])==a and F(d['feedback_omega_ref'])==w
    pose=cert['pose_nodes_Q'][k]
    assert d['center_pose_Q']==[[lo+hi,2*Q] for lo,hi in pose]
    ep=sum(F(hi-lo,2*Q) for lo,hi in pose[:2]);et=F(pose[2][1]-pose[2][0],2*Q)
    distance=sum(max(abs(F(lo,Q)-F(cert['waypoints'][0][j])),abs(F(hi,Q)-F(cert['waypoints'][0][j]))) for j,(lo,hi) in enumerate(pose[:2]))+c*H*(v+v1)/2
    kap,kd=F(d['kappa_upper']),F(d['kappa_delta_upper']);q=kap*rv+max(v,v1)*kd*rd
    theta=rt+et+H*q/2;position=rp+ep+c*H*rv/2+c*max(v,v1)*(H*(rt+et)+H*H*q/3)
    P,T=map(F,cert['source_radius'])
    assert position+c*distance*theta<=P and theta<=T
    assert F(d['reset_pose_source_slack'])==P-position-c*distance*theta
    assert F(d['reset_yaw_source_slack'])==T-theta
    assert abs(a)+rv/H<=1 and abs(w)+rd/H<=1 and 0<=v-rv<=v+rv<=2 and abs(delta)+rd<=F(1,2)
    return True


def in_box(model,state,actor,row):
    d=row['domain'];pose=state['poses'][actor]
    center=[F(*pair) for pair in d['center_pose_Q']]
    return (sum((pose[j]-center[j]).abs_upper() for j in (0,1))<=F(d['position_l1_radius'])
        and (pose[2]-center[2]).abs_upper()<=F(d['yaw_radius'])
        and abs(state['v'][actor]-F(d['center_v']))<=F(d['speed_radius'])
        and abs(state['delta'][actor]-F(d['center_delta']))<=F(d['steering_radius']))


def reset_input(v,delta,row):
    d=row['domain']
    return (F(d['feedback_a_ref'])-(v-F(d['center_v']))/H,
            F(d['feedback_omega_ref'])-(delta-F(d['center_delta']))/H)


def in_flow(model,state,cert,phase):
    actor=cert['car'];p=state['poses'][actor];nom=raw_pose(model,cert['pose_nodes_Q'][phase]);origin=raw_pose(model,cert['pose_nodes_Q'][0])
    angle=p[2]-nom[2];si,co=model.sincos(angle);dx,dy=nom[0]-origin[0],nom[1]-origin[1]
    error=[p[0]-nom[0]-(co-1)*dx+si*dy,p[1]-nom[1]-si*dx-(co-1)*dy]
    P,T=map(F,cert['source_radius']);v,_,d,_=map(F,cert['knots'][phase])
    return sum(x.abs_upper() for x in error)<=P and angle.abs_upper()<=T and state['v'][actor]==v and state['delta'][actor]==d


class Certificates:
    def __init__(self,root):
        self.root=Path(root);self.manifest=read(self.root/'manifest.json')
        for n,d in self.manifest['files'].items():assert sha(self.root/n)==d,'input identity: '+n
        self.cfg=read(self.root/'config.json');self.witness=read(self.root/'witness.json')
        self.rows={x['phase']:x for x in read(self.root/'interfaces.json')};self.objects={}
        for e in self.witness['chain']:
            key=e['object'];path=self.root/'objects'/(key+'.json');assert sha(path)==key
            self.objects[key]={**read(path),'_object_id':key}
        self.edge=self.witness['chain'][0];self.cert=self.objects[self.edge['object']]
        for row in self.rows.values():check_box(row,self.cert)
        chain=self.witness['chain'];assert chain[0]['source']=='0000' and chain[-1]['target']=='3333'
        for i,e in enumerate(chain):
            obj=self.objects[e['object']]
            assert obj['car']==e['actor'] and obj['ticks']==e['ticks']==len(obj['inputs'])
            assert e['source'][CARS.index(e['actor'])]==str(obj['source']) and e['target'][CARS.index(e['actor'])]==str(obj['target'])
            if i:assert chain[i-1]['target']==e['source']
            assert self.witness['U'][e['source']]==sum(x['ticks'] for x in chain[i:])
        assert self.witness['U']['3333']==0 and sum(e['ticks'] for e in chain)==self.witness['accepted_budget']<=self.witness['old_budget']
        self.witness_id=content_id(self.witness)
        self.suffix_ticks=sum(e['ticks'] for e in chain[1:])

    def row(self,phase):return self.rows[phase]
