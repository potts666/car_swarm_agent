"""Read actual four-car execution records; midpoint is for display only."""
import bisect
import copy
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path

CARS=('A1','A2','B1','B2')
LEGACY_RUN=Path(__file__).resolve().parents[1]/'results/pilot_032/asymmetric_baseline_M-T'
DEFAULT_RUN=Path(__file__).resolve().parents[1]/'encounter_coordination_001'


class Recording:
    def __init__(self,directory):
        self.directory=Path(directory).resolve()
        if (self.directory/'results/actual.jsonl.gz').is_file():
            self._read_joint();return
        config=json.loads((self.directory/'config.json').read_text());self.summary=json.loads((self.directory/'summary.json').read_text())
        acceptance=json.loads((self.directory/'independent_acceptance.json').read_text())
        if not self.summary.get('passed') or not acceptance.get('passed'):raise ValueError('Select a run with passed execution and independent acceptance')
        scene=config.get('scene')
        if scene is None:
            source=Path(config['inputs'])/'config.json' if 'inputs' in config else self.directory.parents[2]/'inputs/config.json'
            scene=json.loads(source.read_text())
        scene=copy.deepcopy(scene)
        scene['vehicle']={k:float(Fraction(v)) for k,v in scene['vehicle'].items()}
        for key in ('initial','goals'):scene[key]={c:[float(Fraction(v)) for v in z] for c,z in scene[key].items()}
        scene['road_rectangles']=[[float(Fraction(v)) for v in r] for r in scene['road_rectangles']]
        self.config=scene;self.method=config.get('method',self.summary.get('method','interface'))
        self.dt=float(Fraction(config.get('physical_dt_s',scene['h'])));self.scale=10**20
        if set(scene['initial'])!=set(CARS):raise ValueError('This replay expects A1/A2/B1/B2')
        states={c:[float(Fraction(x)) for x in scene['initial'][c]] for c in CARS}
        self.frames=[dict(states=copy.deepcopy(states),actor=None,groups=[],budget=self.summary.get('initial_budget_ticks',self.summary.get('source_budget')),phase=None,kind='initial')]
        self.paths={c:[(states[c][0],states[c][1])] for c in CARS};self.path_frames={c:[0] for c in CARS}
        path=self.directory/'execution.jsonl.gz';self.execution_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        with gzip.open(path,'rt',encoding='utf8') as stream:
            for i,line in enumerate(stream):
                row=json.loads(line);actor=row.get('actor')
                if actor is None:actor=self.summary['edge']['actor']
                if actor not in CARS:raise ValueError('unknown executed actor')
                if row.get('cycle',i)!=i:raise ValueError('execution cycle is not contiguous')
                actual=row['actual'];pose=[(lo+hi)/(2*self.scale) for lo,hi in actual['actual_pose_Q']]
                # Logged B heading is theta; its physical yaw is pi+theta.
                if actor.startswith('B'):pose[2]+=math.pi
                states[actor]=pose+[float(Fraction(actual['v'])),float(Fraction(actual['delta']))]
                groups=row.get('groups',row.get('token',{}).get('context',{}).get('groups',[]))
                self.frames.append(dict(states=copy.deepcopy(states),actor=actor,groups=groups,budget=row['budget_after'],phase=row.get('phase'),kind=row.get('kind','executed'),candidate_accepted=row.get('optimizer',{}).get('changed',False) or bool(row.get('accepted_result_id'))))
                xy=tuple(states[actor][:2])
                if xy!=self.paths[actor][-1]:self.paths[actor].append(xy);self.path_frames[actor].append(i+1)
        self.duration=(len(self.frames)-1)*self.dt;self.full_task=bool(acceptance.get('full_task',self.summary.get('all_goals_reached',False)))
        if len(self.frames)-1!=acceptance['actual_steps']:raise ValueError('trace length differs from independent acceptance')

    def _read_joint(self):
        root=self.directory;gate=json.loads((root/'GATE.json').read_text());acceptance=json.loads((root/'results/independent_physical.json').read_text())
        if not gate.get('passed') or not acceptance.get('passed'):raise ValueError('Joint replay requires passed gate and independent physical acceptance')
        cfg=json.loads((root/'vehicle_configs.json').read_text());tasks=json.loads((root/'local_tasks.json').read_text());road=json.loads((root/'map_config.json').read_text())
        plan=json.loads(gzip.decompress((root/'results/updated.json.gz').read_bytes()));self.event=plan.get('encounter',{});self.goal_ticks={c:plan['per_car_policies'][c]['goal'] for c in CARS};self.dt=float(Fraction(cfg['h']));self.scale=10**20
        bx0,bx1=map(Fraction,cfg['body_x']);by0,by1=map(Fraction,cfg['body_y']);wheelbase=Fraction(cfg['L'])
        def state(values,car):
            z=[float(Fraction(v)) for v in values];z[2]+=math.pi if cfg['frames'][car]<0 else 0.;return z
        initial={c:state(tasks[c]['initial'],c) for c in CARS};goals={c:state(tasks[c]['goal'],c) for c in CARS}
        self.config={'vehicle':{'L':float(wheelbase),'front':float(bx1-wheelbase),'rear':float(-bx0),'length':float(bx1-bx0),'width':float(by1-by0)},'initial':initial,'goals':goals,'road_rectangles':[[float(Fraction(v)) for v in r] for r in road['roads']],'resource_boundaries':[float(Fraction(v)) for v in road['resource_x']]}
        path=root/'results/actual.jsonl.gz';self.execution_sha256=hashlib.sha256(path.read_bytes()).hexdigest();hashes=json.loads((root/'SHA256SUMS.json').read_text())
        if hashes['results/actual.jsonl.gz']!=self.execution_sha256:raise ValueError('Execution log hash mismatch')
        self.paths={c:[tuple(initial[c][:2])] for c in CARS};self.path_frames={c:[0] for c in CARS};self.method=root.name+' / E-MPC'
        registration=json.loads((root/'event_registration.json').read_text()) if (root/'event_registration.json').exists() else {'source_cycle':0}
        old=json.loads(gzip.decompress((root/'results/R_old.json.gz').read_bytes())) if (root/'results/R_old.json.gz').exists() else plan
        def frame(index,states):
            current=old if index<registration['source_cycle'] else plan;active=[c for c,z in states.items() if abs(z[3])>1e-9];owner=plan['owners'][index] if index<len(plan['owners']) else None
            return dict(states=states,actor=','.join(active) or None,active=active,groups=[list(CARS)],owner=owner,budget=max(max(0,current['per_car_policies'][c]['goal']-index) for c in CARS),phase=index,kind='actual simultaneous execution')
        self.frames=[frame(0,initial)]
        with gzip.open(path,'rt') as stream:
            for index,line in enumerate(stream,1):
                row=json.loads(line)
                if row['cycle']!=index:raise ValueError('Noncontiguous simultaneous execution log')
                states={}
                for c in CARS:
                    pose=[(lo+hi)/(2*self.scale) for lo,hi in row['poses_Q'][c]];pose[2]+=math.pi if cfg['frames'][c]<0 else 0.;states[c]=pose+[float(Fraction(row['v'][c])),float(Fraction(row['delta'][c]))]
                    xy=tuple(pose[:2])
                    if xy!=self.paths[c][-1]:self.paths[c].append(xy);self.path_frames[c].append(index)
                self.frames.append(frame(index,states))
        if len(self.frames)-1!=acceptance['steps'] or acceptance['steps']!=plan['B0']:raise ValueError('Actual trace differs from independent certificate')
        for c,z in self.frames[-1]['states'].items():
            if abs(z[0]-goals[c][0])+abs(z[1]-goals[c][1])>.08 or z[3]!=0 or z[4]!=0:raise ValueError('Final holding state is not a certified goal')
        self.duration=(len(self.frames)-1)*self.dt;self.full_task=True;self.summary={'passed':True,'all_goals_reached':True,'initial_budget_ticks':old['B0']}

    def stage(self,index):
        if not hasattr(self,'event') or not self.event:return 'actual accepted execution'
        if index==len(self.frames)-1:return 'ALL FOUR GOALS - zero-speed holding'
        yielding=self.event['yielding'];passing=self.event['passing'];out=self.event['first_out'];stop=next((i for i,f in enumerate(self.frames[1:],1) if f['states'][yielding][3]==0),0)
        if index<self.event['source_cycle']:return 'before encounter'
        if index<stop:return passing+' PASS / '+yielding+' APPROACH AND BRAKE'
        if index<out:return yielding+' WAIT / '+passing+' PASSAGE - new contract'
        if index<self.event['yield_service_start']+5:return 'ACTUAL EXIT GUARD PASSED / '+yielding+' STEERING'
        if index<self.goal_ticks[yielding]:return yielding+' RESUMED / other vehicles continue'
        return 'remaining vehicles completing'

    def sample(self,seconds):
        # A zero-order display of accepted logged states, not a fabricated rollout.
        index=min(len(self.frames)-1,max(0,int((seconds+1e-9)/self.dt)))
        return index,self.frames[index]

    def trail(self,car,index):return self.paths[car][:bisect.bisect_right(self.path_frames[car],index)]

    def body_center(self,state):
        offset=(self.config['vehicle']['L']+self.config['vehicle']['front']-self.config['vehicle']['rear'])/2
        return state[0]+offset*math.cos(state[2]),state[1]+offset*math.sin(state[2])
