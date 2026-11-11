#!/usr/bin/env python3
"""M2a online C-new/L-new with final-plan barrier and separately invoked offline audit."""
import argparse
import contextlib
import copy
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import time
from unittest.mock import patch
import numpy as np
import scipy
import osqp
from bicycle_core import CARS,SIGN,rollout,audit,completed,validate_scene
from longitudinal_joint import Agent,routes
from linearized_joint import BicycleController
from m2a_local import LocalOptimizer
from m2a_protocol import (Wire,Channel,Barrier,CONTRACT,COORDINATION_ROUNDS,public_report,make_plan,check_plan,acknowledge,cleared)
from run_entry_bicycle import ROOT,write,rows_for,save_rows
from m2a_logs import write_bundle


class Endpoint:
    def __init__(self,car,path,scene):
        self.car=car;self.scene=copy.deepcopy(scene);self.scene['initial']={car:scene['initial'][car]};self.scene['goals']={car:scene['goals'][car]};self.agent=Agent(car,path,self.scene);self.optimizer=LocalOptimizer();self.last=None;self.record=None;self.requested=False;self.last_commit=-1
    def begin(self,context,working=None):
        self.context=copy.deepcopy(context);self.work=working if working is not None else copy.deepcopy(self.agent)
        if working is None:self.work.advance(context['owner']==self.car)
        self.report=public_report(self.work,context);self.peer_report=None;self.peer=None;self.plan=None;self.record=None
    def observe_report(self,report):
        expected={'car','z','index','mode','ready','steering_complete','context'}|({'entry_envelope_xmin','return_envelope_xmax'} if report.get('car')=='B' else set())
        if set(report)!=expected:raise ValueError('invalid public state schema')
        if report['car']==self.car or report['context']!=self.context:raise ValueError('stale peer state report')
        self.peer_report=copy.deepcopy(report)
    def observe_plan(self,plan):
        if self.peer_report is None:raise ValueError('plan before peer state')
        check_plan(plan,self.peer_report,self.context,self.scene);self.peer=copy.deepcopy(plan)
    def seed(self):
        N=self.scene['N'];n=round(self.scene['h']/self.scene['sample_s'])
        if self.last is None:
            if abs(self.work.z[3])>1e-12:raise ValueError('no valid initial prediction for moving state')
            commands=[(0.,0.,'INITIAL_HOLD')]*N;shifted=None
        else:
            if np.max(abs(np.asarray(self.last['states'])[n]-self.work.z))>1e-12:raise ValueError('shifted prediction initial-state mismatch')
            commands=self.last['commands'][1:]+[[0.,0.,'SHIFT_TAIL']];shifted=self.last['id']
        dense=rollout(self.work.z,commands,self.scene)
        return make_plan(self.report,commands,dense,'seed',0,'own_shift_or_initialization',shifted)
    def validate_seed_pair(self,own):
        if self.peer is None:raise ValueError('initialization missing peer prediction')
        own_z=check_plan(own,self.report,self.context,self.scene)
        peer_z=check_plan(self.peer,self.peer_report,self.context,self.scene)
        if not audit({self.car:own_z,self.peer['car']:peer_z},self.scene)['passed']:
            raise ValueError('initialization has no compatible prediction pair')

    def local_candidate(self):
        if self.peer is None:raise ValueError('missing compatible peer prediction')
        commands,dense,record=self.optimizer.predict(self.work,self.context['owner']==self.car,self.peer,self.scene);self.record=record
        if not record['accepted']:raise ValueError(record['message'])
        self.plan=make_plan(self.report,commands,dense,'final',1,'local');return self.plan
    def ack(self):
        if self.peer is None or self.plan is None:raise ValueError('missing final candidate')
        return acknowledge(self.car,self.plan,self.peer,self.report,self.peer_report,self.context,self.scene)
    def check_commit(self,packet):
        token=packet['token'];effective=packet['effective'];context=self.context
        if self.plan is None or token['plans'].get(self.car)!=self.plan['id'] or token['context']!=context:raise ValueError('mixed or superseded commit')
        if context['cycle']<=self.last_commit or effective['cycle']!=context['cycle'] or effective['owner']!=context['owner'] or effective['permission_version']!=context['permission_version']:raise ValueError('commit before effective authorization')
        if context['conditional'] and packet['activated_proposal_id']!=context['proposal_id']:raise ValueError('conditional plan not activated')
        if self.peer['id']!=token['plans'][self.peer['car']]:raise ValueError('peer plan changed after ack')
    def commit(self,packet,central=False):
        self.check_commit(packet);self.last_commit=self.context['cycle'];self.last=copy.deepcopy(self.plan)
        if not central:
            a=self.work;h=self.scene['h'];n=round(h/self.scene['sample_s'])
            self.optimizer.remember(a,self.record)
            if a.mode=='MOVE':
                acceleration=a.segment['direction']*self.plan['commands'][0][0]
                a.s+=h*a.segment['direction']*a.z[3]+.5*h*h*acceleration;a.previous=acceleration
                if a.stop_steps is not None:a.stop_steps-=1
            elif a.steering:a.steering.pop(0)
            a.z=np.array(self.plan['states'][n]);self.agent=a
    def private_checkpoint(self,central_cache=None):
        a=self.work
        return dict(agent={k:(v.tolist() if isinstance(v,np.ndarray) else copy.deepcopy(v)) for k,v in vars(a).items() if k not in ('path','scene')},
                    reference_cache=copy.deepcopy(central_cache if central_cache is not None else self.optimizer.previous))


def exchange_reports(endpoints,wire,context,central_work=None):
    reports={}
    for c,e in endpoints.items():
        e.begin(context,central_work.agents[c] if central_work is not None else None)
        reports[c]=wire.send(c,'channel','state',e.report)
    for c,e in endpoints.items():
        other='B' if c=='A' else 'A';endpoints[other].observe_report(wire.send(c,other,'state',e.report))
    return reports


def online(scene,paths,directory,arm):
    directory.mkdir();wire=Wire();channel=Channel(scene,paths['B'][2]['end']);endpoints={c:Endpoint(c,paths[c],scene) for c in CARS}
    central=BicycleController(scene,paths,3,reference_warm_start=True) if arm=='C-new' else None
    rows=[];plans=[];records=[];timing=[];agreements=[];checkpoints=[];done={};failure=None;layer=None;started=time.perf_counter()
    original_actuals={c:np.array(scene['initial'][c]) for c in CARS}
    try:
      with contextlib.ExitStack() as guards:
        guards.enter_context(patch('longitudinal_joint.Controller.future_witness',side_effect=RuntimeError('central future forbidden online')))
        if arm=='L-new':
            for target in ('longitudinal_joint.solve','longitudinal_joint.build_qp','linearized_joint.subproblem','linearized_joint.BicycleController.predict'):
                guards.enter_context(patch(target,side_effect=RuntimeError('joint optimization forbidden in L-new')))
        for cycle in range(round(scene['timeout_s']/scene['h'])):
            before=time.perf_counter();wire.cycle=cycle;committed=False;stage='state_protocol';central_record=None;context=None;packet=None;planning_start=None;planning_end=None;cycle_failure=None;send_start=len(wire.log)
            try:
                for c,e in endpoints.items():
                    if central is not None:e.agent=central.agents[c]
                    if not e.requested and SIGN[c]*e.agent.z[0]>=-scene['request_rear_axle_abs_x']-1e-8:
                        request=wire.send(c,'channel','request',dict(car=c,state=e.agent.z.tolist(),cycle=cycle,permission_version=channel.version))
                        channel.request(request['car'],cycle);e.requested=True
                channel.grant_initial(cycle);active=channel.context(cycle)
                for c in CARS:wire.send('channel',c,'permission',active)
                working=copy.deepcopy(central) if central is not None else None
                if working is not None:working.prepare({c:active['owner']==c for c in CARS})
                reports=exchange_reports(endpoints,wire,active,working)
                for c,e in endpoints.items():
                    if e.work.mode=='DONE' and c not in done:
                        if not completed(e.work.z,scene['goals'][c],scene):raise ValueError('completed mode misses original goal')
                        done[c]=cycle*scene['h'];channel.event(cycle,'car_completed',car=c)
                if len(done)==2:break
                context=active
                if channel.owner and cleared(channel.owner,reports[channel.owner]['z'],scene) and (channel.owner=='B' or channel.waiting_ready(reports['B'])):
                    stage='release_prepare'
                    checkpoint=dict(cycle=cycle,state_time_s=active['state_time_s'],active=active,
                        offline_only=True,agents={c:e.private_checkpoint(central.reference_previous.get(c) if central else None) for c,e in endpoints.items()})
                    checkpoints.append(checkpoint)
                    release=wire.send(channel.owner,'channel','release_report',dict(car=channel.owner,authorization_version=channel.version,cycle=cycle))
                    context=channel.propose_release(release['car'],release['authorization_version'],reports,cycle)
                    for c in CARS:wire.send('channel',c,'prepare_conditional',context)
                    working=copy.deepcopy(central) if central is not None else None
                    if working is not None:working.prepare({c:context['owner']==c for c in CARS})
                    reports=exchange_reports(endpoints,wire,context,working)
                stage='initialization_or_shift'
                seeds={c:e.seed() for c,e in endpoints.items()}
                for c,e in endpoints.items():
                    other='B' if c=='A' else 'A';plans.append(seeds[c]);endpoints[other].observe_plan(wire.send(c,other,'seed',seeds[c]))
                for c,e in endpoints.items():e.validate_seed_pair(seeds[c])
                stage='candidate';planning_start=time.perf_counter()
                if central is not None:
                    commands,dense,central_record=working.predict({c:context['owner']==c for c in CARS})
                    if not central_record['accepted']:raise ValueError('central candidate rejected: '+central_record['message'])
                    for c,e in endpoints.items():e.plan=make_plan(e.report,commands[c],dense[c],'final',1,'central_joint')
                    for c,e in endpoints.items():
                        other='B' if c=='A' else 'A';plans.append(e.plan);endpoints[other].observe_plan(wire.send(c,other,'final',e.plan))
                else:
                    for c in CARS:
                        plan=endpoints[c].local_candidate();plans.append(plan);other='B' if c=='A' else 'A'
                        endpoints[other].observe_plan(wire.send(c,other,'final',plan))
                planning_end=time.perf_counter();stage='acknowledgement';barrier=Barrier(context)
                for e in endpoints.values():barrier.publish(e.plan)
                for c,e in endpoints.items():barrier.accept(wire.send(c,'channel','ack',e.ack()))
                token=barrier.token();effective=channel.commit_release(barrier) if context['conditional'] else channel.context(cycle)
                packet=dict(token=token,effective=effective,activated_proposal_id=context['proposal_id'] if context['conditional'] else None)
                delivered={c:wire.send('channel',c,'commit',packet) for c in CARS}
                for c,e in endpoints.items():e.check_commit(delivered[c])
                stage='commit';n=round(scene['h']/scene['sample_s'])
                motions={c:np.array(e.plan['states'])[:n+1] for c,e in endpoints.items()}
                rows.extend(rows_for(motions,{c:[e.plan['commands'][0]] for c,e in endpoints.items()},context['state_time_s'],scene,effective['owner']))
                if central is not None:
                    working.commit({c:e.plan['commands'] for c,e in endpoints.items()},{c:np.array(e.plan['states']) for c,e in endpoints.items()},central_record);central=working
                for c,e in endpoints.items():e.commit(delivered[c],central=central is not None)
                committed=True;agreements.append(dict(cycle=cycle,packet=packet,plans=token['plans'],acks=barrier.acks,committed=True))
            except (ValueError,RuntimeError,TimeoutError) as exc:
                cycle_failure=str(exc);failure=cycle_failure;layer=stage;channel.cancel(cycle,cycle_failure)
            finally:
                ended=time.perf_counter()
                records.append(dict(cycle=cycle,context=context,central=central_record,local={c:e.record for c,e in endpoints.items()} if central is None else None))
                sends=[r for r in wire.log[send_start:] if r['event']=='send']
                timing.append(dict(cycle=cycle,state_timestamp_s=round(scene['epoch_s']+cycle*scene['h'],10),start_monotonic_s=before,end_monotonic_s=ended,
                    total_cycle_wall_s=ended-before,planning_wall_s=(planning_end or ended)-(planning_start or before),committed=committed,
                    candidate_wall_s=(planning_end-planning_start) if planning_end is not None else None,
                    control_budget_remaining_s=scene['h']-(ended-before),coordination_rounds=COORDINATION_ROUNDS if planning_start is not None else 0,coordination_rounds_completed=COORDINATION_ROUNDS if committed else 0,
                    send_messages=len(sends),application_bytes=sum(s['application_bytes'] for s in sends),outcome='rejected' if cycle_failure else 'committed' if committed else 'completed',
                    failure_layer=stage if cycle_failure else None,failure=cycle_failure))
            if failure:break
        else:failure='simulation timeout';layer='timeout'
    finally:
        wire.close()
        if not rows:rows=rows_for({c:np.array([z]) for c,z in original_actuals.items()},{c:[] for c in CARS},scene['epoch_s'],scene)
        save_rows(directory/'actual.csv',rows)
        write_bundle(directory,dict(plans=plans,solves=records,agreements=agreements,checkpoints=checkpoints,events=channel.events,messages=wire.log))
        write(directory/'planning.json',timing);write(directory/'config.json',dict(scene=scene,arm=arm,contract=CONTRACT,max_coordination_rounds=COORDINATION_ROUNDS,order=['A','B'],coupling_policy='M1_joint_stage_axis' if arm=='C-new' else 'M2a-local-v4-nodewise-fixed-peer-state-box-certificate',local_reference_variables=scene['N'] if arm=='L-new' else None,local_bicycle_variables=2*scene['N'] if arm=='L-new' else None))
        summary=dict(arm=arm,both_completed=len(done)==2 and failure is None,T_complete_sim_s=max(done.values()) if len(done)==2 and failure is None else None,
            completion_per_car_s=done,failure=failure,failure_layer=layer,online_wall_s=time.perf_counter()-started,online_full_future_calls=0,
            online_joint_optimizer_forbidden=arm=='L-new',centralized_future_diagnostic='not yet executed; offline only')
        write(directory/'summary.json',summary)
    print(directory.name,json.dumps(summary),flush=True)
    return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--arm',choices=['C-new','L-new'],required=True);args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.exists():raise FileExistsError(args.output)
    scene=json.loads((ROOT/'config/entry_bicycle.json').read_text());validate_scene(scene);paths=routes(scene)
    source=args.output.parent/(args.output.name+'_source');source.mkdir()
    for p in Path(__file__).parent.glob('*.py'):shutil.copy2(p,source/p.name)
    write(source/'provenance.json',dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,platform=platform.platform(),threads=os.environ.get('OPENBLAS_NUM_THREADS'),
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.glob('*.py')},command='source /opt/ros/jazzy/setup.bash && PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID='+os.environ.get('ROS_DOMAIN_ID','0')+' ROS_LOG_DIR=/tmp/m2a_ros python3 '+' '.join(__import__('sys').argv),
        contract=CONTRACT,rounds=COORDINATION_ROUNDS,sequence=['A','B'],new_contract_not_equivalent_to_old=True))
    online(scene,paths,args.output,args.arm)


if __name__=='__main__':main()
