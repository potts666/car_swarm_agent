#!/usr/bin/env python3
"""Bounded synchronous four-car C/L runs with five real ROS nodes and audited versions."""
import argparse
import contextlib
import copy
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import time
from unittest.mock import patch
import numpy as np
import scipy
import osqp
from bicycle_core import rollout,completed
from m2a_logs import lines
from m2b_scene import CARS,ORDER,SIGMA,Agent,ROOT,build_scene,build_routes,check_scenario,audit
from m2b_control import Optimizer,commit_agent
import m2b_control
from m2b_protocol import Channel,Wire,Barrier,public_report,context_plan,check_plan,acknowledge,cleared,canonical,CONTRACT
from m2b_recording import compact,diagnostics_record,export
from run_entry_bicycle import write


class Endpoint:
    def __init__(self,car,route,scene):
        self.car=car;self.scene=copy.deepcopy(scene);self.scene['initial']={car:scene['initial'][car]};self.scene['goals']={car:scene['goals'][car]};self.agent=Agent(car,route,self.scene);self.optimizer=Optimizer();self.last=None;self.last_commit=-1;self.peers={};self.reports={};self.record=None;self.plan=None
    def begin(self,context,peer_reports):
        self.context=copy.deepcopy(context);self.work=copy.deepcopy(self.agent);self.work.advance(context['owner']==self.car,peer_reports);self.report=public_report(self.work,context);self.peers={};self.reports={};self.record=None;self.plan=None
    def observe_report(self,r):
        if r['car']==self.car or r['car'] not in CARS or r['context']!=self.context:raise ValueError('stale peer state')
        if set(r)!={'car','z','mode','index','progress_m','queue_arrived','left_queue','steering_complete','context'}:raise ValueError('private/invalid peer state schema')
        self.reports[r['car']]=copy.deepcopy(r)
    def observe_plan(self,p):
        if p['car'] not in self.reports:raise ValueError('plan before public state')
        check_plan(p,self.reports[p['car']],self.context,self.scene);self.peers[p['car']]=copy.deepcopy(p)
    def seed(self):
        N=self.scene['N'];n=round(self.scene['h']/self.scene['sample_s'])
        if self.last is None:
            if abs(self.work.z[3])>1e-12:raise ValueError('moving initialization has no prediction')
            commands=[(0.,0.,'INITIAL_HOLD')]*N;source=None
        else:
            if np.max(abs(np.array(self.last['states'])[n]-self.work.z))>1e-12:raise ValueError('shift initial state mismatch')
            commands=self.last['commands'][1:]+[[0.,0.,'SHIFT_TAIL']];source=self.last['id']
        return context_plan(self.report,commands,rollout(self.work.z,commands,self.scene),'seed','own_shift',source)
    def candidate(self):
        if set(self.peers)!=set(CARS)-{self.car}:raise ValueError('missing three peer predictions')
        u,z,r=self.optimizer.predict({self.car:self.work},{self.car:self.context['owner']==self.car},self.peers,self.scene);self.record=r
        if not r['accepted']:raise ValueError(f'{self.car}:{r["failure_layer"]}:{r["message"]}')
        self.plan=context_plan(self.report,u[self.car],z[self.car],'final','local');return self.plan
    def ack(self):return acknowledge(self.car,{**self.peers,self.car:self.plan},{**self.reports,self.car:self.report},self.context,self.scene)
    def check_commit(self,packet):
        token=packet['token'];effective=packet['effective']
        if token['context']!=self.context or token['plans']!={**{c:p['id'] for c,p in self.peers.items()},self.car:self.plan['id']}:raise ValueError('mixed final versions at commit')
        if self.context['cycle']<=self.last_commit or any(effective[k]!=self.context[k] for k in ('cycle','owner','permission_version','released')):raise ValueError('commit before effective authorization')
        if self.context['conditional'] and packet['activated_proposal_id']!=self.context['proposal_id']:raise ValueError('conditional proposal not activated')
    def commit(self,packet,central):
        self.check_commit(packet)
        if not central:self.optimizer.remember({self.car:self.work},self.record)
        commit_agent(self.work,self.plan['commands'],self.plan['states']);self.agent=self.work;self.last=self.plan;self.last_commit=self.context['cycle']


def broadcast_reports(endpoints,wire,context,initial_reports):
    for e in endpoints.values():e.begin(context,initial_reports[e.car])
    reports={c:wire.send(c,'channel','state',e.report) for c,e in endpoints.items()}
    for c,e in endpoints.items():
        for other in CARS:
            if other!=c:endpoints[other].observe_report(wire.send(c,other,'state',e.report))
    return reports


def online(scene,routes,directory,arm,policy):
    directory.mkdir();wire=Wire();channel=Channel(scene,policy);endpoints={c:Endpoint(c,routes[c],scene) for c in CARS};central=Optimizer() if arm=='central' else None
    logs={k:[] for k in ('actual','plans','solves','agreements','events','messages','planning')};done={};failure=None;failure_layer=None;started=time.perf_counter();diagnostics={};recent=[];retain_through=-1;last_reports={c:{} for c in CARS}
    try:
      with contextlib.ExitStack() as guards:
        guards.enter_context(patch('longitudinal_joint.Controller.future_witness',side_effect=RuntimeError('online complete future forbidden')))
        if arm=='local':
            for target in ('m2b_control.predict_joint','longitudinal_joint.build_qp','longitudinal_joint.solve','linearized_joint.subproblem','linearized_joint.BicycleController.predict'):
                guards.enter_context(patch(target,side_effect=RuntimeError('central optimizer forbidden in local run')))
        for cycle in range(round(scene['timeout_s']/scene['h'])):
            begin=time.perf_counter();wire.cycle=cycle;stage='state';committed=False;central_record=None;record=None;context=None;cycle_failure=None;candidate_start=None;candidate_end=None;send_start=len(wire.log);conditional=False
            try:
                if begin-started>scene['wall_watchdog_s']:raise TimeoutError('predeclared wall watchdog')
                active=channel.context(cycle)
                # Current states are first delivered as public messages; local stages read no other Agent.
                initial={c:public_report(e.agent,active) for c,e in endpoints.items()};inboxes={c:{} for c in CARS}
                for c,r in initial.items():
                    if c not in channel.requests and c not in channel.served and SIGMA[c]*r['z'][0]>=-34-1e-8:
                        req=wire.send(c,'channel','request',dict(car=c,cycle=cycle,z=r['z'],permission_version=channel.version));channel.request(req['car'],cycle)
                    for other in CARS:
                        if other!=c:inboxes[other][c]=wire.send(c,other,'snapshot',r)
                for c in CARS:wire.send('channel',c,'permission',active)
                reports=broadcast_reports(endpoints,wire,active,inboxes)
                for c,e in endpoints.items():
                    if e.work.mode=='DONE' and c not in done:
                        if not completed(e.work.z,scene['goals'][c],scene):raise ValueError('DONE misses goal '+c)
                        done[c]=cycle*scene['h'];channel.event(cycle,'car_completed',car=c)
                if len(done)==4:break
                context=active
                if channel.owner is None or cleared(channel.owner,reports[channel.owner]['z'],scene):
                    stage='release_prepare'
                    reporter=channel.owner
                    if reporter:
                        r=wire.send(reporter,'channel','release_report',dict(car=reporter,authorization_version=channel.version,cycle=cycle));reporter=r['car']
                    context=channel.prepare(reporter,channel.version,reports,cycle);conditional=context['conditional']
                    if conditional:
                        for c in CARS:wire.send('channel',c,'prepare_conditional',context)
                        reports=broadcast_reports(endpoints,wire,context,{c:{p:r for p,r in reports.items() if p!=c} for c in CARS})
                stage='initialization';seeds={c:e.seed() for c,e in endpoints.items()}
                for c,p in seeds.items():
                    logs['plans'].append(p)
                    for other in CARS:
                        if other!=c:endpoints[other].observe_plan(wire.send(c,other,'seed',p))
                if not audit({c:np.array(p['states']) for c,p in seeds.items()},scene)['passed']:raise ValueError('no compatible shifted/initial prediction set')
                stage='candidate';candidate_start=time.perf_counter()
                if central is not None:
                    u,z,central_record=m2b_control.predict_joint(central,{c:e.work for c,e in endpoints.items()},{c:context['owner']==c for c in CARS},scene)
                    if not central_record['accepted']:raise ValueError('central:'+str(central_record['failure_layer'])+':'+central_record['message'])
                    for c,e in endpoints.items():e.plan=context_plan(e.report,u[c],z[c],'final','central_joint')
                    sequence=CARS
                else:sequence=ORDER
                for c in sequence:
                    p=endpoints[c].plan if central else endpoints[c].candidate();logs['plans'].append(p)
                    for other in CARS:
                        if other!=c:endpoints[other].observe_plan(wire.send(c,other,'final',p))
                candidate_end=time.perf_counter();stage='acknowledgement';barrier=Barrier(context)
                for e in endpoints.values():barrier.publish(e.plan)
                for c,e in endpoints.items():barrier.accept(wire.send(c,'channel','ack',e.ack()))
                token=barrier.token();effective=channel.commit(barrier) if conditional else channel.context(cycle)
                packet=dict(token=token,effective=effective,activated_proposal_id=context['proposal_id'] if conditional else None)
                delivered={c:wire.send('channel',c,'commit',packet) for c in CARS}
                for c,e in endpoints.items():e.check_commit(delivered[c])
                stage='commit';n=round(scene['h']/scene['sample_s'])
                logs['actual'].append(dict(cycle=cycle,owner=effective['owner'],permission_version=effective['permission_version'],plans=token['plans'],states={c:e.plan['states'][:n+1] for c,e in endpoints.items()},controls={c:e.plan['commands'][0] for c,e in endpoints.items()},reports={c:e.report for c,e in endpoints.items()}))
                if central:central.remember({c:e.work for c,e in endpoints.items()},central_record)
                for c,e in endpoints.items():e.commit(delivered[c],central is not None)
                logs['agreements'].append(dict(cycle=cycle,packet=packet,plans=token['plans'],acks=barrier.acks,committed=True));committed=True
            except Exception as exc:
                cycle_failure=type(exc).__name__+': '+str(exc);failure=cycle_failure;failure_layer=stage;channel.cancel(cycle,cycle_failure)
            finally:
                ended=time.perf_counter();record=dict(cycle=cycle,context=context,central=central_record,local={c:e.record for c,e in endpoints.items()} if central is None else None)
                logs['solves'].append(compact(record));sends=[r for r in wire.log[send_start:] if r['event']=='send']
                logs['planning'].append(dict(cycle=cycle,state_timestamp_s=round(scene['epoch_s']+cycle*scene['h'],10),start_monotonic_s=begin,end_monotonic_s=ended,total_cycle_wall_s=ended-begin,candidate_wall_s=candidate_end-candidate_start if candidate_start is not None and candidate_end is not None else None,committed=committed,outcome='failed' if cycle_failure else 'committed' if committed else 'completed',failure_layer=failure_layer if cycle_failure else None,failure=cycle_failure,application_bytes=sum(x['application_bytes'] for x in sends),send_messages=len(sends),coordination_rounds=1 if candidate_start is not None else 0,control_budget_remaining_s=scene['h']-(ended-begin)))
                recent.append(record);recent=recent[-3:]
                records=[central_record] if central_record else [e.record for e in endpoints.values() if e.record]
                repaired=any(q.get('stop_input_adjustments') for r in records for q in r['subproblems'])
                if cycle_failure or conditional or repaired:
                    retain_through=cycle+1
                    for r in recent:diagnostics[r['cycle']]=diagnostics_record(r)
                elif cycle<=retain_through:diagnostics[cycle]=diagnostics_record(record)
            if failure:break
            if cycle%100==0:print(directory.name,'cycle',cycle,'sim_s',round(cycle*scene['h'],1),'owner',channel.owner,'done',list(done),flush=True)
        else:failure='simulation timeout';failure_layer='timeout'
    finally:
        wire.close();online_end=time.perf_counter();logs['events']=channel.events;logs['messages']=wire.log
        write(directory/'config.json',dict(scene=scene,arm=arm,policy=policy,contract=CONTRACT,coordination_order=list(ORDER),max_coordination_rounds=1,prediction_budget_scope='one local vehicle call; central one four-vehicle call',budget_s=.5,online_joint_forbidden=arm=='local',online_future_calls=0,source_version=scene.get('source_version')))
        summary=dict(arm=arm,policy=policy,all_completed=len(done)==4 and failure is None,completion_sim_s=max(done.values()) if len(done)==4 and failure is None else None,completion_per_car_s=done,failure=failure,failure_layer=failure_layer,online_wall_s=online_end-started,export_wall_s=time.perf_counter()-online_end,committed_cycles=len(logs['actual']),timed_cycles=len(logs['planning']),last_states={c:e.agent.z.tolist() for c,e in endpoints.items()},old_central_future_diagnostic='not executed; no online future gate')
        write(directory/'summary.json',summary)
        export(directory,logs,list(diagnostics.values()))
        summary['export_wall_s']=time.perf_counter()-online_end;write(directory/'summary.json',summary)
    print(directory.name,json.dumps(summary),flush=True);return summary


def snapshot(root):
    names=['m2b_scene.py','m2b_control.py','m2b_protocol.py','m2b_recording.py','run_m2b.py','verify_m2b.py','report_m2b.py','m2a_logs.py','m2a_local.py','m2a_protocol.py','bicycle_core.py','linearized_joint.py','longitudinal_joint.py','verify_entry_bicycle.py','run_entry_bicycle.py','recovery_core.py']
    hashes={n:hashlib.sha256((Path(__file__).parent/n).read_bytes()).hexdigest() for n in names if (Path(__file__).parent/n).exists()};version=hashlib.sha256(canonical(hashes).encode()).hexdigest();out=root/'sources'/version
    if not out.exists():
        out.mkdir(parents=True)
        for n in hashes:shutil.copy2(Path(__file__).parent/n,out/n)
        write(out/'manifest.json',hashes)
    return version


def main():
    p=argparse.ArgumentParser();p.add_argument('--check-scene',action='store_true');p.add_argument('--arm',choices=['central','local']);p.add_argument('--policy',choices=['fifo-ready','alternating-ready']);p.add_argument('--repeat',type=int,default=1);p.add_argument('--output',type=Path);a=p.parse_args();scene=build_scene();routes=build_routes(scene);gate=check_scenario(scene,routes)
    if a.check_scene:print(json.dumps(gate,indent=2));return
    if not gate['passed']:raise ValueError('scene gate failed')
    if a.output is None or a.arm is None or a.policy is None:p.error('arm/policy/output required')
    a.output.mkdir(parents=True,exist_ok=False);root=a.output.parent;scene['source_version']=snapshot(root)
    write(a.output/'environment.json',dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,platform=platform.platform(),threads=os.environ.get('OPENBLAS_NUM_THREADS'),ros_domain=os.environ.get('ROS_DOMAIN_ID'),command=sys.argv,source_version=scene['source_version']))
    write(a.output/'scene_gate.json',gate);write(a.output/'routes.json',routes)
    for i in range(a.repeat):online(scene,routes,a.output/f'r{i+1}',a.arm,a.policy)

if __name__=='__main__':main()
