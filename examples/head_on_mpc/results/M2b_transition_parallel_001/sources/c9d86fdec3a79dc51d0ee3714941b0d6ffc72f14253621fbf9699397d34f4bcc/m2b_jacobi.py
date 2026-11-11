"""Own-only transactional actors and persistent process backends for sealed rounds."""
import copy
import hashlib
import multiprocessing as mp
from multiprocessing.connection import wait
import os
import resource
import time
import traceback
import numpy as np
from bicycle_core import completed
from m2a_logs import plan_id
from m2b_scene import Agent,CARS
from m2b_protocol import public_report,context_plan,check_plan,acknowledge,canonical
from m2b_control import Optimizer,commit_agent
from m2b_identity import Identity
from m2b_candidates import contract,compatible_reference,build_start,validate_start


def sha(value):return hashlib.sha256(canonical(value).encode()).hexdigest()
def base(context):return {k:v for k,v in context.items() if k!='coordination_round'}


class Actor:
    def __init__(self,role,route,scene,mapping=None):
        self.identity=Identity(mapping);self.role=role;self.car=self.identity.ids[role]
        self.scene=copy.deepcopy(scene)
        for k in ['initial','goals']:self.scene[k]={role:copy.deepcopy(scene[k][role])}
        self.agent=Agent(role,route,self.scene);self.optimizer=Optimizer();self.last=None;self.last_contract=None;self.last_cycle=-1
        self.cache={};self.cache_hits=0;self.cache_checks=0;self.pending=None;self.phase='committed'

    def report(self,context,work=False):
        agent=self.work if work else self.agent
        r=public_report(agent,self.identity.context(context,False))
        return {**r,'car':self.car,'context':copy.deepcopy(context)}

    def snapshot(self,context):return self.report(context)

    def prepare(self,context,peer_reports):
        if context['cycle']!=self.last_cycle+1:raise ValueError('wrong snapshot cycle')
        expected=set(self.identity.cars)-{self.car}
        if set(peer_reports)!=expected:raise ValueError('missing public snapshots')
        for c,r in peer_reports.items():
            if r['car']!=c or r['context']['cycle']!=context['cycle']:raise ValueError('stale/misrouted public report')
        self.work=copy.deepcopy(self.agent);self.working_optimizer=copy.deepcopy(self.optimizer)
        peers={self.identity.roles[c]:self.identity.report_to_role(r) for c,r in peer_reports.items()}
        self.work.advance(context['owner']==self.car,peers)
        if self.work.index>self.agent.index:
            endpoint=np.asarray(self.agent.segment['end']);z=self.work.z
            xy=float(np.linalg.norm(z[:2]-endpoint[:2]));yaw=abs(float(np.arctan2(np.sin(z[2]-endpoint[2]),np.cos(z[2]-endpoint[2]))))
            # These are the original terminal pose boxes, not a tightened physical requirement.
            if xy>.005 or yaw>.003:raise ValueError('next-stage guard misses original terminal pose box')
        self.context=copy.deepcopy(context);self.parameters=self.work.parameters(context['owner']==self.car)
        self.current_contract=contract(self.work,self.parameters,context['owner']==self.car)
        self.reference_changed=not compatible_reference(self.last_contract,self.current_contract)
        if self.reference_changed:self.working_optimizer.previous.pop(self.car,None)
        self.cache={};self.pending=None;self.record=None;self.plan=None;self.phase='prepared'
        return self.report(context,True)

    def start(self,round_index):
        if self.phase!='prepared' or round_index!=1:raise ValueError('invalid start generation phase')
        self.context={**self.context,'coordination_round':1};self.current_report=self.report(self.context,True)
        self.start_plan,self.start_record=build_start(self.work,self.parameters,self.current_report,self.context,self.identity,self.scene,self.last,self.last_contract,self.cache)
        self.seed_wall=self.start_record['build_wall_s'];self.inputs={self.car:copy.deepcopy(self.start_plan)};self.final_set={};self.phase='inputs'
        return dict(plan=self.start_plan,report=self.current_report,candidate_record=self.start_record)

    def _check_public(self,p):
        if p['car'] not in self.identity.cars:raise ValueError('unknown public car')
        if base(p['context'])!=base(self.context):raise ValueError('stale state/permit/proposal')
        normalized=self.identity.plan_to_role(p)
        before=len(self.cache);self.cache_checks+=1
        # Count actual local reuse explicitly, independently from dense timing probes.
        from m2b_protocol import VALIDATION_VERSION
        key=(normalized['id'],canonical(normalized['report']),canonical(normalized['context']),canonical(self.scene),VALIDATION_VERSION)
        self.cache_hits+=int(key in self.cache)
        check_plan(normalized,normalized['report'],normalized['context'],self.scene,self.cache)

    def deliver_inputs(self,envelopes):
        if self.phase!='inputs':raise ValueError('input set already sealed')
        for e in envelopes:
            p=e['payload']
            if e['receiver']!=self.car or e['sender']!=p['car'] or e['kind'] not in ('seed','final'):raise ValueError('wrong input envelope')
            if p['car']==self.car or p['car'] in self.inputs:raise ValueError('duplicate input member')
            expected=self.context['coordination_round']-(1 if p['kind']=='final' else 0)
            if p['context'].get('coordination_round')!=expected:raise ValueError('old/future input round')
            self._check_public(p);self.inputs[p['car']]=copy.deepcopy(p)
        return {'received':len(envelopes)}

    def solve(self):
        started=time.perf_counter();start_ns=time.monotonic_ns()
        if self.phase!='inputs' or set(self.inputs)!=set(self.identity.cars):raise ValueError('initialization has no complete compatible prediction set')
        self.sealed=copy.deepcopy(self.inputs);self.input_ids={c:p['id'] for c,p in self.sealed.items()}
        self.input_set_id=sha(dict(context=self.context,plans=self.input_ids))
        # Every endpoint independently checks all four sealed inputs before computing.
        for p in self.sealed.values():self._check_public(p)
        from m2b_scene import audit
        if not audit({c:np.asarray(p['states']) for c,p in self.sealed.items()},self.scene)['passed']:raise ValueError('sealed starting combination incompatible')
        pre=time.perf_counter()-started;remaining=.5-self.seed_wall-pre
        if remaining<=0:raise TimeoutError('candidate preparation consumed local budget')
        peers={c:p for c,p in self.sealed.items() if c!=self.car}
        u,z,record=self.working_optimizer.predict({self.car:self.work},{self.car:self.context['owner']==self.car},peers,self.scene,budget_s=remaining)
        self.record=record
        if not record['accepted']:raise ValueError(f'local:{record["failure_layer"]}:{record["message"]}')
        self.plan=context_plan(self.current_report,u[self.car],z[self.car],'final','local')
        self.plan.update(applicability=copy.deepcopy(self.current_contract),input_set_id=self.input_set_id)
        self.plan['id']=plan_id(self.plan)
        elapsed=self.seed_wall+time.perf_counter()-started
        if elapsed>.5:raise TimeoutError('combined candidate/solve local computation exceeds .5s')
        self.record.update(candidate_preparation_wall_s=self.seed_wall,local_computation_s=elapsed,round_input_ids=self.input_ids,
                           input_set_id=self.input_set_id,coordination_round=self.context['coordination_round'])
        self.final_set={self.car:copy.deepcopy(self.plan)};self.phase='solved'
        return dict(plan=self.plan,record=self.record,input_ids=self.input_ids,input_set_id=self.input_set_id,
                    car=self.car,pid=os.getpid(),compute_start_ns=start_ns,compute_end_ns=time.monotonic_ns())

    def deliver_finals(self,envelopes):
        if self.phase!='solved':raise ValueError('final received before own computation ended')
        for e in envelopes:
            p=e['payload']
            if e['receiver']!=self.car or e['sender']!=p['car'] or e['kind']!='final':raise ValueError('wrong final envelope')
            if p['car']==self.car or p['car'] in self.final_set:raise ValueError('duplicate final')
            if p['context']!=self.context or p.get('input_set_id')!=self.input_set_id:raise ValueError('mixed final round/input set')
            self._check_public(p);self.final_set[p['car']]=copy.deepcopy(p)
        return {'received':len(envelopes)}

    def ack(self):
        if self.phase!='solved' or set(self.final_set)!=set(self.identity.cars):raise ValueError('missing final combination')
        normalized={self.identity.roles[c]:self.identity.plan_to_role(p) for c,p in self.final_set.items()}
        reports={c:p['report'] for c,p in normalized.items()};ctx=self.identity.context(self.context,False)
        acknowledge(self.role,normalized,reports,ctx,self.scene,self.cache)
        self.acceptance=dict(car=self.car,context=copy.deepcopy(self.context),plans={c:p['id'] for c,p in self.final_set.items()},
                             accepted=True,input_set_id=self.input_set_id)
        self.phase='acknowledged';return self.acceptance

    def retry(self,round_index):
        if self.phase not in ('solved','acknowledged') or round_index!=self.context['coordination_round']+1:raise ValueError('invalid retry round')
        # No state advance, queue consumption or committed warm-start update occurs here.
        self.inputs={self.car:copy.deepcopy(self.plan)};self.context={**self.context,'coordination_round':round_index}
        self.current_report=self.report(self.context,True);self.phase='inputs';self.seed_wall=0.;self.record=None
        return {'ready':True}

    def stage_commit(self,packet):
        if self.phase!='acknowledged':raise ValueError('commit without own valid ACK')
        if packet['token']['context']!=self.context or packet['token']['plans']!=self.acceptance['plans']:raise ValueError('stale/mixed commit')
        effective=packet['effective']
        if any(effective[k]!=self.context[k] for k in ['cycle','owner','permission_version','released']):raise ValueError('commit before correct authorization')
        if self.context['conditional'] and packet['activated_proposal_id']!=self.context['proposal_id']:raise ValueError('proposal not activated')
        next_agent=copy.deepcopy(self.work);next_optimizer=copy.deepcopy(self.working_optimizer)
        next_optimizer.remember({self.car:self.work},self.record);commit_agent(next_agent,self.plan['commands'],self.plan['states'])
        self.pending=(next_agent,next_optimizer,copy.deepcopy(self.plan),copy.deepcopy(self.current_contract),self.context['cycle'])
        self.phase='staged'
        n=round(self.scene['h']/self.scene['sample_s'])
        return dict(car=self.car,states=self.plan['states'][:n+1],control=self.plan['commands'][0],report=self.current_report,
                    next_state=next_agent.z.tolist(),cache_hits=self.cache_hits,cache_checks=self.cache_checks)

    def activate(self):
        if self.phase!='staged' or self.pending is None:raise ValueError('activation without staged four-way decision')
        self.agent,self.optimizer,self.last,self.last_contract,self.last_cycle=self.pending
        self.pending=None;self.phase='committed';return {'activated':True}

    def abort(self):
        self.pending=None;self.phase='committed';self.cache={}
        return dict(actual_state=self.agent.z.tolist(),last_cycle=self.last_cycle,committed_warm_start=copy.deepcopy(self.optimizer.previous))


def worker(pipe,role,route,scene,mapping):
    from unittest.mock import patch
    with patch('m2b_control.predict_joint',side_effect=RuntimeError('central solver forbidden')),patch('longitudinal_joint.Controller.future_witness',side_effect=RuntimeError('online future forbidden')):
        actor=Actor(role,route,scene,mapping)
        pipe.send(dict(ok=True,pid=os.getpid(),role=role,affinity=sorted(os.sched_getaffinity(0)),threads=len(os.listdir('/proc/self/task'))))
        while True:
            try:request=pipe.recv()
            except EOFError:break
            if request['method']=='close':break
            try:
                result=getattr(actor,request['method'])(*request.get('args',[]))
                pipe.send(dict(ok=True,result=result,pid=os.getpid()))
            except Exception as e:
                pipe.send(dict(ok=False,error=type(e).__name__+': '+str(e),phase=actor.phase,car=actor.car,
                               record=getattr(actor,'record',None),state=actor.agent.z.tolist(),pid=os.getpid()))
        pipe.close()


class Workers:
    def __init__(self,scene,routes,mapping=None,parallel=False,start_order=None):
        self.identity=Identity(mapping);self.parallel=parallel;self.processes={};self.pipes={};self.serial=0;self.initial=[]
        ctx=mp.get_context('spawn');order=start_order or list(CARS)
        if set(order)!=set(CARS) or len(order)!=4:raise ValueError('invalid startup order')
        started=time.perf_counter()
        try:
            for role in order:
                parent,child=ctx.Pipe();own_scene=copy.deepcopy(scene)
                for key in ['initial','goals']:own_scene[key]={role:copy.deepcopy(scene[key][role])}
                process=ctx.Process(target=worker,args=(child,role,routes[role],own_scene,mapping));process.start();child.close()
                car=self.identity.ids[role];self.processes[car]=process;self.pipes[car]=parent
            for car in self.identity.cars:
                if not self.pipes[car].poll(20):raise TimeoutError('worker initialization timeout')
                self.initial.append(self.pipes[car].recv())
        except Exception:
            self.close();raise
        self.startup_wall_s=time.perf_counter()-started

    def call(self,method,args,parallel=None,timeout=10.):
        use_parallel=self.parallel if parallel is None else parallel
        pending={};responses={};deadline=time.monotonic()+timeout
        if set(args)!=set(self.identity.cars):raise ValueError('cohort call missing worker')
        for car in self.identity.cars:
            if not self.processes[car].is_alive():raise RuntimeError('worker is not alive '+car)
            self.pipes[car].send(dict(method=method,args=args[car]))
            if use_parallel:pending[self.pipes[car]]=car
            else:
                if not self.pipes[car].poll(max(0.,deadline-time.monotonic())):raise TimeoutError('serial worker timeout')
                responses[car]=self.pipes[car].recv()
        while pending:
            ready=wait(list(pending),timeout=max(0.,deadline-time.monotonic()))
            if not ready:raise TimeoutError('parallel cohort timeout')
            for pipe in ready:
                car=pending.pop(pipe);responses[car]=pipe.recv()
        return responses

    def close(self):
        for pipe in self.pipes.values():
            try:pipe.send(dict(method='close'))
            except (BrokenPipeError,EOFError,OSError):pass
        for process in self.processes.values():
            process.join(timeout=2)
            if process.is_alive():process.terminate();process.join(timeout=2)
        for pipe in self.pipes.values():pipe.close()
