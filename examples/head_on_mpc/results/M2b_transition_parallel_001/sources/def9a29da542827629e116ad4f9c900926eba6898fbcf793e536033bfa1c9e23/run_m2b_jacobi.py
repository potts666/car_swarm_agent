#!/usr/bin/env python3
"""Bounded sealed-input experiments. Parent coordinates resources, never motion."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import time
import numpy as np
from bicycle_core import completed
from m2b_scene import CARS,SIGMA,build_scene,build_routes,check_scenario
from m2b_protocol import Channel,Barrier,cleared,CONTRACT,canonical
from m2b_identity import Identity
from m2b_jacobi import Workers,base
from m2b_round_wire import RoundWire
from m2b_recording import compact,diagnostics_record,export
from run_entry_bicycle import write


def snapshot(root):
    scripts=Path(__file__).parent
    names=[p for p in scripts.glob('*.py') if p.name.startswith(('m2b_','m2a_')) or p.name in ('run_m2b_jacobi.py','verify_m2b.py','verify_m2b_jacobi.py','bicycle_core.py','longitudinal_joint.py','linearized_joint.py','verify_entry_bicycle.py','run_entry_bicycle.py','recovery_core.py')]
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in names};version=hashlib.sha256(canonical(hashes).encode()).hexdigest();out=root/'sources'/version
    if not out.exists():
        out.mkdir(parents=True)
        for p in names:shutil.copy2(p,out/p.name)
        tests=out/'tests';tests.mkdir()
        for p in scripts.parent.joinpath('tests').glob('test_m2b*.py'):shutil.copy2(p,tests/p.name)
        write(out/'manifest.json',hashes)
    return version


def online(scene,routes,directory,backend='S-J',mapping=None,start_order=None,max_cycles=None,max_rounds=1):
    directory.mkdir(parents=True,exist_ok=False);identity=Identity(mapping);cars=identity.cars
    channel=Channel(scene,'fifo-ready');logs={k:[] for k in ('actual','plans','solves','agreements','events','messages','planning','rounds','candidates')}
    workers=None;wire=None;done={};failure=None;failure_layer=None;last_states={identity.ids[c]:scene['initial'][c] for c in CARS};diagnostics=[]
    started=time.perf_counter();startup={};online_start=None;shutdown_wall=0.
    def rpc(method,args,parallel=None):
        call_start=time.perf_counter();response=workers.call(method,args,parallel)
        rpc_times[method]=rpc_times.get(method,0.)+time.perf_counter()-call_start
        bad={c:r for c,r in response.items() if not r['ok']}
        if bad:
            diagnostics.append(diagnostics_record(dict(cycle=cycle,method=method,responses=response)))
            raise ValueError(method+': '+canonical({c:r['error'] for c,r in bad.items()}))
        return {c:r['result'] for c,r in response.items()}
    def publish(reports,kind,include_channel=False):
        batch=[(c,d,kind,r) for c,r in reports.items() for d in (*cars,'channel') if d!=c and (d!='channel' or include_channel)]
        received=wire.exchange(batch)
        return {c:{e['sender']:e['payload'] for e in received if e['receiver']==c} for c in (*cars,'channel')}
    try:
        workers=Workers(scene,routes,mapping,backend=='P-J',start_order);startup['workers']=workers.initial;startup['worker_startup_wall_s']=workers.startup_wall_s
        dds=time.perf_counter();wire=RoundWire(cars);startup['dds_discovery_wall_s']=time.perf_counter()-dds
        online_start=time.perf_counter();startup['total_startup_wall_s']=online_start-started
        for cycle in range(round(scene['timeout_s']/scene['h'])):
            begin=time.perf_counter();wire.cycle=cycle;stage='snapshot';committed=False;context=None;local={};round_count=0;cycle_failure=None;send_start=len(wire.log);candidate_begin=None;executable=None;rpc_times={}
            try:
                if max_cycles is not None and cycle>=max_cycles:raise RuntimeError('bounded prefix limit reached')
                if begin-started>scene['wall_watchdog_s']:raise TimeoutError('predeclared wall watchdog including startup')
                active=identity.context(channel.context(cycle));initial=rpc('snapshot',{c:[active] for c in cars},True)
                requests=[]
                for c,r in initial.items():
                    role=identity.roles[c]
                    if role not in channel.requests and role not in channel.served and SIGMA[role]*r['z'][0]>=-34-1e-8:
                        requests.append((c,'channel','request',dict(car=c,cycle=cycle,z=r['z'],permission_version=channel.version)))
                for e in wire.exchange(requests):channel.request(identity.roles[e['payload']['car']],cycle)
                inbox=publish(initial,'snapshot')
                wire.exchange([('channel',c,'permission',active) for c in cars])
                reports=rpc('prepare',{c:[active,inbox[c]] for c in cars},True);delivered=publish(reports,'state',True);reports=delivered['channel']
                for c,r in reports.items():
                    if r['mode']=='DONE' and c not in done:
                        if not completed(r['z'],scene['goals'][identity.roles[c]],scene):raise ValueError('DONE misses goal '+c)
                        done[c]=cycle*scene['h'];channel.event(cycle,'car_completed',car=identity.roles[c])
                if len(done)==4:break
                context=active
                if channel.owner is None or cleared(channel.owner,reports[identity.ids[channel.owner]]['z'],scene):
                    stage='release_prepare';reporter=channel.owner
                    if reporter:
                        msg=wire.exchange([(identity.ids[reporter],'channel','release_report',dict(car=identity.ids[reporter],authorization_version=channel.version,cycle=cycle))])[0]
                        reporter=identity.roles[msg['payload']['car']]
                    context=identity.context(channel.prepare(reporter,channel.version,{identity.roles[c]:identity.report_to_role(r) for c,r in reports.items()},cycle))
                    if context['conditional']:
                        wire.exchange([('channel',c,'prepare_conditional',context) for c in cars])
                        reports=rpc('prepare',{c:[context,{p:r for p,r in reports.items() if p!=c}] for c in cars},True)
                        publish(reports,'state',True)
                stage='candidate_initialization';candidate_begin=time.perf_counter()
                seeds=rpc('start',{c:[1] for c in cars})
                for c,r in seeds.items():logs['plans'].append(r['plan']);logs['candidates'].append(compact(dict(cycle=cycle,car=c,**r['candidate_record'])))
                # Round-tagged reports and seeds travel over DDS before inputs are sealed.
                publish({c:r['report'] for c,r in seeds.items()},'state',True)
                incoming=wire.exchange([(c,d,'seed',r['plan']) for c,r in seeds.items() for d in cars if d!=c])
                rpc('deliver_inputs',{c:[[e for e in incoming if e['receiver']==c]] for c in cars},True)
                for round_index in range(1,max_rounds+1):
                    round_count=round_index;stage='local_solve';computed=rpc('solve',{c:[] for c in cars})
                    local={c:r['record'] for c,r in computed.items()}
                    ids={r['input_set_id'] for r in computed.values()}
                    if len(ids)!=1 or any(r['input_ids']!=next(iter(computed.values()))['input_ids'] for r in computed.values()):raise ValueError('workers used different sealed input sets')
                    for r in computed.values():logs['plans'].append(r['plan'])
                    round_ctx=next(iter(computed.values()))['plan']['context']
                    for q in local.values():q['round_context']=round_ctx
                    logs['rounds'].append(compact(dict(cycle=cycle,coordination_round=round_index,context=round_ctx,input_set_id=next(iter(ids)),input_ids=next(iter(computed.values()))['input_ids'],workers={c:{k:v for k,v in r.items() if k not in ('plan','record','input_ids')} for c,r in computed.items()})))
                    stage='final_exchange';incoming=wire.exchange([(c,d,'final',r['plan']) for c,r in computed.items() for d in cars if d!=c])
                    rpc('deliver_finals',{c:[[e for e in incoming if e['receiver']==c]] for c in cars},True)
                    stage='compatibility_ack'
                    try:acks=rpc('ack',{c:[] for c in cars},True);break
                    except ValueError:
                        if round_index==max_rounds:raise
                        rpc('retry',{c:[round_index+1] for c in cars},True)
                        rpc('deliver_inputs',{c:[[e for e in incoming if e['receiver']==c]] for c in cars},True)
                stage='commit_barrier';pair={c:r['plan']['id'] for c,r in computed.items()};ack_msgs=wire.exchange([(c,'channel','ack',a) for c,a in acks.items()]);acks={e['sender']:e['payload'] for e in ack_msgs}
                barrier=Barrier(identity.context(context,False))
                for c,r in computed.items():barrier.publish(dict(car=identity.roles[c],context=barrier.context,kind='final',id=r['plan']['id']))
                for c,a in acks.items():
                    if a['context']!=round_ctx or a['plans']!=pair or a['input_set_id']!=next(iter(ids)):raise ValueError('mixed round acknowledgement')
                    barrier.accept(dict(car=identity.roles[c],context=barrier.context,plans={identity.roles[k]:v for k,v in pair.items()},accepted=a['accepted']))
                barrier.token();effective={**base(round_ctx),'proposal_id':None,'conditional':False}
                packet=dict(token=dict(context=round_ctx,plans=pair),effective=effective,activated_proposal_id=context['proposal_id'] if context['conditional'] else None)
                staged=rpc('stage_commit',{c:[packet] for c in cars},True)
                # Frozen offline simulation: all prefixes are staged before the one execution decision.
                wire.exchange([('channel',c,'commit',packet) for c in cars]);executable=time.perf_counter()
                if context['conditional']:channel.commit(barrier)
                logs['actual'].append(dict(cycle=cycle,owner=effective['owner'],permission_version=effective['permission_version'],plans=pair,states={c:r['states'] for c,r in staged.items()},controls={c:r['control'] for c,r in staged.items()},reports={c:r['report'] for c,r in staged.items()}))
                logs['agreements'].append(dict(cycle=cycle,packet=packet,plans=pair,acks=acks,committed=True));committed=True
                last_states={c:r['next_state'] for c,r in staged.items()}
                rpc('activate',{c:[] for c in cars},True)
            except Exception as exc:
                cycle_failure=type(exc).__name__+': '+str(exc);failure=cycle_failure;failure_layer='prefix_limit' if 'bounded prefix' in str(exc) else stage
                channel.cancel(cycle,cycle_failure)
                try:rpc('abort',{c:[] for c in cars},True)
                except Exception as abort_error:diagnostics.append(dict(cycle=cycle,abort_error=str(abort_error)))
            finally:
                record=dict(cycle=cycle,context=next(iter(local.values())).get('round_context',context) if local else context,central=None,local=local)
                logs['solves'].append(compact(record));sends=[r for r in wire.log[send_start:] if r['event']=='send'];ended=time.perf_counter()
                logs['planning'].append(dict(cycle=cycle,state_timestamp_s=round(scene['epoch_s']+cycle*scene['h'],10),start_monotonic_s=begin,end_monotonic_s=ended,total_cycle_wall_s=ended-begin,full_cycle_wall_s=ended-begin,candidate_wall_s=ended-candidate_begin if candidate_begin else None,committed=committed,outcome='failed' if cycle_failure else 'committed' if committed else 'completed',failure_layer=failure_layer if cycle_failure else None,failure=cycle_failure,application_bytes=sum(r['application_bytes'] for r in sends),send_messages=len(sends),coordination_rounds=round_count,control_budget_remaining_s=scene['h']-(ended-begin),control_executable_wall_s=executable-begin if executable else None,rpc_wall_s=rpc_times))
                if cycle_failure or (context and context['conditional']):diagnostics.append(diagnostics_record(record))
            if failure:break
            if cycle%100==0:print(directory.name,'cycle',cycle,'owner',channel.owner,'done',list(done),flush=True)
        else:failure='simulation timeout';failure_layer='timeout'
    except Exception as exc:
        failure=type(exc).__name__+': '+str(exc);failure_layer='startup'
    finally:
        online_end=time.perf_counter();closing=time.perf_counter()
        if workers:workers.close()
        if wire:wire.close();logs['messages']=wire.log
        shutdown_wall=time.perf_counter()-closing
        logs['events']=[identity.event(e) for e in channel.events]
        write(directory/'config.json',dict(scene=identity.scene(scene),arm='local',policy='fifo-ready',contract=CONTRACT,backend=backend,coordination_scheme='sealed-jacobi-v1',identity_mapping=identity.ids,start_order=start_order or list(CARS),coordination_order=list(identity.order),max_coordination_rounds=max_rounds,budget_s=.5,local_budget_scope='start candidate + own prevalidation + own optimizer; IPC/DDS separately in whole cycle',online_joint_forbidden=True,online_future_calls=0,source_version=scene.get('source_version'),validation_cache=True,detailed_timing=False,max_cycles=max_cycles))
        write(directory/'startup.json',startup)
        summary=dict(backend=backend,all_completed=len(done)==4 and failure is None,completion_sim_s=max(done.values()) if len(done)==4 and failure is None else None,completion_per_car_s=done,failure=failure,failure_layer=failure_layer,online_wall_s=online_end-(online_start or started),total_wall_before_export_s=time.perf_counter()-started,shutdown_wall_s=shutdown_wall,committed_cycles=len(logs['actual']),timed_cycles=len(logs['planning']),last_states=last_states,old_central_future_diagnostic='not executed; no online future gate')
        write(directory/'summary.json',summary);export_started=time.perf_counter();export(directory,logs,diagnostics);summary['export_wall_s']=time.perf_counter()-export_started;write(directory/'summary.json',summary)
    print(directory.name,canonical(summary),flush=True);return summary


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--backend',choices=['S-J','P-J'],required=True);p.add_argument('--max-cycles',type=int);p.add_argument('--max-rounds',type=int,default=1);p.add_argument('--variant',choices=['main','rename','position'],default='main');a=p.parse_args()
    scene=build_scene();mapping=None;order=None
    if a.variant=='rename':mapping=dict(A1='west_head',A2='west_tail',B1='east_head',B2='east_tail');order=list(reversed(CARS))
    if a.variant=='position':
        for c in CARS:scene['initial'][c][0]-=SIGMA[c]*.1
    routes=build_routes(scene);gate=check_scenario(scene,routes)
    if not gate['passed']:raise ValueError('reachability/physical scene gate failed')
    a.output.mkdir(parents=True,exist_ok=False);scene['source_version']=snapshot(a.output.parent)
    write(a.output/'routes.json',{Identity(mapping).ids[c]:r for c,r in routes.items()});write(a.output/'scene_gate.json',gate)
    write(a.output/'environment.json',dict(python=platform.python_version(),platform=platform.platform(),affinity=sorted(os.sched_getaffinity(0)),blas_threads=os.environ.get('OPENBLAS_NUM_THREADS'),ros_domain=os.environ.get('ROS_DOMAIN_ID'),source_version=scene['source_version']))
    online(scene,routes,a.output/'r1',a.backend,mapping,order,a.max_cycles,a.max_rounds)


if __name__=='__main__':main()
