#!/usr/bin/env python3
"""Prepare three development-only cases; reuse the supplied geometry/certifier."""
import argparse
from collections import defaultdict
import copy
from fractions import Fraction as F
import hashlib
import heapq
import importlib.util
import json
from pathlib import Path
import shutil
import time

from cross_stop import propose,verify


def encode(value):return json.dumps(value,default=str,ensure_ascii=False,sort_keys=True,separators=(',',':'))+'\n'
def write(path,value):path.write_text(encode(value))


def costs(graph,choices):
    reverse=defaultdict(list);out=defaultdict(list)
    for e in choices:
        reverse[e['target']].append(e);out[e['source']].append(e)
    values={graph['goal']:0};queue=[(0,graph['goal'])]
    while queue:
        v,node=heapq.heappop(queue)
        if values[node]!=v:continue
        for e in reverse[node]:
            new=v+e['ticks']
            if new<values.get(e['source'],float('inf')):
                values[e['source']]=new;heapq.heappush(queue,(new,e['source']))
    return values,out


def prepare(rebuild,out):
    work=(rebuild/'work').resolve()
    verified=json.loads((rebuild/'input_verification.json').read_text())
    assert json.loads((rebuild/'summary.json').read_text())['status']=='passed'
    for name,sha in verified['files'].items():
        if name.endswith('.py') or name.endswith('m2b_fourcar.json'):
            assert hashlib.sha256((work/name).read_bytes()).hexdigest()==sha
    spec=importlib.util.spec_from_file_location('pilot_geometry',work/'NTU_DMPC_Dissertation/certificate_graph.py')
    lib=importlib.util.module_from_spec(spec);spec.loader.exec_module(lib)
    original=copy.deepcopy(lib.CFG)
    definitions=[('asymmetric_baseline','current asymmetric start/goal map'),
                 ('tight_curve','bay and goal |y|=3.15 m; same road and vehicle'),
                 ('exit_merge_dependency','followers A2/B2 approach .5 m closer; resource dependencies retained')]
    summary=[]
    (out/'objects').mkdir()
    for case_id,description in definitions:
        began=time.perf_counter();cfg=copy.deepcopy(original)
        if case_id=='tight_curve':
            for field in ('initial','goals'):
                for car,z in cfg[field].items():z[1]=F(63,20) if z[1]>0 else F(-63,20)
        if case_id=='exit_merge_dependency':
            cfg['initial']['A2'][0]+=F(1,2);cfg['initial']['B2'][0]-=F(1,2)
        lib.CFG=cfg
        lib.YBAYS=sorted(set(z[1] for z in cfg['goals'].values()),reverse=True)
        # Larger but still certified initial domain leaves room for robust
        # runtime membership checks. This does not change the physical initial state.
        lib.STAGE_RADII=[(F(1,1000),F(1,100000)),*lib.STAGE_RADII[1:]]
        case=out/case_id;case.mkdir()
        write(case/'config.json',cfg)
        write(case/'development_definition.json',dict(id=case_id,description=description,
              stage_radii=lib.STAGE_RADII,formal_holdout=False,
              source_domain_change='initial P=.001,T=.00001; recertified, not a physical-limit change'))
        graphs={};maps={};lookup={};all_options=[]
        for stopped,label in ((True,'S'),(False,'M')):
            compiled={};mapping={}
            for car in lib.CARS:
                for source in range(3):
                    for target in ((1,2,3) if source==0 else (3,)):
                        key=f'{car}-{source}-{target}'
                        print(case_id,label,key,flush=True)
                        knobs,joins,points=propose(lib,car,source,target,stopped)
                        cert=verify(lib,car,source,target,knobs)
                        cert.update(joins=joins,waypoints=[list(map(str,p)) for p in points],
                            stage_radii=[list(map(str,p)) for p in lib.STAGE_RADII],
                            scale=lib.Q,model_source_manifest=verified['manifest_sha256'])
                        blob=encode(cert);sha=hashlib.sha256(blob.encode()).hexdigest()
                        path=out/'objects'/(sha+'.json')
                        if path.exists():assert path.read_text()==blob
                        else:path.write_text(blob)
                        mapping[key]=sha;lookup[sha]=cert;compiled[key]=cert
            graph=lib.reachable_graph(compiled);graphs[label]=graph;maps[label]=mapping
            write(case/(label+'_graph.json'),graph)
            write(case/(label+'_motions.json'),mapping)
        # Both methods start with the same feasible S strategy. M also retains
        # every S reference as a legal fallback; no oracle-optimal seed is required.
        shared_nodes=set(graphs['S']['goal_levels'])
        for label in ('S','M'):
            for edge in graphs[label]['winning_edges']:
                if edge['source'] in shared_nodes and edge['target'] in shared_nodes:
                    sha=maps[label][edge['motion']]
                    all_options.append({**edge,'kind':label,'object':sha,'ticks':lookup[sha]['ticks']})
        C_S,outgoing=costs(graphs['S'],[e for e in all_options if e['kind']=='S'])
        C_M,_=costs(graphs['S'],all_options)
        assert shared_nodes==set(C_S)==set(C_M)
        rank=lambda s:sum(2 if x=='0' else 1 if x in '12' else 0 for x in s)
        order=sorted(shared_nodes,key=lambda s:(rank(s),s))
        policy={s:min(edges,key=lambda e:(e['ticks'],e['actor'],e['target'],e['object']))
                for s,edges in outgoing.items() if edges}
        U={graphs['S']['goal']:0}
        for s in order:
            if s!=graphs['S']['goal']:
                edge=policy[s];U[s]=edge['ticks']+U[edge['target']]
        assert all(U[s]>=C_S[s] for s in shared_nodes)
        write(case/'coordination.json',dict(initial=graphs['S']['initial'],goal=graphs['S']['goal'],
            options=all_options,initial_policy=policy,initial_U=U,C_S=C_S,C_M=C_M,topological_order=order,
            input_scope='same certified geometry family; S imposes stops only at physical waypoints',
            permission='one certified moving actor; same rule for all five development methods',
            initial_strategy_rule='shortest current S connection; actor/target/content hash tie order',
            algorithm_frozen=False,formal_batch_enabled=False))
        row=dict(case=case_id,preparation_wall_s=time.perf_counter()-began,
            initial_strategy_ticks=U[graphs['S']['initial']],
            initial_C_S=C_S[graphs['S']['initial']],initial_C_M=C_M[graphs['S']['initial']],
            initial_gap_to_S_library=U[graphs['S']['initial']]-C_S[graphs['S']['initial']],
            S_nodes=len(graphs['S']['goal_levels']),S_edges=len(graphs['S']['winning_edges']),
            M_nodes=len(graphs['M']['goal_levels']),M_edges=len(graphs['M']['winning_edges']))
        summary.append(row);write(out/'progress.json',summary)
        print('case prepared',row,flush=True)
    write(out/'summary.json',dict(passed=True,cases=summary,controller_runs=0,
          development_only=True,formal_batch_enabled=False,algorithm_frozen=False))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--rebuild',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():p.error('use a new output directory')
    args.output.mkdir(parents=True)
    for name in ('prepare_pilot.py','cross_stop.py'):shutil.copy2(Path(__file__).with_name(name),args.output/name)
    try:prepare(args.rebuild,args.output)
    except Exception as error:
        write(args.output/'failure.json',dict(error=repr(error),exit_code=1));raise
    return 0


if __name__=='__main__':raise SystemExit(main())
