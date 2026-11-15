"""Certified reference cuts and immutable transfer objects, not a new route planner.

Position/heading domains retain the exact-flow image of the archived source set
(motion theory (27),(28a)). Numeric boxes do not replace that correlated image.
"""
from fractions import Fraction as F
import copy
import hashlib
import itertools
import json

H = F(1, 5)
CARS = ('A1','A2','B1','B2')


def content_id(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),default=str).encode()).hexdigest()


class Motion:
    def __init__(self, key, data, source_hash, stage_radii):
        self.key, self.data = key, data
        self.source_hash = source_hash
        self.radii = stage_radii[data['source']]
        self.knots = [tuple(map(F,row)) for block in data['blocks'] for row in block['knots']]
        self.inputs = [((v1-v0)/H,(d1-d0)/H) for v0,v1,d0,d1 in self.knots]
        assert len(self.inputs) == data['ticks']
        for a,b in zip(data['blocks'],data['blocks'][1:]):
            assert a['end_pose_Q'] == b['start_pose_Q']
        for a,b in zip(self.knots,self.knots[1:]):
            assert a[1] == b[0] and a[3] == b[2]

    def domain(self, cut):
        if type(cut) is not int or not 0 <= cut <= len(self.inputs):
            raise ValueError('invalid motion phase')
        v,d = (self.knots[cut][0],self.knots[cut][2]) if cut < len(self.inputs) else (self.knots[-1][1],self.knots[-1][3])
        value = dict(motion=self.key,phase=cut,source_object_sha256=self.source_hash,
            source_pose_Q=self.data['blocks'][0]['start_pose_Q'],source_radii=self.radii,
            v=str(v),delta=str(d),pose_definition='exact ZOH prefix image; SE(2) source correlation retained',
            certificate_scope='archived full-source continuous envelope; no new geometry')
        value['id'] = content_id(value)
        return value

    def window(self, cut, n=32):
        self.domain(cut)
        return (self.inputs[cut:cut+n]+[(F(0),F(0))]*n)[:n]

    def continuation(self, phase):
        self.domain(phase)
        return self.inputs[phase] if phase < len(self.inputs) else (F(0),F(0))


def pair_partition(groups):
    flat = [c for group in groups for c in group]
    if len(flat)!=4 or set(flat)!=set(CARS) or any(not group for group in groups):
        raise ValueError('partition must contain each car exactly once')
    owner = {c:j for j,group in enumerate(groups) for c in group}
    pairs = list(itertools.combinations(CARS,2))
    return dict(inside=[p for p in pairs if owner[p[0]]==owner[p[1]]],
                boundary=[p for p in pairs if owner[p[0]]!=owner[p[1]]])


def transfer_bundle(motion, cut, groups, version, coarse_edge, budget_after_receiver):
    actor = motion.data['car']; receiver_phase = min(cut+32,len(motion.inputs))
    if coarse_edge['actor'] != actor or coarse_edge['motion'] != motion.key:
        raise ValueError('coarse edge does not bind this motion/actor')
    return dict(version=version, groups=groups, pair_contracts=pair_partition(groups),
        state_domain=motion.domain(cut), receiver=motion.domain(receiver_phase),
        permission_owner=actor, coarse_edge=copy.deepcopy(coarse_edge),
        inputs={c:motion.window(cut) if c==actor else [(F(0),F(0))]*32 for c in CARS},
        input_ownership={c:c for c in CARS},
        tail=motion.continuation(receiver_phase), remaining_ticks=len(motion.inputs)-cut,
        receiver_deadline_ticks=receiver_phase-cut,
        receiver_continuation_time_bound=len(motion.inputs)-receiver_phase+budget_after_receiver,
        budget_kind='R maximum-chain bound; not C or an optimized controller observation',
        budget=len(motion.inputs)-cut+budget_after_receiver)


def check_group_transfer(old, proposed):
    """Repartition the same certified branch; permission changes need a new edge."""
    if proposed['version'] != old['version']+1:
        raise ValueError('stale or skipped version')
    if proposed['pair_contracts'] != pair_partition(proposed['groups']):
        raise ValueError('missing or mislabeled pair contract')
    for key in old:
        if key not in ('version','groups','pair_contracts') and proposed.get(key) != old[key]:
            raise ValueError('transfer changes certified dependency: '+key)
    return copy.deepcopy(proposed)


def check_permission_edge(previous, proposed, graph, motions, U):
    """Only at the certified coarse receiver; verifies the whole graph interface."""
    if previous['target'] != proposed['source'] or proposed not in graph['winning_edges']:
        raise ValueError('permission handoff lacks a common certified node')
    source,target = proposed['source'],proposed['target']
    if proposed['actor'] != motions[proposed['motion']]['car']:
        raise ValueError('permission and actual moving actor disagree')
    if motions[proposed['motion']]['ticks']+U[target] > U[source]:
        raise ValueError('replacement exceeds witnessed continuation budget')
    return dict(owner=proposed['actor'], source=source, target=target, budget=U[source])


def audit_moving_objects(rebuild):
    """Exact input/interface checks on the fully reverified package; no new OCP."""
    from pathlib import Path
    rebuild = Path(rebuild)
    assert json.loads((rebuild/'summary.json').read_text())['status'] == 'passed'
    base = rebuild/'work/NTU_DMPC_Dissertation/graph_certificate'
    graph = json.loads((base/'joint_graph.json').read_text())
    data = json.loads((base/'motions.json').read_text())
    budget = json.loads((rebuild/'recomputed_budgets.json').read_text())
    for name,digest in budget['source_sha256'].items():
        assert hashlib.sha256((base/name).read_bytes()).hexdigest() == digest
    for key in ('certificate_graph.py','../M2b_首段自由度与递归可行性/route_zoh_certificate.py'):
        # The immutable reconstruction manifest binds the model scripts separately.
        path = (rebuild/'work/NTU_DMPC_Dissertation'/key).resolve()
        rel = str(path.relative_to((rebuild/'work').resolve()))
        expected = json.loads((rebuild/'input_verification.json').read_text())['files'][rel]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected
    radii = graph['metadata']['stage_radii']; templates = {}
    checked = moving = steering = 0
    for key,value in data.items():
        motion = Motion(key,value,budget['source_sha256']['motions.json'],radii)
        templates[key] = motion
        for cut in range(len(motion.inputs)):
            before,after = motion.window(cut),motion.window(cut+1)
            assert before[1:]+[motion.continuation(min(cut+32,len(motion.inputs)))] == after
            domain = motion.domain(cut); v,delta = F(domain['v']),F(domain['delta'])
            moving += int(v>0); steering += int(v>0 and delta!=0)
            for a,w in before:
                assert abs(a)<=1 and abs(w)<=1
                v+=H*a;delta+=H*w
                assert 0<=v<=2 and abs(delta)<=F(1,2)
            end = motion.domain(min(cut+32,len(motion.inputs)))
            assert (v,delta)==(F(end['v']),F(end['delta']))
            checked+=1
    edge = next(e for e in graph['winning_edges'] if e['source']==graph['initial'])
    motion = templates[edge['motion']]
    cut = next(j for j in range(1,len(motion.inputs)-32)
               if F(motion.domain(j)['v'])>0 and motion.continuation(j+32)!=(0,0))
    old = transfer_bundle(motion,cut,[['A1','A2'],['B1'],['B2']],1,edge,budget['R'][edge['target']])
    before_id = content_id(old)
    merged = copy.deepcopy(old);merged.update(version=2,groups=[['A1','A2','B1'],['B2']])
    merged['pair_contracts'] = pair_partition(merged['groups'])
    assert len(merged['pair_contracts']['inside'])==3 and len(merged['pair_contracts']['boundary'])==3
    merged = check_group_transfer(old,merged)
    split = copy.deepcopy(merged);split.update(version=3,groups=[['A1'],['A2','B1'],['B2']])
    split['pair_contracts'] = pair_partition(split['groups']);check_group_transfer(merged,split)
    assert old['receiver_deadline_ticks']+old['receiver_continuation_time_bound'] == old['budget']
    assert content_id(old)==before_id
    rejections=[]
    for kind in ('velocity_jump','wrong_zero_tail','wrong_receiver','stale_version','wrong_permission','input_ownership','deadline_reset'):
        bad=copy.deepcopy(merged)
        if kind=='velocity_jump':bad['state_domain']['v']=str(F(bad['state_domain']['v'])+1)
        if kind=='wrong_zero_tail':bad['tail']=(F(0),F(0))
        if kind=='wrong_receiver':bad['receiver']=motion.domain(cut)
        if kind=='stale_version':bad['version']=0
        if kind=='wrong_permission':bad['permission_owner']=next(c for c in CARS if c!=edge['actor'])
        if kind=='input_ownership':bad['input_ownership'][edge['actor']]='B2'
        if kind=='deadline_reset':bad['receiver_deadline_ticks']+=1
        try:
            check_group_transfer(old,bad)
        except ValueError:
            rejections.append(kind)
        else:
            raise AssertionError('invalid transfer accepted: '+kind)
        assert content_id(old)==before_id
        # This remains the original, certified physical first input.
        a,w=old['inputs'][edge['actor']][0]
        assert (F(old['state_domain']['v'])+H*a,F(old['state_domain']['delta'])+H*w) == (
            F(motion.domain(cut+1)['v']),F(motion.domain(cut+1)['delta']))
    previous,following = next((a,b) for a in graph['winning_edges'] for b in graph['winning_edges']
                             if a['target']==b['source'] and a['actor']!=b['actor'])
    permission = check_permission_edge(previous,following,graph,data,budget['R'])
    next_bundle=transfer_bundle(motion,cut+1,old['groups'],old['version'],edge,budget['R'][edge['target']])
    assert next_bundle['budget']==old['budget']-1
    return dict(passed=True,templates=len(templates),input_windows_checked=checked,
        nonzero_velocity_nodes=moving,nonzero_velocity_and_steering_nodes=steering,
        three_car_merge_split_passed=True,rejected=rejections,permission_handoff=permission,
        example_before=old,example_merged=merged,example_split=split,
        receiver_definition='exact-flow images of source sets, not uncorrelated numeric boxes',
        numeric_arbitrary_state_membership_checked=False,control_loop_integration=False,
        new_cross_stop_connections=False,
        scope='certified branch inheritance and dependency transfer; not a new MPC controller or performance experiment')
