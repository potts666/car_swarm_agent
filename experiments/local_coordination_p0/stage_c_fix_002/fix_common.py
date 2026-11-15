from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/"base_c"))
from c_common import *
HERE=ROOT
def setup():return Certificates(ROOT/"inputs"),legacy.load_model(REBUILD)


def commit_id(token):return content_id(dict(context=token['context'],plans=token['plans']))


def validate_commit_token(data,state,old,plan,token,phase):
    """Execution selects the one plan whose complete context all four cars acknowledged."""
    ctx=token['context'];c=data.cert['car']
    expected=dict(cycle=phase,history=state['history'],state_time_s=str(phase*H),owner=c,permission_version=data.witness['version'],joint_source=data.edge['source'],joint_target=data.edge['target'],groups=A.groups(phase,c),pair_contracts=__import__('moving_objects').pair_partition(A.groups(phase,c)),receiver=plan['receiver'],pending_reset=plan['pending_reset'],witness=plan['witness'],witness_version=plan['witness_version'],budget=plan['budget'],plan_id=plan['id'])
    if content_id(ctx)!=content_id(expected):raise ValueError('mixed commit context')
    kind='locked_inherited' if plan['id']==old['id'] else 'box_prefix_and_actual_K'
    ids={}
    for car in CARS:
        public=dict(car=car,context=ctx,controls=A.window(plan) if car==c else [(F(0),F(0))]*32,certificate=dict(plan_id=plan['id'],witness=plan['witness'],kind=kind));ids[car]=content_id(public)
    if token['plans']!=ids or token['application_messages']!=40:raise ValueError('inconsistent four-car commit')
    return commit_id(token)
