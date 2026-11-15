"""Resource and outside-exit permissions committed with one four-plan barrier."""
from common import *
from witness import actions,plan_id
from certify_domains import check_domain
from physics import body
sys.path.insert(0,str(HERE/'frozen_protocol'))
from m2b_protocol import Barrier
RESOURCE_ID=content_id(dict(road=ROADS,x_abs_max=16))


def prepare(state,wid,parent):
    from strategy import validate_strategy
    validate_strategy(wid)
    w=get(HERE/'objects',wid);k=state['cycle'];owner=w['owners'][k];change=owner!=state['owner'];version=state['permission_version']+int(change)
    return normal(dict(resource_id=RESOURCE_ID,owner=owner,permission_version=version,cycle=k,history=state['history'],source_snapshot=state_record(state),certificate_id=w['domain_certificate'],witness_id=wid,parent_plan_id=parent,plan_id=plan_id(state,wid),remaining_budget=2802-k,actions=actions(w,k),enter_condition='complete body sweep holds reservation before intersecting |x|<=16',release_condition='actual G_out, whole body outside resource, compatible saved exit/next-entry tails',receiver_chain=w['events']))


def validate_request(state,request):
    w=get(HERE/'objects',request['witness_id']);expected=prepare(state,request['witness_id'],request['parent_plan_id'])
    if request!=expected:raise ValueError('stale or damaged resource/plan version')
    if request['parent_plan_id']!=plan_id(state,state['witness']):raise ValueError('wrong actual inherited parent')
    if request['witness_id']!=state['witness']:
        previous=get(HERE/'objects',state['witness']);k=state['cycle']
        if any(w['controls'][c][:k]!=previous['controls'][c][:k] for c in CARS) or w['owners'][:k]!=previous['owners'][:k]:raise ValueError('proposal rewrites executed inputs or permissions')
        if w.get('optimization_origin'):
            raw=get(HERE/'objects',w['optimization_origin'])
            if w['optimization_phase']!=k or raw['history']!=state['history'] or raw['source_witness']!=state['witness']:raise ValueError('stale numerical source')
        elif w['adoption_rank']*663+55!=k:raise ValueError('stale authorization source phase')
    if w['ticks']-state['cycle']>2802-state['cycle'] or w['domain_certificate']!=read(HERE/'domain_checks.json')['certificate_id']:raise ValueError('completion/certificate binding')
    domains=read(HERE/'domain_checks.json')['domains'];new=request['owner'];old=state['owner']
    if new!=old:
        if old:
            check_domain(state,old,'G_out',domains);box=m.axis_box(body(state['poses'][old],old));near=F(box[0],m.Q) if direction(old)==1 else -F(box[1],m.Q)
            if near-TUBE<=16:raise ValueError('whole body not released')
        if new:
            if new in state['served']:raise ValueError('already served owner')
            check_domain(state,new,'W_in',domains)
    return True


def public_plans(state,request):
    w=get(HERE/'objects',request['witness_id']);k=state['cycle'];plans={}
    for c in CARS:
        cmds=w['controls'][c][k:k+32]+[['0','0']]*max(0,32-(2802-k))
        public=dict(car=c,context=request,controls=cmds,permission=request['actions'][c],source_pose_Q=pose_rows(state['poses'][c]),v=str(state['v'][c]),delta=str(state['delta'][c]),certificate_id=w['domain_certificate']);key=content_id(public);plans[c]=dict(**public,id=key,kind='final')
    return plans


def commit(state,request):
    validate_request(state,request);barrier=Barrier(request);plans=public_plans(state,request);wire=[]
    def deliver(sender,target,kind,payload):
        raw=json.dumps(dict(sender=sender,target=target,kind=kind,payload=payload),sort_keys=True,default=str);decoded=json.loads(raw);wire.append(content_id(decoded));return decoded['payload']
    for c in CARS:
        for target in [o for o in CARS if o!=c]+['coordinator']:deliver(c,target,'snapshot',request['source_snapshot'])
    for c in CARS:
        for target in [o for o in CARS if o!=c]+['coordinator']:
            p=deliver(c,target,'final',plans[c])
            if target=='coordinator':barrier.publish(p)
    ids={c:p['id'] for c,p in plans.items()}
    for c in CARS:validate_request(state,request);barrier.accept(deliver(c,'coordinator','ack',dict(car=c,context=request,plans=ids,accepted=True)))
    token=barrier.token()
    for c in CARS:deliver('coordinator',c,'commit',token)
    return dict(**token,commit_id=content_id(token),application_messages=len(wire),message_digest=content_id(wire))


def select_for_execution(state,token):
    ctx=token['context'];validate_request(state,ctx);plans=public_plans(state,ctx)
    if token['plans']!={c:p['id'] for c,p in plans.items()} or token['commit_id']!=content_id(dict(context=ctx,plans=token['plans'])) or token['application_messages']!=40:raise ValueError('mixed four-car commit')
    w=get(HERE/'objects',ctx['witness_id']);owner=ctx['owner']
    if owner!=state['owner'] and owner:state['served'].append(owner)
    state.update(owner=owner,permission_version=ctx['permission_version'],witness=ctx['witness_id'])
    return {c:tuple(map(F,w['controls'][c][state['cycle']])) for c in CARS}
