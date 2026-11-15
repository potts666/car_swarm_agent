"""Explicit physical-role/public-ID mapping; no trajectory or state is inferred."""
import copy
from m2b_scene import CARS,ORDER,SIGMA,PREDECESSOR
from m2a_logs import plan_id


class Identity:
    def __init__(self,mapping=None):
        self.ids=dict(mapping or {c:c for c in CARS})
        if set(self.ids)!=set(CARS) or len(set(self.ids.values()))!=4 or any(not isinstance(v,str) or not v or v=='channel' for v in self.ids.values()):
            raise ValueError('identity map must be a four-role bijection')
        self.roles={v:k for k,v in self.ids.items()}
        self.cars=tuple(self.ids[c] for c in CARS)
        self.order=tuple(self.ids[c] for c in ORDER)
        self.signs={self.ids[c]:SIGMA[c] for c in CARS}
        self.sides={self.ids[c]:c[0] for c in CARS}
        self.predecessors={self.ids[c]:self.ids.get(PREDECESSOR[c]) for c in CARS}

    def context(self,context,public=True):
        result=copy.deepcopy(context);mapping=self.ids if public else self.roles
        result['owner']=mapping[result['owner']] if result['owner'] is not None else None
        result['released']=sorted(mapping[c] for c in result.get('released',[]))
        return result

    def report_to_role(self,r):
        return {**r,'car':self.roles[r['car']],'context':self.context(r['context'],False)}

    def plan_to_role(self,p):
        if p['id']!=plan_id(p):raise ValueError('public plan content changed')
        result={**p,'car':self.roles[p['car']],'report':self.report_to_role(p['report']),'context':self.context(p['context'],False)}
        result['id']=plan_id(result)
        return result

    def scene(self,scene):
        result=copy.deepcopy(scene)
        for key in ['initial','goals']:result[key]={self.ids[c]:v for c,v in scene[key].items()}
        return result

    def event(self,event):
        result=copy.deepcopy(event)
        for key in ['car','owner','old_owner','proposed_owner']:
            if key in result and result[key] is not None:result[key]=self.ids[result[key]]
        if 'eligible' in result:result['eligible']=[self.ids[c] for c in result['eligible']]
        if 'plans' in result:result['plans']={self.ids[c]:v for c,v in result['plans'].items()}
        return result
