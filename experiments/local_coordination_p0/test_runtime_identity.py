"""Equivalent exact zero increments must retain the same committed identity."""
import json
from fractions import Fraction as F
from pathlib import Path
from pilot_control import load_model,initial_state,initial_plan,object_by_id,admit_delta,validate_update


def main():
    here=Path(__file__).resolve().parent
    model=load_model(here/'results/source_rebuild_005')
    root=here/'results/pilot_objects_011'
    cfg=json.loads((root/'asymmetric_baseline/config.json').read_text())
    common=json.loads((root/'asymmetric_baseline/coordination.json').read_text())
    edge=common['initial_policy'][common['initial']]
    cert=object_by_id(root,edge['object']);state=initial_state(model,cfg)
    parent=initial_plan(model,state,cert,edge,stopped=True)
    a=admit_delta(model,state,parent,cert,0,[[0,0]]*32,F(0),F(0),F(0))
    b=admit_delta(model,state,parent,cert,0,[[F(0),F(0)]]*32,F(0),F(0),F(0))
    assert a['id']==b['id'] and a['proof']==b['proof']
    # The runtime validator recomputes its positive scale; use its actual box.
    from pilot_control import receiver_room,derivative_bounds
    rp,rt,_=receiver_room(model,parent,cert,0,cfg);bounds=derivative_bounds()
    scale=min(rp/(bounds['Gp']+1),rt/(bounds['P1']+1),parent['margin']/(bounds['Gc']+1),F(1,100))/2
    a=admit_delta(model,state,parent,cert,0,[[0,0]]*32,rp,rt,scale)
    validate_update(model,state,parent,a,cert,0,cfg)
    print('integer/rational zero identity and committed-update validation passed')


if __name__=='__main__':main()
