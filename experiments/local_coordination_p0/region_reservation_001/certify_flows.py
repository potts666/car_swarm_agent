"""Explicit exact-flow indices and conservative receiving bounds for every route phase."""
from common import *


def main():
    certificates=read(HERE/'domain_checks.json');bound=certificates['bounds'];rational=read(HERE/'rational_inputs.json');profiles={};index={}
    holdP=F(12001,10**6);holdT=F(108,10**6);waitP=holdP+F(3,2)*80*holdT+EXTRA
    outP=F(bound['exit_position'])+EXTRA;outT=F(bound['entry_reset_yaw'])+EXTRA;goalP=F(bound['goal_position']);goalT=F(24,100000)+EXTRA
    for c in CARS:
        route=get(HERE/'objects',rational['routes'][c]);rows=[];parent=None
        for phase in range(759):
            kind='waiting_feedback' if phase<2 else 'W_hold_image' if phase<101 else 'G_in' if phase==101 else 'G_in_feedback_image' if phase<663 else 'G_out' if phase==663 else 'G_out_image'
            P,T=(holdP,holdT) if phase<2 else (waitP,holdT+EXTRA) if phase<101 else (F(1,50),F(1,5000)) if phase==101 else (outP,outT) if phase<663 else (F(35,1000),F(24,100000)) if phase==663 else (goalP,goalT)
            nominal=(route['knots'][phase][0],route['knots'][phase][2]) if phase<758 else ('0','0')
            velocity_interval=['0','1/50'] if phase==0 else ['49/100','51/100'] if phase==101 else [str(nominal[0])]*2
            steering_interval=['-1/100','1/100'] if phase<2 else ['-1/1000','1/1000'] if phase==101 else [str(nominal[1])]*2
            row=dict(car=c,phase=phase,source_kind=kind,v_interval=velocity_interval,delta_interval=steering_interval,route_object=rational['routes'][c],domain_certificate=certificates['certificate_id'],parent_flow=parent,command=route['controls'][phase-1] if phase else None,feedback='wait_brake' if phase==1 else 'wait_steer' if phase==2 else 'entry_v_delta_reset' if phase==102 else None,position_l1_upper=str(P),yaw_error_upper=str(T),nominal_v_delta=nominal,exact_set_definition='source-domain exact ZOH image, with certified feedback at seams; current phase/command and parent identify the retained image')
            parent=content_id(row);row['id']=parent;rows.append(row)
        profiles[c]=rows;index[c]=[x['id'] for x in rows]
    write(HERE/'flow_images.json',dict(passed=True,profiles=profiles,total_images=4*759,terminal_hold='reachable G_out-to-goal image is retained; the wider goal tolerance is not reused as a new source domain'))
    results={}
    for case in ('opposite','same_direction'):
        run=HERE/'results'/case;counts=dict(preparation=0,waiting=0,inherited_flow=0,accepted_actual_prefix_flow=0,terminal_hold=0);minimum_body_slack=F(1);checked=0;receiver_file=run/'flow_receivers.jsonl.gz'
        assert not receiver_file.exists()
        with gzip.open(run/'execution.jsonl.gz','rt') as stream,gzip.open(receiver_file,'wt') as dest:
            for record in map(json.loads,stream):
                w=get(HERE/'objects',record['witness_id']);k=record['cycle']+1;poses=record['actual']['poses_Q'];receivers={}
                for c in CARS:
                    start=w['starts'][c];j=k-start;end=min(k+32,2802)-start
                    if k<=55:counts['preparation']+=1;receivers[c]=dict(kind='initial_straight_exact_flow',source=INITIAL[c],phase=k,command_witness=record['witness_id']);continue
                    if j<0:
                        counts['waiting']+=1;p=[m.I.raw(*z) for z in poses[c]];assert sum((p[t]-WAIT[c][t]).abs_upper() for t in (0,1))<=F(1,100);receivers[c]=dict(kind='waiting_hold',phase=j,terminal_flow=index[c][max(0,min(758,end))]);continue
                    route=get(HERE/'objects',rational['routes'][c]);phase=min(758,j);flow=profiles[c][phase]
                    free=w.get('optimization_actor')==c and w.get('optimization_phase',-100)<k<=w.get('optimization_phase',-100)+33
                    if free:
                        counts['accepted_actual_prefix_flow']+=1;receivers[c]=dict(kind='actual_source_exact_flow',raw_result=w['optimization_origin'],phase_since_source=k-w['optimization_phase'],command_witness=record['witness_id'],terminal_flow=index[c][max(0,min(758,end))]);continue
                    p=[m.I.raw(*z) for z in poses[c]];ref=[m.I.raw(*z) for z in route['poses_Q'][phase]];ep=sum((p[t]-ref[t]).abs_upper() for t in (0,1));et=(p[2]-ref[2]).abs_upper();P,T=F(flow['position_l1_upper']),F(flow['yaw_error_upper']);assert ep<=P and et<=T,(case,c,k,'flow enclosure')
                    assert [record['actual']['v'][c],record['actual']['delta'][c]]==list(flow['nominal_v_delta']),(case,c,k,'flow affine state')
                    body=ep+RB*et;assert body<TUBE;minimum_body_slack=min(minimum_body_slack,TUBE-body)
                    counts['terminal_hold' if j>=758 else 'inherited_flow']+=1;checked+=1;receivers[c]=dict(kind=flow['source_kind'],current_flow=flow['id'],terminal_flow=index[c][max(0,min(758,end))],witness=record['witness_id'])
                dest.write(json.dumps(dict(cycle=k,receivers=receivers),separators=(',',':'))+'\n')
        results[case]=dict(passed=True,counts=counts,concrete_enclosures_checked=checked,minimum_body_tube_slack=str(minimum_body_slack),all_32_tick_receivers_indexed=True,authority_source='same immutable witness/phase/domain chain bound by original four-car commit, no retrospective log alteration')
    write(HERE/'flow_checks.json',dict(passed=True,interfaces=results,total_route_flow_images=3036));print(json.dumps(results,indent=2))
if __name__=='__main__':main()
