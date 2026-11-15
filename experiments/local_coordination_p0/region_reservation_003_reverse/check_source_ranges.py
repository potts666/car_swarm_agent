"""Exact affine extrema of signed variational rows over whole source rectangles."""
from common import *
from derivatives import builder
from rules import quantize,equality_blocks

def main():
    results=[]
    for name,lo,hi,sign in [('negative',F(-3,10),F(-1,10),1),('zero',F(0),F(0),1),('cross_zero',F(-1,20),F(1,20),1),('cross_zero_opposite_frame',F(-1,20),F(1,20),-1)]:
        v=(lo+hi)/2;r=(hi-lo)/2;ar=r/H
        cert=dict(scale=m.Q,car='a_location_without_direction_semantics',direction_sign=sign,knots=[[str(v),str(v),'0','0']],pose_nodes_Q=[[[0,0],[0,0],[0,0]]]);J=builder.interval_jacobian(m,cert,0,dict(speed_radius=str(r)),32)
        rows={(i,j):(F(a,J['scale']),F(b,J['scale'])) for i,j,a,b in J['nonzero_intervals']};extrema=[]
        # These exact entries are affine in source v and a; corner extrema prove
        # coverage of the entire rectangular source, including v=0 in its interior.
        for key,formula in [((1,2),lambda v,a:sign*(H*v+H*H*a/2)),((2,4),lambda v,a:(H*v+H*H*a/2)/m.L),((2,6),lambda v,a:(H*H*v/2+H**3*a/3)/m.L)]:
            vals=[formula(x,y) for x in (lo,hi) for y in (-ar,ar)];low,high=rows.get(key,(F(0),F(0)));assert low<=min(vals)<=max(vals)<=high,(name,key)
            extrema.append(dict(entry=key,exact_range=[min(vals),max(vals)],interval=[low,high]))
        results.append(dict(name=name,passed=True,source_v=[lo,hi],source_a=[-ar,ar],fixed_delta=0,frame_sign=sign,jacobian_id=put(HERE/'objects',J),exact_affine_extrema=extrema))
    # A projection plus balanced grid rounding preserves every stationary equality.
    event=next(e for e in read(HERE/'results/A/events.json') if e['kind']=='local_socp');anchor=get(HERE/'objects',event['anchor_id']);raw=get(HERE/'objects',event['raw_result_id']);delta,q=quantize(raw['x'],F(1),anchor['contracts']);frozen,blocks=equality_blocks(anchor['contracts']);assert q['passed'] and all(delta[i]==0 for i in frozen) and all(sum(delta[i] for i in b)==0 for b in blocks)
    result=dict(passed=True,ranges=results,stationary_equalities=dict(passed=True,frozen=len(frozen),zero_sum_blocks=len(blocks),quantization_error=q['error'],budget=q['budget']),new_solver_calls=0,scope='range/variational checks; future policy synthesis uses exact measured v/delta, not an uncertain-speed robust deployment')
    write(HERE/'results/source_range_checks.json',result);print(json.dumps(normal(result),indent=2))
if __name__=='__main__':main()
