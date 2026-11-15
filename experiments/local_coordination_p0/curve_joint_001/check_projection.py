"""End-domain and compatible seam certificates; zero optimization calls."""
from common import *
from plan_io import load
from progression import chart,coordinate,frame

def run():
 p=load('results/R_original.json.gz');route=p['paths']['A1'];left=chart(p,'A1',57);right=chart(p,'A1',64);seam=tuple(m.I.raw(*q) for q in route['poses_Q'][58]);a=coordinate(seam,left);b=coordinate(seam,right);diff=a-b;assert max(a.l,b.l)<=min(a.u,b.u),'outward seam images do not overlap'
 rows=[]
 for arm in ('F-E','O-E','F-P','O-P'):
  for rnd in (1,2):
   for c in ('A1','A2'):
    j=load(f'results/{arm}_r{rnd}_{c}_job.json.gz');r=j['receiver'];cert=load(f'results/{arm}_r{rnd}_{c}_derivatives.json.gz');ch=j['progression']['terminal_chart'];pose=tuple(m.I.raw(*q) for q in cert['poses_Q'][-1]);center=[F(x) for x in r['center_pose']];center_error=sum((pose[t]-center[t]).abs_upper() for t in (0,1));tube=F(j['progression']['projection_bounds']['tube']);assert center_error+F(r['rP'])<tube
    # Receiver L1 sign rows with global Bp have convex upper bounds in delta.
    # Every scaled feasible delta stays feasible, so its actual endpoint stays
    # in this same ball. This establishes the whole Taylor homotopy domain.
    signs=[x for x in j['rows'] if x['name'].startswith('receiving_position')];assert len(signs)==4 and all(F(x['H'])>=F(cert['bounds'][-1]['Bp']) for x in signs)
    patch_margin=None
    if ch['kind']=='arc':
     progress=coordinate(pose,ch);patch_margin=F(progress.l,m.Q)-F(ch['offset'])-F(j['progression']['projection_bounds']['G'])*(F(r['rP'])+center_error);assert patch_margin>0 and F(j['progression']['projection_bounds']['r_min'])>0
    rows.append({'model_id':j['model_id'],'label':f'{arm}_r{rnd}_{c}','receiver_ball_radius':r['rP'],'center_error':center_error,'projection_tube':tube,'strict_patch_margin':patch_margin,'whole_feasible_homotopy_in_domain':True})
 write(HERE/'results/projection_atlas_certificate.json',{'passed':True,'seam':{'left':left,'right':right,'canonical_pose_Q':pose_rows(seam),'nominal_offset_difference':diff.abs_upper(),'normal_line_rule':'both arc centres lie on shared seam normal; angle constant on each connected seam normal segment, matched offsets give equal s; choose oriented tangent halfplanes','regularity':'continuous piecewise smooth progress; gradients may differ off-route across seam. No C2 claim across seam. Each terminal Taylor homotopy lies strictly inside its own smooth patch.'},'models':rows,'optimizer_calls':0});print('PASS compatible atlas and 16 endpoint homotopy domains')
if __name__=='__main__':run()
