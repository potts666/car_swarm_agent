"""Independent saved-raw adoption and obligation seam recomputation, no solve."""
from common import *
from plan_io import load
from quantization import quantize

def run():
 rows=[];seams=[]
 for win in (1,2):
  for rnd in (1,2):
   label=f'e{win}_r{rnd}';e=read(HERE/f'results/numeric/{label}_event.json')
   for c in e['actors']:
    job=load(f'results/numeric/{label}_{c}_job.json.gz');raw=read(HERE/f'results/workers/{label}_{c}_response.json');ad=read(HERE/f'results/numeric/{label}_{c}_admission.json');assert len(raw['x'])==65 and len(raw['s'])==len(raw['z'])==raw['assembly']['shape'][0];delta,q=quantize(raw['x'][:64],F(ad['alpha']),job['lattice']);assert delta==list(map(F,ad['delta'])) and q['integer_coordinates']==ad['quantization']['integer_coordinates'];rows.append({'label':label,'actor':c,'raw_model_id':raw['model_id'],'raw_x_to_adopted_exact_grid':True,'alpha':ad['alpha'],'new_solver_calls':0})
  a=read(HERE/f'results/numeric/e{win}_r1_event.json');b=read(HERE/f'results/numeric/e{win}_r2_event.json');p=load(f'results/numeric/e{win}_r1_committed.json.gz');q=load(f'results/numeric/e{win}_r2_committed.json.gz');assert b['source']==a['successor']
  for c in a['actors']:
   oldphase=a['source']['cycle']+32;newphase=b['source']['cycle']+32;seams.append({'event':win,'actor':c,'old_phase':oldphase,'new_phase':newphase,'old_K_id':content_id(p['controls'][c][oldphase:]),'new_K_id':content_id(q['controls'][c][newphase:]),'parent_before_round2':content_id(p['controls'][c]),'new_own_plan_id':content_id(q['controls'][c]),'old_complete_policy':f'results/numeric/e{win}_r1_committed.json.gz','replacement':'whole new prefix/Q/new-phase K accepted; old complete policy remains valid fallback, old phase is not claimed actually reached'})
 write(HERE/'results/raw_adoption_certificate.json',{'passed':True,'rows':rows,'obligation_replacement':seams,'new_optimizer_calls':0});print('PASS8 raw quantized adoption and4 obligation seams',flush=True)
if __name__=='__main__':run()
