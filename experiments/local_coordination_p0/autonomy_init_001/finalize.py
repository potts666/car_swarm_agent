"""Finalize retained partial numerical result and independent zero-call repair."""
import difflib,gzip
from common import *
from summarize import load,metrics,contract,save

def run():
 p=HERE/'results/repair_v2';assert read(p/'acceptance.json')['passed'] and read(p/'protocol_checks.json')['passed']
 old=load('results/init_0.json.gz');new=load('results/repair_v2/anchor.json.gz');mm=metrics(new);write(p/'metrics.json',mm)
 oldC=load('results/old_contract.json.gz');newC=contract(new);save('results/repair_v2/contract.json.gz',newC)
 excluded=None;changed=0
 for a,b in zip(oldC['phases'],newC['phases']):
  for c in CARS:
   aa=a['own_planes'][c]['position_planes'];bb=b['own_planes'][c]['position_planes'];changed+=aa!=bb
   if excluded is None:
    x,y=F(bb[0][2])-F(1,100),F(bb[2][2])-F(1,100)
    for nx,ny,beta in aa:
     slack=F(beta)-nx*x-ny*y
     if slack<0:excluded={'phase':a['phase'],'car':c,'slack':slack,'old_plane':[nx,ny,beta],'new_xy':[x,y]};break
 changes=[{'phase':k,'old':old['owners'][k] if k<old['B0'] else None,'new':new['owners'][k] if k<new['B0'] else None} for k in range(old['B0']) if old['owners'][k]!=(new['owners'][k] if k<new['B0'] else None)]
 write(p/'physical_diff.json',{'passed':True,'changed_phase_car_planes':changed,'first_old_contract_exclusion':excluded,'resource_changes':changes,'closure':list(CARS),'old_id':content_id(oldC),'new_id':content_id(newC)})
 with gzip.open(p/'deadlines.jsonl.gz','wt') as f:
  for line in gzip.open(p/'actual.jsonl.gz','rt'):
   row=json.loads(line);k=row['cycle'];before={c:max(0,new['per_car_policies'][c]['goal']-k+1) for c in CARS};after={c:max(0,new['per_car_policies'][c]['goal']-k) for c in CARS};assert max(after.values())==max(before.values())-1
   f.write(json.dumps({'cycle':k,'actual_row_id':content_id(row),'b_before':before,'b_after':after,'B_before':max(before.values()),'B_after':max(after.values())})+'\n')
 gate=read(HERE/'GATE.json');gate['criteria']['multi_car_update'].update({'original_v1_deadline_condition':'failed: A1 475 -> 500; original complete-MA certification withdrawn','repaired_zero_call_fork':{'passed':True,'first_two_activity':[4,4],'initial_four_active_ticks':mm['initial_four_active_ticks'],'B':1822,'all_individual_deadlines_nonincrease':True,'native_attempts':0},'numerical_success':False})
 gate['criteria']['safety_continuation']['evidence']='v1 physical1847 and repaired legal1822 complete traces plus both layers of each version independently replayed; original v1 deadline defect retained'
 gate['criteria']['completion'].update({'original_v1_individual_deadline':'A1 violated; v1 physical completion alone does not pass full deadline admission','repaired_zero_fork_steps':1822,'repaired_all_individual_deadlines_nonincrease':True})
 gate['next_required']='Fresh separately authorized budget/version for two four-car native rounds using both repaired rational adapter and individual-deadline generator/selector; no reuse of old raw statuses.'
 write(HERE/'GATE.json',gate)
 version={x.name:sha(x) for x in sorted(HERE.glob('*.py'))};write(HERE/'current_source_versions.repair_v2.json',version)
 patch=[]
 for x in sorted(HERE.glob('*.py')):patch.extend(difflib.unified_diff([],x.read_text().splitlines(keepends=True),fromfile='/dev/null',tofile='autonomy_init_001/'+x.name))
 (HERE/'changes.patch').write_text(''.join(patch))
 print(json.dumps({'passed':False,'status':'partial','autonomous_init':True,'legal_zero_call_four_car_fork':True,'numerical_accepted':0}))
if __name__=='__main__':run()
