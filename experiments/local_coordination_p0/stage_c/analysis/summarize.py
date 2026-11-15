"""C0 original/rounded/retreated residual diagnostics; never accepts a solver point."""
import sys,csv
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from c_common import *
from c_socp import nearest
from socp import evaluate

attempts=read(HERE/'results/repairs_001/attempts.json');rows=[];detail={};data,model=setup()
for attempt in attempts:
    result=get(HERE/'objects',attempt['result_id']);scale=F(result['numerical_variable_scale']);raw=[scale*F.from_float(float(x)) for x in result['raw_solution'][:64]];tau=scale*F.from_float(float(result['raw_solution'][64]))
    assembly=get(HERE/'objects',result['assembly_id']);levels={}
    for alpha in (F(1),F(1,2),F(1,4)):
        point=nearest([alpha*x for x in raw]);check=evaluate(result['rows'],point);level=dict(check=check,error_norm=str(norm_upper([a-alpha*b for a,b in zip(point,raw)])))
        if check['passed']:
            anchor=get(HERE/'objects',result['anchor_id']);state=restore(model,anchor['state']);commands=[tuple(F(x)+point[2*j+k] for k,x in enumerate(u)) for j,u in enumerate(anchor['plan']['remaining'][:32])]
            proposal=numerical_plan(data,state,anchor['plan'],commands)
            try:proof=A.validate_candidate(model,data,state,anchor['plan'],proposal,anchor['phase'])
            except ValueError as exc:level['nonlinear_diagnostic']=dict(passed=False,rejection=str(exc))
            else:level['nonlinear_diagnostic']=dict(passed=True,proof=proof,actually_committed=False)
        levels[str(alpha)]=level
    auxiliary=[]
    for original in result['rows']:
        value=F(original['g_upper'])+sum(F(a)*d for a,d in zip(original['linear'],raw))+F(original['gradient_error'])*tau+F(original['hessian'])*tau*tau/2
        auxiliary.append(dict(name=original['name'],physical_aux_residual=str(value)))
    key=attempt['interface']+'_'+attempt['selection']['variant']+'_'+attempt['selection']['input_scale'].replace('/','_')
    detail[key]=dict(result_id=attempt['result_id'],assembly_id=result['assembly_id'],solver_state_eligible=result['status']=='Solved',raw_true_norm=str(norm_upper(raw)),raw_t=str(tau),raw_physical_aux_rows=auxiliary,rounding_levels=levels,maximum_inner_reserve=str(max(F(x['eta']) for x in result['rounding_reserves'])),reserve_omitted=[x['name'] for x in result['rounding_reserves'] if not x['applied']],note='passing original rows after retreat does not override the required Solved status')
    rows.append(dict(interface=attempt['interface'],variant=attempt['selection']['variant'],scale=attempt['selection']['input_scale'],status=result['status'],eligible=result['status']=='Solved',accepted=result['accepted'],primal=result['primal_residual'],dual=result['dual_residual'],objective_gap=result['absolute_objective_gap'],iterations=result['iterations'],variables=result['variables'],constraints=result['constraint_rows'],raw_quantized_pass=levels['1']['check']['passed'],half_quantized_pass=levels['1/2']['check']['passed'],quarter_quantized_pass=levels['1/4']['check']['passed'],solver_seconds=result['solver_wall_s']))
write(HERE/'analysis/numerical_levels.json',detail)
with (HERE/'C0_comparison.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print('C0 configurations:',len(rows)//2,'eligible:',sum(x['eligible'] for x in rows),'accepted:',sum(x['accepted'] for x in rows))
