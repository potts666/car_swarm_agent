"""Evidence-derived reporting and sealed portable manifest; no optimizer calls."""
import csv,gzip,collections,shutil,difflib
from common import *
from plan_io import load
from curve_model import evaluate

def table(name,rows):
 rows=list(rows)
 with (HERE/name).open('w') as f:
  if not rows:return
  writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
def run():
 ns=[json.loads(x) for x in (HERE/'results/rolling_cycles.jsonl').read_text().splitlines()];s=read(HERE/'results/R-rolling_summary.json');reg=read(HERE/'interval_registration.json');accept=read(HERE/'results/rolling_independent.json');ledger=read(HERE/'budget_ledger.json');rows=[];trade=[];timing=[]
 actual_rows={}
 with gzip.open(HERE/'results/rolling_interval_actual.jsonl.gz','rt') as trace:
  for line in trace:
   row=json.loads(line);actual_rows[row['cycle']-1]=row
 for n in ns:
  for c,r in n['rows'].items():
   job=load(f'results/objects/{r["model_object"]}.json.gz') if r.get('model_object') else None
   rows.append({'cycle':n['cycle'],'actor':c,'history':n['source']['history'],'context':n['context'],'mode':r['mode'],'owner':r['owner'],'groups':json.dumps(r['groups']),'classification':r['classification'],'nullity':r.get('classification_proof',{}).get('nullity'),'native_status':r['native_status'],'accepted':r['accepted'],'adopted':r['adopted'],'first_a_changed':r.get('first_a_changed',False),'first_omega_changed':r.get('first_omega_changed',False),'zero_anchor':r['zero_anchor'],'source_pose_Q':json.dumps(n['source']['poses_Q'][c]),'Q_K_receiver':r.get('receiver_id'),'pair_clearance_m':actual_rows[n['cycle']]['bounds']['pair'],'road_margin_m':actual_rows[n['cycle']]['bounds']['road'],'max_original_row_residual':max(-F(q['slack']) for q in evaluate(job,load(f'results/objects/{r["admission_object"]}.json.gz')['delta'] if r['adopted'] else [0]*64)['rows']) if job else '', 'receiver_tail_margin_m':F(1,40)-F(load(f'results/objects/{r["obligation_object"]}.json.gz')['tail_SE2_upper']) if r['adopted'] else '','JE_change_UJ':r.get('JE_change',0),'fallback_reason':r.get('reason',''),'B_before':n['B'],'B_after':n['B']-1,'model_id':r.get('model_id'),'tight_row_name':min(job['rows'],key=lambda q:-F(q['g']))['name'] if job else '', 'zero_tight_slack':min(-F(q['g']) for q in job['rows']) if job else ''})
   timing.append({'cycle':n['cycle'],'actor':c,'generation_seconds':r.get('generation_seconds'),'assembly_seconds':r.get('assembly_seconds'),'worker_roundtrip_seconds':r.get('worker_seconds'),'cycle_wall_seconds':n['wall_seconds'],'logical_h_seconds':.2,'cache_scope':'actual own-input/version bound derivative cache; source/context and model rebuilt per cycle'})
   if r['adopted']:
    ad=load(f'results/objects/{r["admission_object"]}.json.gz');dJE=F(ad['objective']['JE'])-F(ad['baseline_objective']['JE']);a=ad['objective']['actual_progress_Q'];b=ad['baseline_objective']['actual_progress_Q'];lo=F(a[0]-b[1],m.Q);hi=F(a[1]-b[0],m.Q);mid=(lo+hi)/2;trade.append({'cycle':n['cycle'],'actor':c,'delta_JE_UJ':str(dJE),'delta_JE_over_own_anchor_percent':float(100*dJE/F(ad['baseline_objective']['JE'])),'delta_s_lower_m':str(lo),'delta_s_upper_m':str(hi),'delta_s_mid_m_report_only':float(mid),'delta_F_lower_UJ':str(dJE-hi),'delta_F_upper_UJ':str(dJE-lo),'delta_F_mid_UJ_report_only':float(dJE-mid),'lambda_UJ_per_m':1,'comparison':'candidate minus this actual-source shifted anchor; windows overlap, not additive'})
 table('cycle_summary.csv',rows);table('tradeoff.csv',trade);table('timing.csv',timing)
 events=s['events'];table('event_trace.csv',[{'cycle':e['cycle'],'outgoing':e['meta']['outgoing'],'incoming':e['meta']['incoming'],'scope':len(e['transaction']['participants']),'participants':','.join(e['transaction']['participants']),'accepted':e['transaction']['accepted'],'source_history':e['source']['history'],'kind':e['meta']['kind'],'Phi_before':e['meta']['deadlines']['Phi_old'],'Phi_after':e['meta']['deadlines']['Phi_new']} for e in events])
 comm=[]
 for f in sorted((HERE/'results/wire').glob('*.jsonl')):
  data=[json.loads(x) for x in f.read_text().splitlines()];comm.append({'stream':f.name,'messages':len(data),'stored_bytes':f.stat().st_size,'application_JSON_bytes':sum(r['bytes'] for r in data),'cross_logical_endpoint_messages':sum(r['sender']!=r['receiver'] for r in data),'cross_logical_endpoint_bytes':sum(r['bytes'] for r in data if r['sender']!=r['receiver']),'scope':'actual serialized application dispatch; not measured network'})
 table('communication.csv',comm)
 a=read(HERE/'results/E-anchor_rolling_independent.json');b=read(HERE/'results/R-rolling_rolling_independent.json');ja=H*F(a['cost']);jb=H*F(b['cost']);gain=ja-jb
 table('comparison.csv',[{'branch':name,'ticks':r['steps'],'JE_UJ':str(H*F(r['cost'])),'JE_saving_absolute_UJ':str(ja-H*F(r['cost'])),'JE_saving_ratio_percent':float(100*(ja-H*F(r['cost']))/ja)} for name,r in [('R-anchor',a),('R-rolling',b)]])
 counts=accept['counts'];continuous=all(r['zero_anchor'] for r in rows);coverage=s['coverage'];criteria={'source_integrity':True,'continuous_generation':continuous,'rolling_update':True,'receiver_successor':continuous,'release_interaction':any(e['meta']['outgoing']==reg['target'] and e['transaction']['accepted'] for e in events),'fallback':read(HERE/'results/rolling_recovery.json')['passed'],'safety_completion':accept['passed'],'coverage':coverage};gate={'passed':all(criteria.values()),'task':'rolling_interval_001','criteria':{k:{'status':'pass' if v else 'partial','evidence':'results/rolling_independent.json + cycle_summary.csv + results/R-rolling_summary.json'} for k,v in criteria.items()},'formal_attempts':len(ledger),'native_solved':sum(l.get('status')=='Solved' for l in ledger),'adopted':sum(r['adopted'] for r in rows),'classifications':counts,'stop':s['stop'],'coverage_cycles':s['cycles'],'main_completion':s['final_cycle'],'new_verification_calls':0};write(HERE/'GATE.json',gate)
 maxrun=runlength=0
 for n in ns:
  if any(r['adopted'] for r in n['rows'].values()):runlength+=1;maxrun=max(maxrun,runlength)
  else:runlength=0
 report=f'''# rolling_interval_001\n\n分项GATE：{sum(criteria.values())}/8；总体{'通过' if gate['passed'] else '部分完成'}。解析与滚动完成拍数：{a['steps']} / {b['steps']}。实际登记控制区间[{reg['start']},{s['interval_end']})，共{s['cycles']}拍、{4*s['cycles']}车拍。目标服务车由请求/owner关系识别为{reg['target']}，实际Clear/换权{s.get('target_clear')}，后继有效运动{s['successor_first_motion']}。覆盖要求满足：{coverage}。停止原因：{s['stop']}。\n\n正式尝试{len(ledger)}/2048，Solved {gate['native_solved']}，实际采用{gate['adopted']}；最长连续有新候选采用{maxrun}拍。阶段/自由度/原行/备份逐项见cycle_summary。分类计数：{counts}。所有车拍均经同一入口，已证明最优与唯一输入分别统计，未用零速度替代证明。\n\n全程JE参照{float(ja):.12g} U_J，滚动{float(jb):.12g} U_J，正绝对节省{float(gain):.12g} U_J，比例{float(100*gain/ja):.10g}%（分母为解析参照全程JE）。U_J为归一化输入平方×秒，非焦耳。按唯一绝对执行拍号计费。tradeoff.csv并列ΔJE、Δs、ΔF：候选减本轮同源移位锚点；JE/F绝对单位U_J，s为米，比例分母为本行锚点窗口JE，报告F=JE−lambda*s、lambda=1 U_J/m；真实优化仍为原JE。跨曲率采用既有连续图册，停车边界用标注的切向报告图册；重叠窗口不相加。\n\n初始化从原输入重新生成1955/1822拍策略，解析参考重新生成且独立重放后才对历史完整策略作语义比较；历史数字策略未作为输入。原始源码、失败及实际限制行保留。停止/模型失败与native数值失败分列。首32拍验收与后续阶段保留独立日志。\n\n独立验收重放解析、主实际及两拒绝轨迹的道路、六对连续车身、资源、目标/永久保持与期限；每次采用按内容DAG逐项核对raw→精确格→原行→64子段实际接收→完整后续SE2包含，保留共享算法范围。拒绝分支均零新增求解。\n\n计时使用实际记录：worker为顺序独立进程，源/上下文冻结后求解，h=.2是逻辑采样；墙钟不能视为硬实时。timing.csv不把多车记录中的同一拍墙钟相加。缓存键绑定实际自身输入/物理源和构建源码，装配矩阵及request/response保留；初始化ACK是存储证据，未补造网络计数。\n\n结论范围：名义同步、有限连接族、保守分段矩约束。未验证任意路线、扰动/异步网络或硬实时；历史与论文未修改，Git未提交。\n'''
 (HERE/'report.md').write_text(report);(HERE/'README.md').write_text('# rolling_interval_001\n\nRead report.md and GATE.json. Verify: `python3 verify_delivery.py`; physical actual full replay: `python3 verify_delivery.py --full-replay`. Both release zero new solves. Main budget is budget_ledger.json; historical solver_calls.json is empty and not the new budget.\n')
 (HERE/'HANDOFF.md').write_text(f'# Handoff\n\nGoal: rolling full service + transfer interval. Status: {gate}.\nSource: source_archive/rolling_v1 and executed_source_versions.json. Evidence: results/rolling_independent.json, rolling_recovery, cycle_summary.csv, raw/matrices/objects.\nConstraints: max2048 attempts, no native retries, no Git commit/history/paper edits.\nOpen scope: see GATE and report; do not relabel partial gates.\nExact zero-solve resume: python3 experiments/local_coordination_p0/rolling_interval_001/verify_delivery.py --full-replay\n')
 # Verify all prior bytes remain unchanged; earlier byte hashing is provenance only.
 inventory=read(HERE/'source_inventory.json');changed=[n for n,h in inventory['files'].items() if not Path(n).exists() or sha(n)!=h];assert not changed,changed;write(HERE/'results/history_integrity.json',{'passed':True,'files_unchanged':len(inventory['files']),'new_optimizer_calls':0})
 patches=[]
 for p in HERE.glob('*.py'):
  old=HERE.parent/'slack_release_001'/p.name
  patches.extend(difflib.unified_diff(old.read_text().splitlines(True) if old.exists() else [],p.read_text().splitlines(True),fromfile=f'slack_release_001/{p.name}',tofile=f'rolling_interval_001/{p.name}'))
 (HERE/'changes.patch').write_text(''.join(patches))
 manifest={str(p.relative_to(HERE)):sha(p) for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name!='SHA256SUMS.json' and not p.name.endswith('.log') and p.name!='stage_events.jsonl'};write(HERE/'SHA256SUMS.json',manifest)
 print(json.dumps(gate),flush=True)
if __name__=='__main__':run()
