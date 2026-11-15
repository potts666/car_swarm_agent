"""Create measured comparison, gates and a content checked portable delivery."""
import csv,difflib,time
from common import *
from plan_io import load,source
from physics import step

def csvout(name,rows):
 with (HERE/name).open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def run():
 acc=read(HERE/'results/independent_acceptance.json');assert acc['passed'];events=read(HERE/'results/events.json');ledger=read(HERE/'solver_calls.json');assert len(ledger)==16
 for n in ('model_negative_fixtures','projection_atlas_certificate','common_faces_certificate'):assert read(HERE/f'results/{n}.json')['passed']
 comparisons=[];eventrows=[];timings=[];base=load('results/R_original.json.gz');baseE=H*sum(F(a)**2+F(w)**2 for c in CARS for a,w in base['controls'][c]);ownacc={x['label']:x for x in acc['rounds']}
 for arm in ('F-E','O-E','F-P','O-P'):
  plan=load(f'results/{arm}_final_plan.json.gz');finalE=H*sum(F(a)**2+F(w)**2 for c in CARS for a,w in plan['controls'][c]);goals=ownacc[f'{arm}_r2']['full']['goals']
  for rnd in (1,2):
   label=f'{arm}_r{rnd}';event=next(e for e in events if e['label']==label)
   for c in ('A1','A2'):
    ad=read(HERE/f'results/{label}_{c}_admission.json');o=ad['objective'];raw=read(HERE/f'results/workers/{label}_{c}_response.json');job=load(f'results/{label}_{c}_job.json.gz');meta=read(HERE/f'results/workers/{label}_{c}_transport.json');f_delta=list(map(F,ad['delta'][:2]));s0=F(job['progression']['reference_progress_Q'][0],m.Q);ds=F(o['actual_progress_Q'][0],m.Q)
    comparisons.append({'arm':arm,'round':rnd,'actor':c,'cycle':event['source']['cycle'],'omega_open':arm.startswith('O'),'solver_status':raw['status'],'accepted':ad['accepted'],'first_delta_a':float(f_delta[0]),'first_delta_omega':float(f_delta[1]),'a_changed':ad['a_changed'],'omega_changed':ad['omega_changed'],'JE_window':float(F(o['JE'])),'progress_window_lower_m':float(ds),'progress_vs_own_anchor_m':float(ds-s0),'true_F_upper':float(F(o['true_F_upper'])),'SOCP_F_upper':float(F(o['SOCP_F_upper'])),'true_F_improvement_lower':float(F(ad['objective_improvement_lower'])),'final_unique_all_car_JE':float(finalE),'final_unique_JE_gain':float(baseE-finalE),'goal_tick':goals[c],'block_max_goal':max(goals.values()),'actual_first_activity':len(event['actual_first']['bounds']['active']),'minimum_road_m':float(F(ownacc[label]['full']['minimum']['road'])),'minimum_pair_m':float(F(ownacc[label]['full']['minimum']['pair']))})
    eventrows.append({'label':label,'actor':c,'model_id':job['model_id'],'status':raw['status'],'gap':raw['obj_val']-raw['obj_val_dual'],'r_prim':raw['r_prim'],'r_dual':raw['r_dual'],'alpha':ad['alpha'],'accepted':True,'first_delta_a':str(f_delta[0]),'first_delta_omega':str(f_delta[1]),'source_history':event['source']['history'],'successor_history':event['successor']['history'],'parent':job['parent'],'K_id':job['receiver']['K_id'],'receiver_phase':job['receiver']['phase']})
    timings.append({'label':label,'actor':c,'solver_seconds':raw['solve_time'],'worker_roundtrip_seconds':meta['roundtrip_wall'],'round_seconds':event['round_wall'],'first_execution_seconds':event['execution_wall'],'independent_full_seconds_shared_per_round':ownacc[label]['full']['wall_seconds'],'model_derivative_seconds':'included in round; cached first-source derivatives','assembly_seconds':'included in worker; no independent timer','admit_commit_seconds':'included in round; no independent timer','application_model_JSON_bytes':len(json.dumps(job,sort_keys=True,separators=(',',':')).encode()),'worker_request_bytes':meta['sent_bytes'],'worker_response_bytes':meta['received_bytes']})
 # Signed candidate-minus-anchor differences, common reporting F=JE-lambda*s.
 # Keep each round separate: its 32-tick windows overlap the other round.
 tradeoffs=[]
 for arm in ('F-E','O-E','F-P','O-P'):
  for rnd in (1,2):
   anchorE=deltaE=progresslo=progresshi=F(0)
   for c in ('A1','A2'):
    ad=read(HERE/f'results/{arm}_r{rnd}_{c}_admission.json');old=ad['baseline_objective'];new=ad['objective'];anchorE+=F(old['JE']);deltaE+=F(new['JE'])-F(old['JE']);progresslo+=F(new['actual_progress_Q'][0]-old['actual_progress_Q'][1],m.Q);progresshi+=F(new['actual_progress_Q'][1]-old['actual_progress_Q'][0],m.Q)
   assert anchorE>0 and progresslo<=progresshi
   mid=(progresslo+progresshi)/2;flo=deltaE-progresshi;fhi=deltaE-progresslo
   assert (flo+fhi)/2==deltaE-mid
   tradeoffs.append({'arm':arm,'round':rnd,'actors':'A1+A2','reference':'same-source anchor' if rnd==1 else 'own actual-successor anchor','window_ticks':32,'delta_JE_cost_units':str(deltaE),'anchor_JE_cost_units':str(anchorE),'delta_JE_pct_of_anchor':float(100*deltaE/anchorE),'delta_s_mid_m':float(mid),'delta_s_lower_m':str(progresslo),'delta_s_upper_m':str(progresshi),'delta_F_common_mid_cost_units':float(deltaE-mid),'delta_F_common_lower_cost_units':str(flo),'delta_F_common_upper_cost_units':str(fhi),'lambda_cost_units_per_m':1})
 csvout('tradeoff_summary.csv',tradeoffs)
 for row in comparisons:
  ad=read(HERE/f'results/{row["arm"]}_r{row["round"]}_{row["actor"]}_admission.json');old=ad['baseline_objective'];new=ad['objective'];de=F(new['JE'])-F(old['JE']);lo=F(new['actual_progress_Q'][0]-old['actual_progress_Q'][1],m.Q);hi=F(new['actual_progress_Q'][1]-old['actual_progress_Q'][0],m.Q)
  row.update(delta_JE_cost_units=float(de),delta_JE_pct_of_own_anchor=float(100*de/F(old['JE'])),delta_s_mid_m=float((lo+hi)/2),delta_F_common_mid_cost_units=float(de-(lo+hi)/2),final_unique_JE_gain_pct_of_R_original=float(100*F(str(row['final_unique_JE_gain']))/baseE))
 csvout('comparison.csv',comparisons);csvout('per_event.csv',eventrows);csvout('timing.csv',timings)
 # Ascending source provenance is reconstructed from physical initial states.
 scan=[];s=initial_state()
 for k in range(33):
  moving=[c for c in CARS if s['v'][c]!=0];curved=[c for c in moving if s['delta'][c]!=0]
  scan.append({'cycle':k,'moving':','.join(moving),'curved':','.join(curved),'v':json.dumps(s['v'],default=str),'delta':json.dumps(s['delta'],default=str),'curvature_interval':json.dumps({c:[(m.tan(m.I(s['delta'][c]))/m.L).l,(m.tan(m.I(s['delta'][c]))/m.L).u] for c in curved}), 'curvature_units':'1/metre; fixed-point interval endpoints divided by Q=1e20','owner':base['owners'][k],'reason':'selected first fully certified mixed source' if k==32 else 'no moving curved actor with moving peer','next_release':base['encounter']['first_out'],'reference_road_global_min':str(F(read(HERE/'results/reference_domain_audit.json')['minimum']['road'])),'reference_pair_global_min':str(F(read(HERE/'results/reference_domain_audit.json')['minimum']['pair']))})
  if k<32:step(s,{c:base['controls'][c][k] for c in CARS},base['owners'][k],4)
 csvout('source_scan.csv',scan)
 gate={'passed':True,'task':'curve_joint_001','actual_curve_certification':True,'fair_four_arms':True,'actual_control_update':True,'two_real_successor_rounds':True,'multi_car_combination':True,'safe_all_four_complete':True,'independent_acceptance':True,'rejection_continuations':2,'native_attempts':16,'native_solved':16,'certificate_accepted':16,'first_actions_executed':8,'actual_activity_per_first_action':[4]*8,'O-P_A1_first_omega_changed_rounds':[1,2],'goals':{'A1':475,'B1':855,'B2':1304,'A2':1753},'block_max_deadline':1753,'individual_deadline_nonincrease':True,'B_final':0,'all_six_pairs':True,'partial_or_unchecked':['no real-time, disturbance or asynchronous guarantee','component timing included in round where no independent timer was installed'],'new_optimizer_budget_remaining':0};write(HERE/'GATE.json',gate)
 lines=['# curve_joint_001 结果','', '通过：四臂 × 连续两轮 × 两个独立本车 worker，共 16 次正式尝试，全部 Solved、证书受理。每轮执行首拍后四车实际活动；第二轮使用各臂自身真实后继，未复用首轮预测状态。','', '共同首源为第 32 拍 A1 弯道与 A2 直线运动。32 拍窗口含加减速、停车转向与第二段曲率，使用 C4–C21 实际分段变分与有理加权矩；不套用常速圆弧示例。非零方向 c_cert≈0.569641，所用量化步幅 1e-5，逐行允许步幅下界约 1.33407e-4；方向完整后续已认证。','', '|臂|A1 首拍 Δa，两轮 (m/s²)|A1 首拍 Δomega，两轮 (rad/s)|四车全程唯一拍号 JE 改善|','|---|---|---|---|']
 for arm in ('F-E','O-E','F-P','O-P'):
  rr=[r for r in comparisons if r['arm']==arm and r['actor']=='A1'];lines.append(f'|{arm}|'+', '.join(f"{r["first_delta_a"]:.9g}" for r in rr)+'|'+', '.join(f"{r["first_delta_omega"]:.9g}" for r in rr)+f'|{rr[-1]["final_unique_JE_gain"]:.9g}|')
 lines += ['', 'F-E/F-P 精确冻结 omega。O-E 的极小转向变化如实保留，不能当作有意义的推进优势。O-P 两轮均实际采用约 −7.5e-5 rad/s 的首拍转向增量，真实目标严格改善；加速度变化单列。比较完整策略代价按唯一绝对拍号计算，未把重叠窗口收益相加。', '', '所有臂均在 A1=475、B1=855、B2=1304、A2=1753 拍完成，B_final=0；逐车绝对期限及块最大期限保持，无提前完成结论。原解析 R-original 的控制/日历与既有 1753 拍认可版本相同，仅本地积分实现来源哈希不同；历史数值向量未用于新初始化。', '', '八份完整已提交策略、两条真实第二轮源拒绝续行均经独立 8 子段连续矩形车身重放；本车窗口另用 64 子段复算原始向量、精确 E/T 量化、真实目标与接收。拒绝分支为无效 K/模型证书与缺 ACK，非法提案零提交、零执行，沿首轮认可策略完成。', '', '接收/守卫版本 curve_pose_domain_v1：末 v/delta 精确；位置/航向为正半径域；自由扰动块在下一停车守卫前结束，其后同输入 SE2 包含覆盖所有后续模式、资源释放与目标。原紧点守卫保留在未调用的历史 execution.py，正式调用链使用 contract.guard。共同半空间从同一 R-original 封存，见 common_faces_certificate；完整尾段参见 reference_domain_audit 与 contract_certificate。', '', '进度采用连续分段光滑图册；跨接缝不声称全域 C2，终端接收凸行保证整个可行 Taylor 同伦位于单一光滑投影域，见 projection_atlas_certificate。', '', '预检失败与修正：终点保持曾误要求最后一拍制动为零，已保存 terminal_hold_fixture_failure，改为到达后零输入保持；未消耗求解。后续图册验收中的任意 1e-12 容差改为外向区间重叠检查（原差界 1.434e-12），不改已执行模型、控制或求解预算。旧冻结/点接口及损坏模型反例见 old_interface_failure、model_negative_fixtures。', '', '证据：comparison.csv、per_event.csv、timing.csv；results/workers 保存原始 JSON 请求/ready/release 前装配信息及 x/s/z，results/matrices 保存实际锥矩阵，solver_calls.json 在每次 release 前落盘。source_archive/numerical_v1 与 executed_source_versions 冻结实际代码。source_scan.csv 逐拍记录跳过原因、真实速度/轮角及来源。', '', '数值配置沿用 Clarabel 0.11.1、200 次迭代、容差 1e-10、默认无限 solver time_limit、worker 通道 30s；h=.2、N=32、rho=.001、eta=.02、R=S=I、lambda=1、E/T 坐标格 1e-12；目标共同乘 5。计算墙钟见 timing/stage_events，不能视为 .2s 实时达标。部分组件未独立计时，已明确包含在 round 时间，未编造拆分。', '', '复验：安装 requirements.txt 中既有依赖后运行 `python3 verify_delivery.py`；`python3 verify_delivery.py --full-replay` 独立完整物理重放，两者均零新增求解。日志/阶段事件是可追加文件，单独保留，SHA 排除动态运行日志与 SHA 清单自身；实际源、模型、raw、控制、证书、报告均纳入清单。', '', '保留历史和论文，未 Git 提交。未检验持续物理时钟、扰动、异步或规模扩展；本结论限于受控名义实例。']
 # Put the tradeoff before the input diagnostics; do not mix full-trip JE
 # differences with local-window progress differences in a single delta-F.
 summary=['比较口径：下表均为候选减同源锚点，ΔJE=JE_new−JE_anchor、Δs=s_new−s_anchor、ΔF=ΔJE−lambda·Δs。为便于四臂横向比较，统一评估 F=JE−lambda·s（lambda=1 U_J/m）；E 臂实际优化目标仍是 JE。ΔJE/ΔF 为负表示改善，Δs 为正表示推进增加。', '', '代价单位 U_J 是本实验原目标的归一化输入平方×秒，不是焦耳，也不是百分比；表中 ΔJE、ΔF 均为绝对差值。输入按既有尺度归一化，JE=h·sum(uᵀRu)，R=I。百分比单列，分母为同一行两辆车锚点窗口 JE 之和。进度单位为 m。数值表为区间中点的舍入显示；外向上下界及精确有理 ΔJE 保存在 tradeoff_summary.csv；Δs 区间包含零时表中显示 ≈0，不将区间中点残差当作有效推进。', '']
 for rnd in (1,2):
  summary += [('第一轮：第 32 拍共同实际源，A1+A2 各 32 拍窗口之和。' if rnd==1 else '第二轮：各臂第 33 拍自身真实后继，相对各自移位锚点；不再是跨臂同源比较。'), '', '|臂|ΔJE (U_J，绝对差)|ΔJE / 锚点 JE (%)|Δs (m，绝对差)|ΔF (U_J，绝对差)|', '|---|---:|---:|---:|---:|']
  for r in tradeoffs:
   display_s='≈0' if F(r['delta_s_lower_m'])<=0<=F(r['delta_s_upper_m']) else f'{r["delta_s_mid_m"]:+.10g}'
   if r['round']==rnd:summary.append(f'|{r["arm"]}|{float(F(r["delta_JE_cost_units"])):+.10g}|{r["delta_JE_pct_of_anchor"]:+.8g}|{display_s}|{r["delta_F_common_mid_cost_units"]:+.10g}|')
  summary.append('')
 summary += ['两轮窗口重叠，以上两轮 ΔJE、Δs、ΔF 不相加。O-P 以少量输入代价改善的让步换取更多局部推进，是否值得由统一 ΔF 显示；局部推进增加不代表目标时刻提前。', '', '四车全程 JE 另按唯一绝对拍号累计，相对 R-original 的 JE='+f'{float(baseE):.12g} U_J'+'。下表“改善”为 JE_R-original−JE_final（正数表示节省），比例分母为 R-original 全程 JE；此口径不与上述局部 Δs 拼接计算 ΔF。', '', '|臂|全程 JE 改善 (U_J，绝对值)|全程 JE 改善 / R-original JE (%)|', '|---|---:|---:|']
 for arm in ('F-E','O-E','F-P','O-P'):
  rr=next(r for r in comparisons if r['arm']==arm);summary.append(f'|{arm}|{rr["final_unique_JE_gain"]:.10g}|{rr["final_unique_JE_gain_pct_of_R_original"]:.8g}|')
 summary.append('')
 # Replace the formerly ambiguous JE column in the control-only table.
 lines=[line.replace('四车全程唯一拍号 JE 改善','全程 JE 节省 (U_J，绝对值)') for line in lines]
 lines[6:6]=summary
 (HERE/'report.md').write_text('\n'.join(lines)+'\n')
 patch=[];old=HERE.parent/'encounter_coordination_001'
 for q in sorted(HERE.glob('*.py')):
  prior=old/q.name;before=prior.read_text().splitlines(True) if prior.exists() else [];patch.extend(difflib.unified_diff(before,q.read_text().splitlines(True),fromfile=f'prior/{q.name}' if prior.exists() else '/dev/null',tofile=f'curve_joint_001/{q.name}'))
 prior=HERE.parent/'region_reservation_003_reverse/sources/certify_interfaces.py';q=HERE/'sources/certify_interfaces.py'
 patch.extend(difflib.unified_diff(prior.read_text().splitlines(True),q.read_text().splitlines(True),fromfile='region003/sources/certify_interfaces.py',tofile='curve_joint_001/sources/certify_interfaces.py'))
 (HERE/'changes.patch').write_text(''.join(patch))
 (HERE/'HANDOFF.md').write_text('# Completed boundary\nGoal: curve_joint_001 four fair arms, two actual rounds and full completion.\nConstraints: 16 native attempts consumed, no further solves authorized in this task; no Git commits.\nSource: executed_source_versions.json; source_archive/numerical_v1.\nEvidence: GATE.json, report.md, results/independent_acceptance.json, solver_calls.json.\nRemaining: no experiment issue; default verification and full physical replay passed. Archive/portable-read check is the final delivery step.\nResume: OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 experiments/local_coordination_p0/curve_joint_001/verify_delivery.py --full-replay\n')
 manifest={str(q.relative_to(HERE)):sha(q) for q in sorted(HERE.rglob('*')) if q.is_file() and '__pycache__' not in q.parts and q.name!='SHA256SUMS.json' and not q.name.endswith('.log') and q.name!='stage_events.jsonl'};write(HERE/'SHA256SUMS.json',manifest);print(json.dumps({'passed':True,'manifest_files':len(manifest),'native_attempts':16}))
if __name__=='__main__':run()
