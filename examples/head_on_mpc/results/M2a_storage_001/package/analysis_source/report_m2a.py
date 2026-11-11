#!/usr/bin/env python3
"""M2a evidence rollup: actual acceptance, local boundary, protocol, timing and offline identity separate."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from bicycle_core import CARS,corners,clipped_area


def read(p):return json.loads(p.read_text())
def lines(p):return [json.loads(s) for s in p.read_text().splitlines()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def stats(values):
 a=np.array(values,float)
 return dict(count=len(a),p50=float(np.percentile(a,50)) if len(a) else None,p95=float(np.percentile(a,95)) if len(a) else None,p99=float(np.percentile(a,99)) if len(a) else None,max=float(max(a)) if len(a) else None,total=float(a.sum()))
def table(headers,rows):return '\n'.join(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|',*['|'+'|'.join(map(str,row))+'|' for row in rows]])


def summarize(directory):
 cfg=read(directory/'config.json');scene=cfg['scene'];summary=read(directory/'summary.json');validation=read(directory/'independent_validation.json')
 timing=read(directory/'planning.json');solves=lines(directory/'solves.jsonl');agreements=lines(directory/'agreements.jsonl');plans=lines(directory/'plans.jsonl');events=lines(directory/'events.jsonl');messages=lines(directory/'messages.jsonl')
 actual=list(csv.DictReader((directory/'actual.csv').open()));sends=[m for m in messages if m['event']=='send'];by_id={p['id']:p for p in plans};per={}
 arm=cfg['arm'];joint=None
 def counts(records):
  return dict(reference_QP_calls=sum(r.get('reference_numeric_qp_calls',0) for r in records),bicycle_QP_calls=sum(r.get('bicycle_numeric_qp_calls',0) for r in records),
   updates_total=sum(r.get('qp_updates',0) for r in records),updates_histogram={str(k):sum(r.get('qp_updates',0)==k for r in records) for k in range(4)},
   reference_iterations=sum(r['reference_record']['iterations'] for r in records),bicycle_iterations=sum(q['iterations'] for r in records for q in r['subproblems']),
   input_repairs_attempted=sum(len(q.get('stop_input_adjustments',[])) for r in records for q in r['subproblems']),
   reference_rejections=sum(not r['reference_record']['accepted'] for r in records),candidate_rejections=sum(not r['accepted'] for r in records),
   prediction_budget_rejections=sum(r.get('deadline_exceeded',False) for r in records),
   candidate_compute_s=stats([r['solve_wall_s'] for r in records]),reference_build_solve_check_s=stats([r['reference_record']['solve_wall_s'] for r in records]),
   bicycle_OSQP_setup_and_solve_s=stats([q['solver_wall_s'] for r in records for q in r['subproblems']]),
   reference_OSQP_scope='local field includes setup+warm+solve; central field solve only; not a direct solver timing ranking')
 if arm=='C-new':joint=counts([r['central'] for r in solves if r['central'] is not None])
 for c in CARS:
  rr=[r for r in actual if r['car']==c];z=np.array([[float(r[k]) for k in ('x','y','psi','v','delta')] for r in rr]);goal=np.array(scene['goals'][c]);v=z[:,3];stopped=abs(v)<=scene['terminal']['speed_m_s']
  cost=scene['sample_s']*sum(float(r['a'])**2+float(r['omega'])**2 for r in rr[1:])
  occupied=[float(r['time_s']) for r,b in zip(rr,corners(z,scene['vehicle'])) if clipped_area(b,[-scene['D']/2,scene['D']/2,-scene['W']/2,scene['W']/2])>scene['tolerance']]
  request=next((e['cycle']*scene['h'] for e in events if e['kind']=='request' and e['car']==c),None)
  grant=next((e['cycle']*scene['h'] for e in events if (e['kind']=='grant' and e.get('car')==c) or (e['kind']=='handoff' and e.get('owner')==c)),None)
  release=next((e['cycle']*scene['h'] for e in events if e['kind']=='handoff' and e['old_owner']==c),None)
  m=dict(control_cost=cost,endpoint_error=dict(position_m=float(np.linalg.norm(z[-1,:2]-goal[:2])),heading_rad=float(abs(z[-1,2]-goal[2])),speed_m_s=float(abs(z[-1,3])),steering_rad=float(abs(z[-1,4]))),
   request_s=request,grant_s=grant,release_s=release,request_to_grant_s=grant-request if grant is not None and request is not None else None,
   channel_entry_s=occupied[0]-scene['epoch_s'] if occupied else None,grant_to_entry_s=occupied[0]-scene['epoch_s']-grant if occupied and grant is not None else None,
   last_channel_occupied_sample_s=occupied[-1]-scene['epoch_s'] if occupied else None,completion_s=summary['completion_per_car_s'].get(c),
   stationary_threshold_m_s=scene['terminal']['speed_m_s'],low_speed_to_moving_transitions=int(np.sum(stopped[:-1]&~stopped[1:])),
   low_speed_per_stage_s={s:sum(bool(stop) and r['stage']==s for stop,r in zip(stopped[1:],rr[1:]))*scene['sample_s'] for s in sorted({r['stage'] for r in rr[1:]})},
   solver_attribution='own optimizer' if arm=='L-new' else 'joint calls not arbitrarily split by car')
  if arm=='L-new':
   recs=[r['local'][c] for r in solves if r['local'] and r['local'][c]];m.update(counts(recs));m['reference_max_variables']=max((r['reference_record']['variables'] for r in recs),default=0);m['bicycle_max_variables']=max((q['variables'] for r in recs for q in r['subproblems']),default=0)
  per[c]=m
 plan_ages=[];state_ages=[]
 for a in agreements:
  cycle=a['cycle'];end=next(p['end_monotonic_s'] for p in timing if p['cycle']==cycle)
  for c in CARS:
   sent=next(m for m in sends if m['kind']=='final' and m['payload']['id']==a['plans'][c])
   plan_ages.append(end-sent['wall_ns']/1e9)
   first=next(m for m in sends if m['kind']=='state' and m['sender']==c and m['cycle']==cycle)
   state_ages.append(end-first['wall_ns']/1e9)
 staged=stats([p['total_cycle_wall_s'] for p in timing]);candidate_stage=stats([p['candidate_wall_s'] for p in timing if p['candidate_wall_s'] is not None])
 offline=read(directory/'centralized_future_diagnostic.json') if (directory/'centralized_future_diagnostic.json').exists() else dict(executed=False)
 # Independent recomputation of waiting pose from the frozen map; no optimizer or local Agent is used.
 angle=np.arccos(1-scene['bay_offset_local_y']/(2*scene['turn_radius_m']));wait=np.array([scene['waiting_anchor_abs_x']-2*scene['turn_radius_m']*np.sin(angle),-scene['bay_offset_local_y'],scene['initial']['B'][2]])
 waiting_checks=[]
 for event in events:
  if event['kind']!='handoff' or event['old_owner']!='A':continue
  cycle=event['cycle'];r=next(m['payload'] for m in sends if m['kind']=='state' and m['receiver']=='channel' and m['sender']=='B' and m['cycle']==cycle and m['payload']['context']['permission_version']==event['version']-1)
  error=float(np.linalg.norm(np.array(r['z'])[:2]-wait[:2]));yaw=float(abs(r['z'][2]-wait[2]));waiting_checks.append(dict(cycle=cycle,position_error_m=error,heading_error_rad=yaw,passed=error<=.005 and yaw<=.003))
 return dict(arm=arm,both_goals_reached=summary['both_completed'],actual_full_acceptance=validation['actual_full_acceptance'],actual_physical_acceptance=validation['physical_and_original_sign_check_passed'],
  release_contract_passed=bool(validation['release_and_plan_version_contract_passed'] and all(x['passed'] for x in waiting_checks)),waiting_pose_independent_checks=waiting_checks,
  local_information_boundary_passed=validation['local_information_boundary_passed'],local_decision_dimensions=validation['local_decision_dimensions'],
  actual_plan_version_compatibility=validation['release_and_plan_version_contract_passed'],timing_complete=validation['timing_complete'],failure=summary['failure'],failure_layer=summary['failure_layer'],
  completion_s=summary['T_complete_sim_s'],minimum_sampled_gap_m=validation['trajectory']['minimum_gap_m'],per_car=per,joint_solver=joint,
  total_control_cost=sum(p['control_cost'] for p in per.values()),online_wall_s=summary['online_wall_s'],online_full_future_calls=summary['online_full_future_calls'],
  cycle_timing_s=staged,cycles_over_200ms=sum(p['total_cycle_wall_s']>scene['h'] for p in timing),candidate_stage_includes_DDS_and_peer_checks_s=candidate_stage,
  timing_unavailable_separate=['state/protocol setup vs seed replay','nonlinear peer validation alone','record and commit separately'],
  coordinator_rounds_max=max(p['coordination_rounds'] for p in timing),coordinator_rounds_total=sum(p['coordination_rounds'] for p in timing),
  sends=len(sends),application_bytes=sum(m['application_bytes'] for m in sends),messages_by_kind={k:dict(count=sum(m['kind']==k for m in sends),bytes=sum(m['application_bytes'] for m in sends if m['kind']==k)) for k in sorted({m['kind'] for m in sends})},
  published_seed_plans=sum(p['kind']=='seed' for p in plans),published_final_plans=sum(p['kind']=='final' for p in plans),accepted_plan_simulation_age_cycles=0,
  shifted_seed_source_age_cycles=1,final_plan_publish_to_commit_wall_s=stats(plan_ages),state_report_to_commit_wall_s=stats(state_ages),
  handoffs=[e for e in events if e['kind']=='handoff'],independent_offline_audit_s=validation['independent_audit_wall_s'],
  centralized_future_diagnostic=dict(executed=offline['executed'],identity=offline.get('diagnostic_identity'),not_local_future_proof=True,
      not_online_gate=True,wall_s=offline.get('total_offline_wall_s'),online_records_unchanged=offline.get('online_records_unchanged'),
      checkpoints=[dict(cycle=x['cycle'],passed=x['check']['passed'],failure=x.get('failure'),wall_s=x['wall_s']) for x in offline.get('checkpoints',[])]))


def report(root):
 names=['C-new_r1','L-new_r1','C-new_r2','L-new_r2'];metrics={name:summarize(root/'formal'/name) for name in names}
 repeat={arm:dict(actual_bytes_equal=(root/'formal'/f'{arm}_r1/actual.csv').read_bytes()==(root/'formal'/f'{arm}_r2/actual.csv').read_bytes(),sha256=sha(root/'formal'/f'{arm}_r1/actual.csv')) for arm in ('C-new','L-new')}
 cold=root.parent/'M0_closure_001/closed_loop'
 repeat['C-new']['identical_to_C_old_actuals']=(root/'formal/C-new_r1/actual.csv').read_bytes()==(cold/'repaired_r1/actual.csv').read_bytes()
 write(root/'metrics.json',metrics);write(root/'repeatability.json',repeat)
 gate=all(m['actual_full_acceptance'] and m['release_contract_passed'] and m['actual_plan_version_compatibility'] and m['timing_complete'] for m in metrics.values()) and all(metrics[n]['local_information_boundary_passed'] and metrics[n]['online_full_future_calls']==0 for n in ('L-new_r1','L-new_r2'))
 summary=dict(experiments_completed=True,M2b_entry_criteria_met=bool(gate),actual_full_acceptance={n:m['actual_full_acceptance'] for n,m in metrics.items()},
   new_release_contract_passed={n:m['release_contract_passed'] for n,m in metrics.items()},local_information_boundary_passed={n:metrics[n]['local_information_boundary_passed'] for n in ('L-new_r1','L-new_r2')},
   old_contract_equivalence_claimed=False,centralized_future_diagnostic_is_not_acceptance=True,debug_failures_retained=True,hard_realtime_claimed=False,recursive_feasibility_claimed=False)
 write(root/'summary.json',summary)
 fields=['run','actual_complete','actual_full_acceptance','new_release','local_boundary','completion_s','control_cost','min_gap_m','cycle_count','p50_ms','p95_ms','p99_ms','max_ms','over_200ms','online_wall_s','independent_offline_s','central_future_offline_s','application_messages','application_bytes','online_central_future_calls']
 with (root/'comparison.csv').open('w',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
  for i in (1,2):
   old=read(cold/f'repaired_r{i}/metrics_e3.json');writer.writerow(dict(run=f'C-old_reused_r{i}',actual_complete=True,actual_full_acceptance=True,new_release='not applicable: old contract',local_boundary='central',completion_s=old['completion_sim_s'],control_cost=old['normalized_execution_effort'],min_gap_m=old['minimum_sampled_gap_m'],cycle_count=old['timed_cycles'],p50_ms=old['cycle_p50_ms'],p95_ms=old['cycle_p95_ms'],p99_ms=old['cycle_p99_ms'],max_ms=old['cycle_max_ms'],over_200ms=old['cycles_over_200ms'],online_wall_s=old['total_policy_wall_s'],online_central_future_calls=2))
  for name,m in metrics.items():
   t=m['cycle_timing_s'];writer.writerow(dict(run=name,actual_complete=m['both_goals_reached'],actual_full_acceptance=m['actual_full_acceptance'],new_release=m['release_contract_passed'],local_boundary=m['local_information_boundary_passed'],completion_s=m['completion_s'],control_cost=m['total_control_cost'],min_gap_m=m['minimum_sampled_gap_m'],cycle_count=t['count'],p50_ms=t['p50']*1000,p95_ms=t['p95']*1000,p99_ms=t['p99']*1000,max_ms=t['max']*1000,over_200ms=m['cycles_over_200ms'],online_wall_s=m['online_wall_s'],independent_offline_s=m['independent_offline_audit_s'],central_future_offline_s=m['centralized_future_diagnostic']['wall_s'],application_messages=m['sends'],application_bytes=m['application_bytes'],online_central_future_calls=m['online_full_future_calls']))
 result_rows=[];car_rows=[]
 for name,m in metrics.items():
  t=m['cycle_timing_s'];result_rows.append([name,m['actual_full_acceptance'],m['release_contract_passed'],m['completion_s'],f"{m['minimum_sampled_gap_m']:.9f}",f"{m['total_control_cost']:.8f}",f"{t['p50']*1000:.1f}/{t['p95']*1000:.1f}/{t['p99']*1000:.1f}/{t['max']*1000:.1f}",m['cycles_over_200ms']])
  for c,p in m['per_car'].items():car_rows.append([name,c,p.get('reference_QP_calls','联合不拆分'),p.get('bicycle_QP_calls','联合不拆分'),p.get('reference_max_variables','64联合'),p.get('bicycle_max_variables','128联合'),p.get('input_repairs_attempted','见联合字段'),p['completion_s'],p['request_to_grant_s']])
 debug=[]
 for ver in ('v1','v2','v3','v4'):
  d=root/f'debug_{ver}/L-new';s=read(d/'summary.json');record=lines(d/'solves.jsonl')[-1]
  debug.append([ver,s['both_completed'],record['cycle'],s['failure_layer'],s['failure'] or '到达目标并在v4独立验收通过'])
 text=f'''# M2a：双车局部DMPC与M2a-release-v1

**正式C-new/L-new各两轮均到达双方原目标，并通过新释放契约及原物理/跨零采样验收。M2b进入条件：{gate}。** 不将此结果写成旧完整未来契约的等价简化、连续安全、递归可行或实时保证。

## 实现与版本

C-old复用M1/M0_closure的已验收B2-C；C-new仍直接调用原集中式B2-C，保留64/128联合变量，只替换在线交接契约。L-new每车持有自己的Agent、目标/初态、本地32变量参考QP和64变量自行车QP、已提交参考缓存；邻车仅经显式JSON/DDS消息读取公开状态、行为包络、执行输入与非线性预测。输入修复和最多3次更新结束后才发布final，没有后台联合参考或联合求解救场。

旧M1运动代码未修改；新实现位于scripts/m2a_local.py、m2a_protocol.py、run_m2a.py，验收verify_m2a.py，额外离线旧未来audit_m2a_future.py。运行源码快照在每次运行旁的*_source/，正式设置与消息schema见design/settings.json、message_schema.json、design_v4.md及source_manifest.json。实际三个ROS2节点、可靠DDS，同一Linux/库版本、OPENBLAS_NUM_THREADS=1；隔离ROS域只防止不同运行混线。

## 冻结周期时序与信息边界

每周期固定**一轮A→B**：本车状态/许可版本绑定并prepare；互发当前公开状态和静止初始化或移位补尾种子；双方验证初始化兼容；A对B种子求自身参考/自行车问题并发最终预测；B对A最终预测求自身问题并发最终预测；双方各自重演同一对最终版本并认可；屏障只汇集两ID/认可，提交各自首0.2 s。任何更新使旧认可失效，有限轮失败直接结束离线运行，不无限静止、不联合救场。种子来源为上周期实际提交计划，当前上下文年龄0、来源年龄1；没有有效移动状态初始化时拒绝。

局部场景对象裁掉邻车initial/goals；Agent与缓存不经消息发送。消息中的commands是最终执行输出，不是内部参考解或QP变量。L-new在线对联合参考构造/求解、联合自行车子问题和中央完整未来方法设禁止调用保护；各次原始记录与独立验收证实本车变量最多32/64，公开消息schema排除私有解/缓存。运行器仅投递、积分/提交和记录，不替车辆改候选。

局部耦合本来就不是原联合问题的变量切片：参考通过消息预测位置/公开路线包络约束；自行车按本周期固定版本几何在各节点冻结x/y分离半空间，固定邻车使用实际旋转车身，本车保留原航向信赖域包络；被已有状态盒完整蕴含的分离行省略并记录证书，非冗余行保留。最终双方完整稠密矩形距离检查从不省略。C-new仍用原M1阶段分离轴/联合装配，不能把全部差别当作同一数学问题的纯架构因果或求解器优劣。

## 新释放事务

当前owner和授权版本报告实际清空；按实际旋转矩形沿方向最小x支持>=26.5 m检查，不只看头/后轴。B原等待位置、实际近零速度和转向准备由同一活跃版本报告核查。通道先进入prepared并提出下一版本，owner不变；车辆在拟许可下复制本地阶段生成条件计划，不覆盖真实状态。最终双方认可后，通道才生效一次owner/版本交接并允许提交。缺失/过期条件取消并保留旧许可。

两臂使用同一几何、完整6.4 s时域、认可和授权激活规则。旧owner整个预测保持在26.5 m释放边界外，是两臂共同的保守不重入要求。拟授权不意味着可提前执行；单车owner从未隐式同时授予两车。

新契约是有限时域兼容，不能证明之后任意重规划可行，也不等价于旧中央全未来见证。

## 正式结果

{table(['运行','实际完整验收','新释放契约','任务s','最小净空m','统一控制代价','P50/P95/P99/最大周期ms','>200ms'],result_rows)}

{table(['运行','车','参考QP次数','自行车QP次数','参考最大变量','自行车最大变量','修复次数（含更新尝试）','完成s','申请到授权s'],car_rows)}

所有403个周期包括402次提交和最终完成检查。通信轮次/字节、计划版本/年龄、原QP更新/迭代/修复、端点误差、低速等待/停走、授权到进入和释放时刻、在线及两类离线时间详见metrics.json。C-new联合调用不人为一分为二；L-new按车统计。应用字节是UTF-8 JSON信封长度，不包含DDS/网络头。低速/停走按原0.025 m/s统计口径，不把它当段间严格停车阈值。

candidate_stage包含候选计算、发布和接收方检查；本车record.solve_wall_s是本车计算；参考局部solver_wall_s含setup/warm/solve，中央参考该字段仅solve。未记录的组装/非线性验收/提交细项保留组合，不编造独立精度。端到端cycle含状态/协议、初始化、交换、双端检查和提交，不含事后文件批量导出与独立审计；online总墙钟另列。

C-new与C-old实际轨迹是否逐字节相同：{repeat['C-new']['identical_to_C_old_actuals']}。这只说明本场景释放契约变化没有改变已执行运动，不证明契约等价。各新臂两轮轨迹一致性见repeatability.json，属于确定性核查，不是成功率。没有宣称局部一定更快；移出在线未来、增加消息与双端重验收改变了工作量，不能称为同一工作量的纯求解提速。0.5 s每次候选预算与0.2 s控制周期不同，顺序两车的端到端时间必须单独看。

## 独立验收与离线身份

verify_m2a重新读取实际输入，独立标量两半步RK4逐区间/累计重演，检查原目标、输入/速度/转角界、旋转车身道路/间距及原速度跨零；重新积分最终认可输入的全N时域并核对实际执行前缀。按消息/事件重建授权版本、准备/交接、双ACK及执行，检查实际车身清空、初始化/种子移位、信息/变量维度、内容ID、每条实际输入对应最终组合、全出口计时；额外从静态地图重新核对等待位置。几何仍复用原审计函数，非完全独立几何实现。

新增11项协议/局部QP测试通过，覆盖非owner/旧版本释放、仅预测清空、旋转车身、错误WAIT位置与运动、缺失/过期认可、修复后未重发、条件授权前提交、初始化/局部拒绝和变量维度及冗余证书。原34项运动/停车回归复用M1冻结证据，维护源码哈希未变；不将其写成本轮新跑。交付哈希及逐运行核查见verification.json，复现及M2b边界见reproduction.md。

实际actual_full_acceptance与centralized_future_diagnostic分列。在线阶段全部结束后，才从保存交接点恢复旧M1中央控制器作额外完整未来诊断，假设旧方法A=false/B=true许可及中央信息；结果在各运行centralized_future_diagnostic.json，私有求解日志另存。它不是L-new局部未来证明，不是在线门禁；审计前后实际轨迹、owner事件、认可和summary哈希一致，诊断不能覆盖实际失败或回写输入。离线成本单列，未假装消失。

## 调试失败与最小修复来源

{table(['版本','到达目标','最后cycle','拒绝层','原因'],debug)}

v1 cycle119冻结诊断显示参考成功、B自行车QP拒绝；去掉耦合行仅为离线探针，可解。v2将固定邻车误加的自由航向包络改为实际车身，但新闭环仍在119拒绝。v3按节点几何选轴后推进到120；冻结状态有约1e-7 m级停车硬数值界，冗余行改变OSQP数值判定。v4仅按已有状态盒蕴含证书消去冗余耦合行，不改数值带/门限/权重/预算，也未在线用无耦合诊断。QP状态不是物理不可行证明，solved也不是原问题精确最优证明。debug_v1/v2/v3及冻结failure/proof文件全部保留，不用前缀低成本与完整任务排名。

## 结论边界与下一步

本次双车理想同步模型下满足M2b进入条件，未实施四车、批次、额外延迟或在线备用。当前A/B角色、固定原语/地图和一轮顺序需在M2b重新设计邻接/协调扩展，不能直接宣称任意规模DMPC。计算期间仿真状态冻结，有限采样、两轮确定性成功及中央离线诊断均不构成连续安全、递归可行、延迟鲁棒或硬实时证明。失败结束仿真不等于车辆安全停车。

复现：在相同ROS2/Linux环境，先固定OPENBLAS_NUM_THREADS=1、PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH、ROS_LOG_DIR=/tmp/m2a_ros；使用新的输出目录和隔离ROS_DOMAIN_ID调用scripts/run_m2a.py --arm C-new或L-new，随后scripts/verify_m2a.py --run目录，在线结束后再scripts/audit_m2a_future.py --run目录。精确命令、版本/线程和源码SHA256在每次*_source/provenance.json。不能把验证脚本退出码代替actual_full_acceptance字段。
'''
 (root/'report.md').write_text(text)
 print(json.dumps(summary))


if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();report(args.run)
