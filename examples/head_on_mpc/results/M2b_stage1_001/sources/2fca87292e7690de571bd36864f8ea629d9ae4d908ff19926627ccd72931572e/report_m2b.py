#!/usr/bin/env python3
"""Four-car physical, protocol, numerical, timing and storage outcomes, without success relabeling."""
import argparse
import csv
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
from m2b_recording import lines
from m2b_scene import CARS,SIGMA
from run_entry_bicycle import write


def stats(values):
 a=np.asarray(values,float);return dict(count=len(a),p50=float(np.percentile(a,50)) if len(a) else None,p95=float(np.percentile(a,95)) if len(a) else None,p99=float(np.percentile(a,99)) if len(a) else None,max=float(a.max()) if len(a) else None,total=float(a.sum()))

def metrics(directory):
 cfg=json.loads((directory/'config.json').read_text());s=cfg['scene'];summary=json.loads((directory/'summary.json').read_text());validation=json.loads((directory/'independent_validation.json').read_text()) if (directory/'independent_validation.json').exists() else {};actual=lines(directory/'actual.jsonl');solves=lines(directory/'solves.jsonl');timing=lines(directory/'planning.jsonl');messages=lines(directory/'messages.jsonl');sends=[m for m in messages if m['event']=='send'];per={};records=[]
 for c in CARS:
  z=np.array([s['initial'][c]]+[q for a in actual for q in a['states'][c][1:]]);v=z[:,3];goal=np.array(s['goals'][c]);per[c]=dict(completion_s=summary['completion_per_car_s'].get(c),goal_position_error_m=float(np.linalg.norm(z[-1,:2]-goal[:2])),goal_heading_error_rad=float(abs(np.arctan2(np.sin(z[-1,2]-goal[2]),np.cos(z[-1,2]-goal[2])))),goal_speed_m_s=float(abs(z[-1,3])),goal_steering_rad=float(abs(z[-1,4])),distance_m=float(np.linalg.norm(np.diff(z[:,:2],axis=0),axis=1).sum()),control_cost=s['h']*sum(a['controls'][c][0]**2+a['controls'][c][1]**2 for a in actual),low_speed_threshold_m_s=s['terminal']['speed_m_s'],low_speed_to_moving_transitions=int(np.sum((abs(v[:-1])<=s['terminal']['speed_m_s'])&(abs(v[1:])>s['terminal']['speed_m_s']))),queue_advance_cycles=sum(a['reports'][c]['mode']=='MOVE' and a['reports'][c]['index']>0 and a['controls'][c][2]=='QUEUE_ADVANCE' for a in actual),low_speed_per_mode_s={mode:sum(abs(a['states'][c][-1][3])<=s['terminal']['speed_m_s'] and a['reports'][c]['mode']==mode for a in actual)*s['h'] for mode in sorted({a['reports'][c]['mode'] for a in actual})},**validation.get('waiting',{}).get(c,{}))
  if c in validation.get('first_protection_entry_s',{}):per[c]['grant_to_protection_entry_s']=validation['first_protection_entry_s'][c]-per[c].get('grant_s',0)
 if cfg['arm']=='local':
  for c in CARS:records.append((c,[r['local'][c] for r in solves if r['local'].get(c)]))
 else:records=[('joint',[r['central'] for r in solves if r['central']])]
 numerical={}
 for name,rr in records:
  numerical[name]=dict(reference_QP_calls=sum(r['reference_numeric_qp_calls'] for r in rr),bicycle_QP_calls=sum(r['bicycle_numeric_qp_calls'] for r in rr),reference_variables_max=max((r['reference_record']['variables'] for r in rr),default=0),bicycle_variables_max=max((q['variables'] for r in rr for q in r['subproblems']),default=0),updates=sum(r['qp_updates'] for r in rr),repairs=sum(len(q['stop_input_adjustments']) for r in rr for q in r['subproblems']),reference_iterations=sum(r['reference_record']['iterations'] for r in rr),bicycle_iterations=sum(q['iterations'] for r in rr for q in r['subproblems']),compute_s=stats([r['solve_wall_s'] for r in rr]),reference_total_s=stats([r['reference_record']['solve_wall_s'] for r in rr]),bicycle_OSQP_s=stats([q['solver_wall_s'] for r in rr for q in r['subproblems']]),model_error_max={key:max((e[key] for r in rr for e in r['mapping_errors'].values()),default=None) for key in ('position_m','heading_rad','speed_m_s','steering_rad')},failed_candidates=sum(not r['accepted'] for r in rr))
 stored=sum(p.stat().st_size for p in directory.rglob('*') if p.is_file());dedup_raw=sum(len(gzip.decompress(p.read_bytes())) for p in directory.glob('*.gz'));msg_raw=sum(len((json.dumps(m,ensure_ascii=False,allow_nan=False)+'\n').encode()) for m in messages);msg_dedup=len(gzip.decompress((directory/'messages.jsonl.gz').read_bytes()));other=sum(p.stat().st_size for p in directory.glob('*.json'));seed_saved=json.loads((directory/'recording_stats.json').read_text()).get('seed_prefix_raw_bytes_saved',0) if (directory/'recording_stats.json').exists() else 0
 phase={};plan_age=[];state_age=[];send_bycycle={}
 for m in sends:send_bycycle.setdefault(m['cycle'],[]).append(m)
 for a in actual:
  ms=send_bycycle[a['cycle']];end=timing[a['cycle']]['end_monotonic_s'];final=[m['wall_ns']/1e9 for m in ms if m['kind']=='final'];commit=[m['wall_ns']/1e9 for m in ms if m['kind']=='commit'];state=[m['wall_ns']/1e9 for m in ms if m['kind']=='snapshot']
  plan_age.extend(end-t for t in final);state_age.extend(end-t for t in state)
  if final and commit:phase.setdefault('last_final_publish_to_first_commit_s',[]).append(min(commit)-max(final))
 return dict(arm=cfg['arm'],policy=cfg['policy'],all_completed=summary['all_completed'],actual_full_acceptance=validation.get('actual_full_acceptance'),actual_prefix_acceptance=validation.get('actual_prefix_acceptance'),local_information_boundary_passed=validation.get('local_information_boundary_passed'),completion_s=summary['completion_sim_s'],per_car=per,numerical=numerical,min_gap_m=validation.get('physical',{}).get('minimum_gap_m'),failure=summary['failure'],failure_layer=summary['failure_layer'],violations=validation.get('violations'),cycle_s=stats([r['total_cycle_wall_s'] for r in timing]),cycles_over_200ms=sum(r['total_cycle_wall_s']>s['h'] for r in timing),candidate_stage_s=stats([r['candidate_wall_s'] for r in timing if r['candidate_wall_s'] is not None]),exchange_and_ack_s={k:stats(v) for k,v in phase.items()},application_messages=len(sends),communication_by_sender={c:dict(messages=sum(m['sender']==c for m in sends),application_bytes=sum(m['application_bytes'] for m in sends if m['sender']==c)) for c in (*CARS,'channel')},maximum_granted_request_wait_s=max((v.get('request_to_grant_s',0) for v in per.values()),default=0),application_bytes=sum(m['application_bytes'] for m in sends),communication_rounds=sum(r['coordination_rounds'] for r in timing),plan_sim_age_cycles=0,shift_source_age_cycles=1,plan_publish_to_cycle_end_s=stats(plan_age),state_snapshot_to_cycle_end_s=stats(state_age),permission_order=[e['owner'] for e in validation.get('handoffs',[]) if e['owner']],idle_time_s=validation.get('idle_time_s'),online_wall_s=summary['online_wall_s'],export_wall_s=summary.get('export_wall_s'),offline_audit_wall_s=validation.get('independent_audit_wall_s'),storage=dict(logical_full_raw_bytes=dedup_raw+msg_raw-msg_dedup+other+seed_saved,logical_dedup_raw_bytes=dedup_raw+other,stored_bytes=stored,soft_30MiB_met=stored<=30*2**20,largest=[dict(name=p.name,bytes=p.stat().st_size) for p in sorted(directory.iterdir(),key=lambda p:p.stat().st_size,reverse=True)[:5] if p.is_file()]),actual_sha256=hashlib.sha256((directory/'actual.jsonl.gz').read_bytes()).hexdigest())


def report(root):
 formal={}
 for group in ('C-FIFO','L-FIFO','L-ALT'):
  for p in sorted((root/group).glob('r*/summary.json')):
   if (p.parent/'independent_validation.json').exists():formal[f'{group}/{p.parent.name}']=metrics(p.parent)
 write(root/'metrics.json',formal)
 fields=['run','completed','actual_full_acceptance','prefix_acceptance','completion_s','min_gap_m','cost','cycle_p50_ms','cycle_p95_ms','cycle_p99_ms','cycle_max_ms','over_200ms','application_bytes','stored_MiB','failure']
 with (root/'comparison.csv').open('w',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
  for name,m in formal.items():writer.writerow(dict(run=name,completed=m['all_completed'],actual_full_acceptance=m['actual_full_acceptance'],prefix_acceptance=m['actual_prefix_acceptance'],completion_s=m['completion_s'],min_gap_m=m['min_gap_m'],cost=sum(c['control_cost'] for c in m['per_car'].values()),**{f'cycle_{q}_ms':m['cycle_s'][q]*1000 for q in ('p50','p95','p99','max')},over_200ms=m['cycles_over_200ms'],application_bytes=m['application_bytes'],stored_MiB=m['storage']['stored_bytes']/2**20,failure=m['failure']))
 complete=len(formal)==6 and all(m['actual_full_acceptance'] for m in formal.values());repeat={}
 for group in ('C-FIFO','L-FIFO','L-ALT'):
  rr=[m for name,m in formal.items() if name.startswith(group+'/')]
  repeat[group]=dict(runs=len(rr),actual_bytes_equal=len(rr)==2 and rr[0]['actual_sha256']==rr[1]['actual_sha256'],permission_orders=[m['permission_order'] for m in rr])
 write(root/'repeatability.json',repeat);write(root/'summary.json',dict(six_formal_runs_audited=len(formal)==6,all_six_full_acceptance=complete,policies_same_orders=repeat['L-FIFO']['permission_orders']==repeat['L-ALT']['permission_orders'] if repeat['L-ALT']['runs']==2 else None,local_task_passed=all(formal.get('L-FIFO/r'+str(i),{}).get('actual_full_acceptance',False) for i in (1,2)),continuous_safety_proven=False,recursive_feasibility_proven=False,hard_realtime_proven=False))
 rows='\n'.join(f"|{n}|{m['actual_full_acceptance']}|{m['completion_s']}|{m['min_gap_m']}|{m['cycle_s']['max']*1000:.1f}|{m['storage']['stored_bytes']/2**20:.2f}|{m['permission_order']}|{m['failure']}|" for n,m in formal.items())
 text=f'''# M2b：四车理想同步通信真实实验

六次正式运行完整验收全通过：**{complete}**。逐轮结果见comparison.csv、metrics.json和各运行independent_validation.json；失败不改报成功，未运行的组不算通过。

|运行|完整验收|完成s|最小采样净空m|最大周期ms|工件MiB|授权顺序|失败|
|---|---|---|---|---|---|---|---|
{rows}

固定初态/目标严格采用任务书；中央窄通道不变，外侧道路显式扩至abs(x)=52 m。等待点为左(-22,+3)/(-28,+3)，右(+22,-3)/(+28,-3)。每车的本地曲率原语、转角停车转换及最终车位均独立；同侧尾车先等前车实际到队首再合流，前车旋转车身离开队首区域后才补位。DONE车一直保留真实车身，全部6车对在321点最终预测和全部实际样本中检查。

控制沿用五状态、N32/h0.2/sample0.02、原目标/物理/模型误差阈值、OSQP参数、原权重、单步导数与最多3次更新。保留停车首输入修复并全重验。调试发现上一周期允许形成后续一控制步停车越过路径数值界的状态；新增首输入后的必要停车距离s1+h*v1/2不超过段端点的可达性行（数值带补偿，不放宽任何门限）。只作用于实际即将提交前缀，未把状态/速度覆盖为零。详细变更与调试反例见design.md及debug目录。

集中式真正单次组装/优化128参考和256自行车输入变量；参考层分块目标和本车行同时在一个QP求解，自行车层装配6对联合变量耦合行。局部每车只求32/64变量，自身Agent/目标/缓存，另三车由真实ROS2消息取得公开状态和预测。按A1→B1→A2→B2固定一轮，读取已更新最终计划/未更新种子；局部禁用中央入口和旧联合方法，无救场。两臂使用相同路线、道路/权限/分离几何，但参考分块单次与局部依次求解、联合/固定邻车耦合、预算作用域不同，不是等算力或同一数学问题的纯架构因果实验。

M2b-release-v1以实际旋转车身最小方向x>=26.5 m核查释放，初次授予及交接均经过四车同版本条件候选/认可后才生效；无合格下一队首允许owner归None。请求首次周期保留，资格只从已请求且实际停在队首、转向就绪的同侧队首选取。FIFO-ready与alternating-ready只改变选择规则，后者仅在实际授予时更新方向历史。单车独占，无批次、无在线中央全未来门禁或备用。

真实五个ROS2节点（同进程）、可靠DDS、理想同步无注入延迟，计算期间仿真时钟冻结。0.5 s是每次本车预测调用或中央一次四车调用，四车顺序总预算不同；端到端包括参考/求解/修复/消息/认可/提交，输出导出及离线审计单列。消息字节为原完整应用JSON信封，不是压缩存储字节，不含网络头；分项无法完全分离处保留组合口径，不虚构纯通信耗时。在线/离线时间及每车/联合QP、变量、迭代、更新、修复、误差、请求/就绪等待、进入、空闲原因、停走和工程存储见metrics.json。

日志使用无损gzip和单份计划ID引用；完整final321节点保留，seed相同前缀引用前周期已提交final并保存新增10点尾部，恢复原ID及逐浮点内容。消息引用计划正文，原应用字节和浮点精度不变。正常周期不保存QP矩阵/冗余行全细节；失败及前两周期、交接/输入修复及邻接周期保存诊断。无穷矩阵界显式编码为Infinity/-Infinity字符串以保持JSON可读且不污染有限物理数值。代码按内容版本在sources/保存一份，每轮只引用版本，六次均保留自己的实际、事件、消息时间及计时。工件30 MiB/总200 MiB为软目标，超过时最大文件和原因列明，不删物理/协议证据凑数。

验收独立标量两半步RK4重积分每个实际间隔及全累计、全部最终预测，重建首次请求、独立就绪/选车、清空/授权/进入、四最终ID与四ACK/提交、信息来源及全部出口计时。旋转矩形几何复用M1库，未宣称完全独立几何实现；summary的true不替代验收。未新增四车中央完整未来诊断，不将其当成局部未来证明。

两次重复只核查确定性；授权顺序打平是本场景结果，不修改初态制造差异。未实现六车、同向批次、额外延迟、在线备用，未证明连续安全、递归可行、硬实时、实车可靠或持续到车无饥饿。

复现先source /opt/ros/jazzy/setup.bash，OPENBLAS_NUM_THREADS=1、PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH、ROS_LOG_DIR=/tmp/m2b_ros；依赖版本和每组精确命令见environment.json。run_m2b.py --check-scene；--arm central/local --policy fifo-ready/alternating-ready --repeat 2 --output 新目录。随后verify_m2b.py --run 总包、report_m2b.py --run 总包。不得覆盖已存在输出。
'''
 (root/'report.md').write_text(text);print('formal audits',len(formal),'all passed',complete)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();report(a.run)
