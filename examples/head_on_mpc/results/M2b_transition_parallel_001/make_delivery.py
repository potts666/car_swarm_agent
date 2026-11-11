"""Read finished artifacts; summarize only. Never edits experiment logs or acceptance."""
import csv,gzip,hashlib,json,math,os,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def load(p):return json.loads(p.read_text())
def rows(p):
 with gzip.open(p,'rt') as stream:
  for line in stream:yield json.loads(line)
def write(p,value):p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def table(p,data):
 if not data:return
 with p.open('w') as f:
  out=csv.DictWriter(f,fieldnames=list(data[0]));out.writeheader();out.writerows(data)
def distribution(values):
 if not values:return {k:None for k in ('p50_ms','p95_ms','p99_ms','max_ms','over_200ms')}
 return dict(zip(('p50_ms','p95_ms','p99_ms','max_ms'),[float(v*1000) for v in np.percentile(values,[50,95,99,100])]),over_200ms=sum(v>.2 for v in values))

def compare(left,right,mapping=None):
 ia=iter(rows(left/'actual.jsonl.gz'));ib=iter(rows(right/'actual.jsonl.gz'));maximum=np.zeros(5);input_error=0.;identity_equal=True;count=0;mismatch=[]
 while True:
  a=next(ia,None);b=next(ib,None)
  if a is None or b is None:break
  count+=1;identity_equal &= a['plans']==b['plans']
  for c in a['states']:
   d=mapping.get(c,c) if mapping else c;delta=abs(np.asarray(a['states'][c])-np.asarray(b['states'][d]));maximum=np.maximum(maximum,delta.max(axis=0));input_error=max(input_error,float(max(abs(np.array(a['controls'][c][:2])-b['controls'][d][:2]))))
  if (mapping.get(a['owner'],a['owner']) if mapping else a['owner'])!=b['owner']:mismatch.append(a['cycle'])
 rounds_a=list(rows(left/'rounds.jsonl.gz'));rounds_b=list(rows(right/'rounds.jsonl.gz'));sealed_equal=len(rounds_a)==len(rounds_b) and all(x['input_ids']==y['input_ids'] and x['input_set_id']==y['input_set_id'] for x,y in zip(rounds_a,rounds_b))
 return dict(common_cycles=count,actual_components_max_abs=maximum.tolist(),first_input_max_abs=input_error,public_final_ids_equal=bool(identity_equal),sealed_input_ids_equal=sealed_equal,owner_mismatch_cycles=mismatch,declared_absolute_tolerance=1e-9,numeric_equal_within_tolerance=bool(max(maximum.max(),input_error)<=1e-9),rename_mapping=mapping)

def main():
 available=[];details={};stages=load(ROOT/'study_status.json').get('stages',[]) if (ROOT/'study_status.json').exists() else []
 for stage in stages:
  name=stage['name'];run=ROOT/name/'r1';summary=load(run/'summary.json');audit=load(run/'independent_validation.json');planning=list(rows(run/'planning.jsonl.gz'));committed=[r for r in planning if r['committed']];rounds=list(rows(run/'rounds.jsonl.gz'));startup=load(run/'startup.json')
  all_quantiles=distribution([r['full_cycle_wall_s'] for r in planning]);commit_quantiles=distribution([r['full_cycle_wall_s'] for r in committed]);local_time=[];car_metrics={c:dict(reference_qp_calls=0,start_reference_qp_calls=0,bicycle_qp_calls=0,qp_updates=0,input_repairs=0) for c in audit['dimensions']};rpc={};cache_scope='not persisted in this frozen logger; no zero-hit inference'
  for r in planning:
   for k,v in r.get('rpc_wall_s',{}).items():rpc[k]=rpc.get(k,0.)+v
  for r in rows(run/'solves.jsonl.gz'):
   for c,q in (r.get('local') or {}).items():
    m=car_metrics[c];m['reference_qp_calls']+=q['reference_numeric_qp_calls'];m['bicycle_qp_calls']+=q['bicycle_numeric_qp_calls'];m['qp_updates']+=q['qp_updates'];m['input_repairs']+=sum(len(u.get('stop_input_adjustments',[])) for u in q['subproblems']);local_time.append(q['local_computation_s'])
  sources={};candidate_checks=0
  for r in rows(run/'candidates.jsonl.gz'):
   sources[r['source']]=sources.get(r['source'],0)+1;candidate_checks+=1+int(bool(r.get('old_shift_rejection')))
   if r.get('reference_record'):car_metrics[r['car']]['start_reference_qp_calls']+=int(r['reference_record']['solver_called'])
  for r in rounds:
   w=r['workers'];starts=[q['compute_start_ns'] for q in w.values()];ends=[q['compute_end_ns'] for q in w.values()];r['span_wall_s']=(max(ends)-min(starts))/1e9;r['overlap_s']=max(0.,(min(ends)-max(starts))/1e9)
  table(ROOT/(name+'_concurrency.csv'),[dict(cycle=r['cycle'],round=r['coordination_round'],span_wall_s=r['span_wall_s'],overlap_s=r['overlap_s'],worker_pids=';'.join(str(q['pid']) for q in r['workers'].values())) for r in rounds])
  row=dict(name=name,backend=stage['backend'],variant=stage['variant'],complete=summary['all_completed'],full_acceptance=audit['actual_full_acceptance'],prefix_acceptance=audit['actual_prefix_acceptance'],committed_cycles=summary['committed_cycles'],completion_sim_s=summary['completion_sim_s'],online_wall_s=summary['online_wall_s'],driver_stage_wall_s=stage['wall_s'],startup_wall_s=startup['total_startup_wall_s'],shutdown_wall_s=summary['shutdown_wall_s'],export_wall_s=summary['export_wall_s'],audit_wall_s=audit['independent_audit_wall_s'],**all_quantiles,max_local_computation_ms=1000*max(local_time,default=0.),four_way_overlap_cycles=sum(r['overlap_s']>0 for r in rounds),send_messages=sum(r['send_messages'] for r in planning),application_bytes=sum(r['application_bytes'] for r in planning),cache_hits=None,cache_hits_measured=False,artifact_bytes=sum(p.stat().st_size for p in (ROOT/name).rglob('*') if p.is_file()),failure=summary['failure'])
  available.append(row);details[name]=dict(all_exit_timing=all_quantiles,committed_cycle_timing=commit_quantiles,executable_timing=distribution([r['control_executable_wall_s'] for r in planning if r.get('control_executable_wall_s') is not None]),local_timing=distribution(local_time),worker_span_timing=distribution([r['span_wall_s'] for r in rounds]),own_car_metrics=car_metrics,candidate_sources=sources,candidate_validation_evaluations=candidate_checks,cache_hits=None,cache_hits_measured=False,cache_counter_scope=cache_scope,rpc_wall_s=rpc,exchange_parent_and_logging_wall_s=summary['online_wall_s']-sum(rpc.values()),resources=load(run/'resources.json'),dimensions=audit['dimensions'],minimum_actual_gap_m=audit['physical']['minimum_gap_m'],handoffs=audit['handoffs'])
 table(ROOT/'comparison.csv',available);write(ROOT/'comparison.json',dict(stages=available,details=details))
 pairs={}
 for a,b in [('S-J_1','P-J_1'),('P-J_1','P-J_2'),('S-J_1','S-J_2')]:
  if a in details and b in details:pairs[a+'__'+b]=compare(ROOT/a/'r1',ROOT/b/'r1')
 if 'S-J_1' in details and 'rename_full' in details:pairs['rename_numeric']=compare(ROOT/'S-J_1/r1',ROOT/'rename_full/r1',dict(A1='west_head',A2='west_tail',B1='east_head',B2='east_tail'))
 write(ROOT/'determinism.json',pairs)
 # Metrics are definitions, never claims that an unrecorded quantity was measured.
 defs=[('full_cycle_wall_s','s','all exits, perf_counter end-start, includes start preparation/IPC/DDS/checks/ACK/staging','online','200ms target; frozen simulation, no real-time proof'),('local_computation_s','s','candidate build + own prevalidation + own optimizer','online','original .5s budget; IPC/final ACK are in full cycle'),('worker_span_wall_s','s','max worker end - min worker start in a shared monotonic clock','online','wall span, not CPU sum'),('four_way_overlap_s','s','max(0,min ends-max starts)','online','separate PIDs; excludes candidate preparation'),('rpc_wall_s','s','parent RPC entry to full response cohort, separated by method','online','includes IPC and waiting; not pure solver time'),('exchange_parent_and_logging_wall_s','s','online wall minus sum RPC calls','online','combined DDS/parent bookkeeping; no isolated pure DDS measurement'),('cache_hits','count','null when counters not persisted','online','never interpret null as zero'),('application_bytes','byte','UTF-8 canonical envelope length, original plan restored','online','excludes DDS framing/retransmission overhead'),('candidate_partial_residuals','original units','independent queue or scalar reference conditions','offline','not complete OCP proof'),('actual_full_acceptance','boolean','original dynamics/sign/physical/goals plus resource/final-version audit','offline','finite samples and ideal communication'),('model_usage','token','not captured by this experiment driver','outside','no billing inference from duration or hook counts')]
 table(ROOT/'metric_dictionary.csv',[dict(metric=a,unit=b,definition=c,scope=d,limitation=e) for a,b,c,d,e in defs])
 if available:
  try:
   import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
   fig,ax=plt.subplots(figsize=(9,3.8));labels=[r['name'] for r in available if r['variant']=='main'];mainrows=[r for r in available if r['variant']=='main']
   ax.bar(labels,[r['p95_ms'] for r in mainrows],color=['#4c78a8' if r['backend']=='S-J' else '#59a14f' for r in mainrows]);ax.axhline(200,color='red',linestyle='--',label='200 ms target');ax.set_ylabel('Complete cycle P95 (ms)');ax.legend();fig.tight_layout();fig.savefig(ROOT/'cycle_p95.png',dpi=160);plt.close(fig)
   if 'P-J_1' in details:
    spans=list(csv.DictReader((ROOT/'P-J_1_concurrency.csv').open()));fig,ax=plt.subplots(figsize=(9,3.5));ax.plot([int(r['cycle']) for r in spans],[1000*float(r['overlap_s']) for r in spans]);ax.set(xlabel='Cycle',ylabel='Four-worker overlap (ms)');fig.tight_layout();fig.savefig(ROOT/'parallel_overlap.png',dpi=160);plt.close(fig)
   if 'P-J_1' in details:
    events=details['P-J_1']['handoffs'];owners=['A1','B1','A2','B2'];fig,ax=plt.subplots(figsize=(9,3.4))
    for index,event in enumerate(events):
     owner=event.get('owner')
     if owner not in owners:continue
     begin=event['cycle']*.2;end=events[index+1]['cycle']*.2 if index+1<len(events) else next(r['completion_sim_s'] for r in available if r['name']=='P-J_1')
     ax.broken_barh([(begin,end-begin)],(owners.index(owner)-.3,.6),facecolors='#4c78a8');ax.text((begin+end)/2,owners.index(owner),'v'+str(event['version']),ha='center',va='center',color='white')
    ax.set(yticks=range(4),yticklabels=owners,xlabel='Simulation time (s)',title='Actual exclusive owner, P-J_1');fig.tight_layout();fig.savefig(ROOT/'permission_timeline.png',dpi=160);plt.close(fig)
  except ImportError as e:write(ROOT/'plot_missing.json',dict(error=str(e)))
 status=load(ROOT/'study_status.json');formal=[r for r in available if r['variant']=='main'];complete=len(formal)==4 and all(r['full_acceptance'] for r in formal);deterministic=bool(pairs) and all(v['numeric_equal_within_tolerance'] and v['sealed_input_ids_equal'] for k,v in pairs.items() if k!='rename_numeric')
 lines=['# M2b 候选衔接与同步并行：实际结果','',f'本轮已实现模式／停车参考感知的公开候选、工作副本提交、封存同轮信息、四个持久计算进程和批量定向 DDS。主场景四次完整验收：{complete}；S-J/P-J 封存输入及数值一致性：{deterministic}。以 comparison.csv、独立验收和原始日志为准。','', '历史诊断 30 项阶段／24 项终端覆盖于 94 个重建窗口；两类诊断不默认视作实际闭环失败。全部本车队列由实际已提交输入和规则恢复，不依赖当次 final 推断。新候选只承诺声明的物理／阶段及部分参考行；完整 OCP 证书、一般递归可行性与稳定性仍未证明。','', '|运行|完整验收|仿真完成 s|在线墙钟 s|P95 ms|最大 ms|四进程重叠周期|','|---|---|---:|---:|---:|---:|---:|']
 for r in available:lines.append(f"|{r['name']}|{r['full_acceptance']}|{r['completion_sim_s']}|{r['online_wall_s']:.3f}|{r['p95_ms']:.2f}|{r['max_ms']:.2f}|{r['four_way_overlap_cycles']}|")
 lines+=['','S-G 引用 M2b_stage1_001：176.4 s、882 个执行周期、在线 543.774104 s，原独立验收通过。它与新机制的信息与候选规则不同；纯后端比较只使用 S-J/P-J。上线计算移除或输入规则变化不能统称纯求解提速。','', '与 S-G 多出的一个周期已定位：B2 在 cycle855 控制后仍有约8.30436e−8 m/s，原切段规则要求绝对速度≤1e−12，因此 cycle856 继续施加 −4.15218e−7 m/s² 的停车控制，cycle857 才转段；旧 S-G 在 cycle856 已转段。没有把残余速度截成零。事件表见 historical_transition_comparison.csv。','', '历史 debug_v1 在同一原始矩阵诊断中重现参考 solved／自行车 primal infeasible；必要离散停车余量约 −1.71924e−7 m。部分候选残差通过不消除完整 QP 的拒绝。debug_v2 缺源码版本及矩阵，保留缺口。','', '正式设置：一轮，原 N/h/sample/物理门限/权重/最多三次更新和 .5 s 本车预算。阶段重建时使不适用热启动失效；其余本车参考继续用已提交参考移位。输入修复后仍重积分、验收并重发带内容 ID 的预测。','', '验收器保留原独立标量重积分、原跨零、全部六车对、真实车身、授权／清空／READY／FIFO、实际输入与最终计划对应，再追加恢复队列及轮号／封存输入／DDS来源检查。full=false 的有界前缀不冒充任务通过。','', '源码与测试：sources/<current_source_version>/ 为冻结主快照，manifest 核对字节；supplemental_sources/ 为独立复现发现的 5 个既有传递依赖，另有哈希，不覆写原 manifest。isolated_snapshot_tests.log 保留首次遗漏，isolated_complete_snapshot_tests.log 记录补齐后 31 项通过。','', '计时与资源：完整周期包含准备、IPC、DDS、最终检查和提交；启动、发现、关闭、导出、独立审计分列。rpc 方法边界有实测；DDS 与父进程簿记只能合并报告，不能伪称独立纯通信时延。当前持久日志未保存 Actor 的命中计数，cache_hits=null／未测。首个 S-J 完整运行期间曾进行约 3.9 s 独立快照单元检查，披露其短时资源竞争；正式在线和整条独立验收由驱动串行，不重叠。','', '所有提交前失败均保留原实际状态，不消耗转向队列或已提交缓存。工作进程异常负例通过；执行决定后的故障不被说成能撤销既有实际输入。离线结束不是实车备用停车。','', '范围限制：理想通信、冻结仿真时钟、有限采样及固定配置的重复，不能证明硬实时、连续安全、任意初态、延迟鲁棒或一般递归可行。200 ms 是否达到按实测表报告；缓存未测、纯 DDS 细分未测、模型 token usage 未采集，不编造零值、费用或订阅换算。','', '论文读取版本与条件见 literature_conditions.md；单周期时序与提交边界见 design.md；字节／来源与准确入口见 README.md。']
 if (ROOT/'driver_failure.json').exists():lines+=['','自动驱动明确失败：'+load(ROOT/'driver_failure.json')['error']+'。本轮未完成，保留反例，不增加预算／轮数或放松条件。']
 if (ROOT/'post_review_validation.json').exists():
  review=load(ROOT/'post_review_validation.json');change=load(ROOT/'post_review_change.json')
  lines+=['','收尾版本边界：六次完整运行使用 formal_source_version='+change['formal_source_version']+'。随后仅在 run_m2b_jacobi.py 的资源适配入口加固，维护版本='+change['maintenance_source_version']+'，改为消费实际报文授权版本并绑定发送者／周期。全部六次原日志的1002次资源准备返回上下文一致；24份实际释放各4类坏报文，共96次拒绝且通道无状态变更。新源码260周期真实DDS前缀独立验收='+str(review['prefix_acceptance'])+'，提交计划ID与原正式前缀一致='+str(review['submitted_plan_ids_equal_to_formal_prefix'])+'。新源码未重跑六次全任务，不能把历史完整结果改标成新字节完整复现。详情见 post_review_change.json / post_review_validation.json / release_adapter_replay.json。']
  lines+=['','S-J的具体串行范围是起始候选构造与本车优化；两臂均持有四个worker，阶段、公开计划验证、ACK和暂存均按组并行。它不是单进程完全串行实现。完整周期包含相同协议和检查开销；纯DDS细分不单独宣称。','', 'P-J每次883个执行周期均有四个PID计算区间重叠。主场景每次883个实际执行周期均超过200ms；.5s本车预算全部满足。最小实际车身间距约1.199998m，对原0.5m要求的净裕量约0.699998m；单位与净裕量分列。','', '相邻final与轮内seed→final的时间对齐变化见 prediction_change_summary.json 和各CSV；本轮观测最大值不是新的一致性上界。三张图分别为 cycle_p95.png、parallel_overlap.png、permission_timeline.png。']
 (ROOT/'report.md').write_text('\n'.join(lines)+'\n')
 write(ROOT/'delivery_status.json',dict(main_complete=complete,determinism_passed=deterministic,driver_complete=status['completed'],generalization_full_acceptance={r['name']:r['full_acceptance'] for r in available if r['name'].endswith('_full')},reported_stages=len(available),final_source_full_repetitions=False,final_source_affected_prefix_acceptance=load(ROOT/'post_review_validation.json')['prefix_acceptance'] if (ROOT/'post_review_validation.json').exists() else None))
 print(json.dumps(load(ROOT/'delivery_status.json')),flush=True)

if __name__=='__main__':main()
