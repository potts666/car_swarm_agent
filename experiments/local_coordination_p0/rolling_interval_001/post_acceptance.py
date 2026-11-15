"""Additional algebraic tail/ACK/version checks and readable summary enrichment. Zero solves."""
import csv,copy,gzip,os,statistics
from common import *
from plan_io import load
from rolling_recover import nodes,objects,plan_at
from contracts import physical_id
from release import canonical_command,clear
from frames import validate_publication,discover

def csvread(name):
 csv.field_size_limit(__import__('sys').maxsize)
 with (HERE/name).open() as f:return list(csv.DictReader(f))
def csvwrite(name,rows):
 with (HERE/name).open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

def run():
 assert read(HERE/'results/rolling_independent.json')['passed'];ns=nodes();ledger=read(HERE/'budget_ledger.json');budget_sha=sha(HERE/'budget_ledger.json');assert read(HERE/'results/source_isolation_certificate.json')['passed'];p=plan_at(ns[0]['cycle']);checks=[];metadata={};epoch=(HERE/'version_history/rolling_v1/pre_registration.json').stat().st_mtime_ns;seen={x.name[:-len('.json.gz')] for x in (HERE/'results/derivatives').glob('*.json.gz') if x.stat().st_mtime_ns<=epoch};cache_initial=len(seen);acks={}
 for line in (HERE/'results/wire/R-rolling.jsonl').read_text().splitlines():
  message=json.loads(line)
  if message['kind']=='ACK_MPC':
   v=json.loads(message['wire_json'])['payload'];acks.setdefault((v['context'],tuple(v['group']),v['commit_id']),set()).add(v['actor'])
 from canonical import reference
 for n in ns:
  k=n['cycle']
  if n['release'] and n['release']['transaction']['accepted']:
   e=n['release'];from plan_io import restore
   fresh_clear=clear(p,restore(n['source']),e['meta']['outgoing']);assert fresh_clear['full_Clear'] and json.loads(json.dumps(fresh_clear,default=str))==e['clear'];new=load(e['candidate']);a,b=read(HERE/p['frame_file']),read(HERE/new['frame_file']);validate_publication(a);validate_publication(b);closure=discover(p,new,a,b,k);assert set(closure['participants'])==set(e['transaction']['participants']);assert set(e['transaction']['ACK'])==set(closure['participants']);p=new
  assert physical_id(p)==n['plan_before'];assert n['context']==content_id({'plan':physical_id(p),'source':n['source'],'outer':p['outer_frame_id']});new=copy.deepcopy(p);refs=reference(p)
  for c,row in n['rows'].items():
   info={'actual_activity':c in n['actual_activity'],'actual_v':float(F(n['source']['v'][c])),'actual_delta':float(F(n['source']['delta'][c])),'executed_a':float(F(n['commands'][c][0])),'executed_omega':float(F(n['commands'][c][1])),'successor_v':float(F(n['successor']['v'][c])),'successor_delta':float(F(n['successor']['delta'][c])),'native_attempted':any(x['cycle']==k and x['actor']==c for x in ledger),'nominal_a':float(F(canonical_command(c,k,p['per_car_policies'][c]['service_start'])[0])),'first_delta_a':0.,'first_delta_omega':0.,'window_input_changed':False}
   if row.get('model_object'):
    job=objects(row['model_object']);assert job['actor']==c and job['cycle']==k and job['arm']=='O-E' and job['objective']['kind']=='energy' and F(job['objective']['h'])==H and job['objective']['R']==job['objective']['S']==[1,1] and F(job['rho'])==F(1,1000) and len(job['reference'])==64;ref=job['reference'];key=content_id({'car':c,'pose':job['source']['poses_Q'],'v':job['source']['v'],'delta':job['source']['delta'],'controls':[[ref[2*j],ref[2*j+1]] for j in range(32)],'sub':64,'builder_sha':sha(HERE/'sources/certify_interfaces.py')});assert (HERE/f'results/derivatives/{key}.json.gz').exists(),('cache key',k,c);info.update(derivative_cache_key=key,derivative_cache_hit=key in seen);seen.add(key)
   if row['adopted']:
    job=objects(row['model_object']);ad=objects(row['admission_object']);commands=[[F(x) for x in u] for u in ad['commands']];phase=k+32;assert F(n['source']['v'][c])+H*sum(u[0] for u in commands)==F(refs[phase]['v'][c]);assert F(n['source']['delta'][c])+H*sum(u[1] for u in commands)==F(refs[phase]['delta'][c]);assert all(tuple(map(F,p['controls'][c][t]))==tuple(map(F,canonical_command(c,t,p['per_car_policies'][c]['service_start']))) for t in range(phase,p['B0']));checks.append({'cycle':k,'actor':c,'model_id':job['model_id'],'exact_canonical_terminal_v_delta':True,'full_saved_suffix_canonical':True,'remaining_tail_id':content_id(p['controls'][c][phase:]),'independent_64_prefix_and_receiver':'results/rolling_independent.json'});new['controls'][c][k:phase]=commands;info.update(first_delta_a=float(F(ad['delta'][0])),first_delta_omega=float(F(ad['delta'][1])),window_input_changed=bool(ad['a_changed'] or ad['omega_changed']))
    group=next(g for g in row['groups'] if c in g);assert set(group)==acks[(n['context'],tuple(group),row['commit_id'])],('all required block ACKs',k,c)
   metadata[k,c]=info
  assert physical_id(new)==n['plan_after'];p=new
 versions={}
 for name,path in [('rolling_v1',HERE/'version_history/rolling_v1/executed_source_versions.json'),('rolling_v2',HERE/'version_history/rolling_v2/executed_source_versions.json'),('rolling_v3',HERE/'executed_source_versions.json')]:
  files=read(path)
  for key,h in files.items():assert sha(HERE/'source_archive'/name/key)==h,(name,key)
  versions[content_id(files)]=name
 assert all(x['source_version'] in versions for x in ledger)
 for version in ('rolling_v1','rolling_v2'):
  inv=read(HERE/f'results/{version}_preserved_inventory.json')
  for name,h in inv['files'].items():assert sha(HERE/'version_history'/version/name)==h,(version,name)
 write(HERE/'results/tail_ack_source_certificate.json',{'passed':True,'adopted_obligations':checks,'all_registered_block_ACKs_before_joint_activation':True,'activation_scope':'ideal shared logical boundary; notifications sequential, no asynchronous network assertion','versions':versions,'new_optimizer_calls':0})
 rows=csvread('cycle_summary.csv')
 for r in rows:
  r.update(metadata[int(r['cycle']),r['actor']])
  for name in ('JE_change_UJ','max_original_row_residual','receiver_tail_margin_m','zero_tight_slack'):
   if r.get(name) not in ('',None):r[name]=float(F(r[name]))
 csvwrite('cycle_summary.csv',rows);timing=csvread('timing.csv')
 for r in timing:r.update({k:v for k,v in metadata[int(r['cycle']),r['actor']].items() if k.startswith('derivative_cache_')})
 csvwrite('timing.csv',timing)
 trade=csvread('tradeoff.csv')
 for r in trade:
  for name in ('delta_JE_UJ','delta_F_lower_UJ','delta_F_upper_UJ'):r[name]=float(F(r[name]))
 csvwrite('tradeoff.csv',trade)
 comparison=csvread('comparison.csv')
 for r in comparison:
  for name in ('JE_UJ','JE_saving_absolute_UJ'):r[name]=float(F(r[name]))
 csvwrite('comparison.csv',comparison)
 worker_bytes=[]
 for mode,pattern in [('solve','k*_transport.json.gz'),('assemble','k*_precheck_transport.json.gz')]:
  files=[x for x in (HERE/'results/workers').glob(pattern) if ('_precheck_' in x.name)==(mode=='assemble')];data=[json.loads(gzip.decompress(x.read_bytes())) for x in files];worker_bytes.append({'mode':mode,'messages':len(data),'request_bytes':sum(x['sent_bytes'] for x in data),'response_bytes':sum(x['received_bytes'] for x in data),'scope':'actual worker pipe JSON, separate from logical application messages'})
 data=[read(x) for x in (HERE/'results/preflight_samples').glob('*_transport.json')];worker_bytes.append({'mode':'zero_call_phase_samples','messages':len(data),'request_bytes':sum(x['sent_bytes'] for x in data),'response_bytes':sum(x['received_bytes'] for x in data),'scope':'preflight worker assembly only; zero solve calls'})
 csvwrite('worker_communication.csv',worker_bytes)
 communication=[]
 for wirefile in sorted((HERE/'results/wire').glob('*.jsonl')):
  segments=[];previous=0
  for line in wirefile.read_text().splitlines():
   msg=json.loads(line)
   if msg['seq']<=previous or not segments:segments.append([])
   segments[-1].append(msg);previous=msg['seq']
  for segment,data in enumerate(segments):
   scope='actual stream; logical in-process dispatch'
   if wirefile.stem=='R-rolling_suffix':scope=['v1_stopped_completion','v2_stopped_completion','final_main_suffix'][segment]
   elif wirefile.stem=='R-rolling':scope='actual_main_prefix_stage_'+str(segment+1)
   for phase in sorted({x['phase'] for x in data}):
    rows=[x for x in data if x['phase']==phase];communication.append({'stream':wirefile.stem,'run_segment':segment+1,'phase':phase,'scope':scope,'messages':len(rows),'JSON_bytes':sum(x['bytes'] for x in rows),'cross_logical_endpoint_messages':sum(x['sender']!=x['receiver'] for x in rows),'cross_logical_endpoint_bytes':sum(x['bytes'] for x in rows if x['sender']!=x['receiver']),'note':'separate original stopped branches and current suffix; not measured network'})
 csvwrite('communication.csv',communication)
 counts={name:{'attempts':sum(versions[x['source_version']]==name for x in ledger),'Solved':sum(versions[x['source_version']]==name and x['status']=='Solved' for x in ledger)} for name in versions.values()};cold=sum(not r.get('derivative_cache_hit',True) for r in metadata.values());warm=sum(r.get('derivative_cache_hit',False) for r in metadata.values());kin={'goal_hold','wait'};active_max=max(len(n['actual_activity']) for n in ns);adopted=sum(r['adopted'] for n in ns for r in n['rows'].values());first_changes={name:sum(r[name]!=0 for r in metadata.values()) for name in ('first_delta_a','first_delta_omega')};fallback_cycles=sorted({n['cycle'] for n in ns if any(r['classification']=='fallback_after_failure' for r in n['rows'].values())});write(HERE/'results/readable_summary.json',{'passed':True,'versions':counts,'car_cycles':4*len(ns),'candidate_adopted':adopted,'whole_window_changed':sum(x['window_input_changed'] for x in metadata.values()),'first_changes':first_changes,'interval_effective_activity_max':active_max,'fallback_logical_cycles':fallback_cycles,'derivative_cache_reconstructed':{'cold':cold,'warm':warm,'initial_keys':cache_initial,'definition':'exact frozen car/ownpose/v/delta/32control/sub/builderSHA key; pre-v1-registration file timestamps and successive real accesses; no other process generated derivatives during execution'},'cycle_wall_median_seconds':statistics.median(n['wall_seconds'] for n in ns),'native_solve_median_seconds':statistics.median(x['solve_time'] for x in ledger),'new_optimizer_calls':0})
 write(HERE/'results/source_read_scope.json',{'passed':True,'source_inventory_false_flag_scope':'inventory collection hashes bytes, not a claim that no earlier audit process read historical policies','earlier_source_audit_read_history':True,'original_1822_comparison_after_its_own_replay_before_Eanchor':True,'Eanchor_comparison_after_its_replay':True,'generation_process_does_not_use_historical_controls':True,'fresh_strict_order_recheck':'results/source_isolation_certificate.json','strict_order_recheck_before_comparison':['fresh1822 independent audit','fresh1615 anchor independent audit'],'new_optimizer_calls':0})
 report=HERE/'report.md';report.write_text(report.read_text().replace('解析参考重新生成且独立重放后才对历史完整策略作语义比较；历史数字策略未作为输入','初始策略在自身独立重放后对照旧初始策略，解析参照独立重放后对照旧E-anchor；开工来源审计会读取历史策略，均不作为初始化控制输入。另用文件访问白名单隔离重建，禁止历史完整策略读取直到1822初始与1615解析参照均独立重放通过，所得控制/owner/策略与实际主线完全相同，详见source_isolation_certificate与source_read_scope'))
 extra='\n\n补充独立接口检查：所有实际受理前缀的终端v/delta与所选规范相位精确相等，N之后保存输入逐项等于规范尾段；原64子段前缀/接收审查与此代数检查共同闭合完整SE2后续证明。所有实际块ACK、源码版本/尝试身份与历史停止版本SHA通过，见results/tail_ack_source_certificate.json。\n\n原始失败与修复：v1在413—415拍出现6个序列化备份车拍（Python4300位限制），v2在A1的466、467拍及B1的521—523拍出现5个改善下限备份车拍；总共8个逻辑拍沿备份。两次都先停止新增数值释放并安全完成，保存完整版本，零求解重放raw后从真实416/524检查点分叉。v3原JE采用精确严格下降，不设任意正改善下限；约束、量化、目标、求解设置保持。保存raw的回归候选未追溯采用。\n\n实际尝试版本统计：'+str(counts)+'。解析245次为参考预测；实际滚动后的非零尾段不能沿用原零输入最优证书，因此实际调用更多。区间有效活动最多'+str(active_max)+'车，与整个任务初始最多4车分列。首拍加速度/转角速度真实改变数'+str(first_changes)+'，完整窗改变'+str(sum(x['window_input_changed'] for x in metadata.values()))+'；幅度列见cycle_summary，不能仅凭非零就声称显著转向收益。\n\n缓存冷暖按冻结get键及首次登记前文件时间重建：cold='+str(cold)+',warm='+str(warm)+'；这是导数模板缓存，不代表全部CPU缓存。不同阶段负载不同，不据此声称冷暖加速比。逐拍墙钟中位数'+str(statistics.median(n['wall_seconds'] for n in ns))+'秒，h=.2仍是逻辑时间。\n\n代表性同源窗口交换（完整表tradeoff.csv；汇总为显示近似，原始精确数值见内容对象；以下Δs/ΔF使用区间，非严格进度改善断言）：\n\n|拍/车|ΔJE(U_J)|Δs(m)区间|ΔF(U_J)区间|\n|---|---:|---|---|\n'
 selected=[];seen_modes=set()
 for r in trade:
  key=next(n['rows'][r['actor']]['mode'] for n in ns if n['cycle']==int(r['cycle']))
  if key in seen_modes:continue
  seen_modes.add(key);selected.append(r)
 for r in selected[:8]:extra+=f"|{r['cycle']}/{r['actor']}|{float(r['delta_JE_UJ']):.9g}|[{float(F(r['delta_s_lower_m'])):.6g},{float(F(r['delta_s_upper_m'])):.6g}]|[{float(r['delta_F_lower_UJ']):.9g},{float(r['delta_F_upper_UJ']):.9g}]|\n"
 report.write_text(report.read_text()+extra);assert sha(HERE/'budget_ledger.json')==budget_sha;print('PASS additional tail/ACK/source/cache/report checks; zero new solves',flush=True)
if __name__=='__main__':run()
