import collections,datetime,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;R=Path('examples/head_on_mpc/results/M2b_fourcar_001')
def read(n):return json.loads((A/n).read_text())
def write(n,d):(A/n).write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')
S=read('audit_summary.json');P=read('stage_summary.json');K=read('usage_consistency.json');limits=read('historical_limit_samples.json');env=json.loads((R/'C-FIFO/environment.json').read_text());root='01a18b08-73cc-7001-ba3d-92d49ecdd80e';base='/home/sunflower/.codex/sessions/2026/10/30/rollout-2026-10-30T12-19-09-01a18b08-73cc-7001-ba3d-92d49ecdd80e.jsonl'
actors=[]
U=read('task_usage_increments.json')
for sid in dict.fromkeys(u['thread_id'] for u in U):
 rows=[u for u in U if u['thread_id']==sid];actors.append(dict(thread_id=sid,role='main' if sid==root else 'automatic guardian approval',usage_records=len(rows),first_usage=rows[0]['timestamp'],last_usage=rows[-1]['timestamp'],totals={k:sum(x['effective_delta'][k] for x in rows) for k in rows[0]['effective_delta']}))
write('actor_totals.json',actors)
write('environment_and_scope.json',dict(audit_target='M2b_fourcar_001',MRIB_alias_confirmed=False,MRIB_scope_note='No MRIB name in original task evidence; the supplied M2b output/task is matched. Whether MRIB is the user\'s alias remains unconfirmed.',cwd='/home/sunflower/car_swarm_switch_ws/src/car_swarm_single_lane',original_host_platform=env['platform'],original_hostname=None,host_evidence='M2b_fourcar_001/C-FIFO/environment.json',original_client='codex_vscode / source=vscode',original_cli_version='0.164.0-alpha.11.1',original_extension_version=None,current_installed_extension_version_for_corroboration_only='26.1021.22027',current_bundled_cli_same_version=True,main_model='gpt-6.1-sol',main_reasoning='medium',guardian_model='codex-auto-review',guardian_reasoning='low',historical_plan_type='plus',exact_authentication_mode=None,credential_files_read=False,api_cost_available=False,credits_snapshot=dict(has_credits=False,balance='0'),task_start_utc=K['task_start_utc'],task_end_utc=K['task_end_utc'],local_timezone='Asia/Shanghai (+08:00)',task_start_local='2026-11-01 21:50:13.786',task_end_local='2026-11-02 07:46:00.619',root_thread_id=root,root_turn_ids=K['root_turn_ids'],excluded='T1/M0/M1/M2a/storage, later folder explanation, this audit, old Oct30 guardian',field_definition_evidence='same-version generated protocol token_schema_definitions.json + historical per-response/cumulative arithmetic',billing_semantics_not_established=True))
# Evidence of waits is a cross-cut of the stage table, never an additional total.
main=S['root_totals'];g=S['guardian_totals'];total=S['totals'];cache_ratio=total['cached_input_tokens']/total['input_tokens']
rows=[]
wall_labels={P[0]['stage']:'未独立记录',P[1]['stage']:f"Σ{P[1]['program_wall_s']:.1f}s，调试有重叠",P[2]['stage']:f"在线Σ{S['program_reported_sums']['online_wall_s']:.2f}s",P[3]['stage']:f"正式审计Σ{S['program_reported_sums']['audit_wall_s']:.2f}s",P[4]['stage']:f"导出Σ{S['program_reported_sums']['export_wall_s']:.2f}s；gzip测量464.29s",P[5]['stage']:'未独立记录完整时长',P[6]['stage']:'仅有启动/返回观测'}
operations=['读任务/源码、场景门禁、实现两级QP','停车QP/诊断JSON修复、回归、四次调试','36次驱动回看、定时sleep、进度观察','分组独立验收的返回观察','报告/种子/时序检查、归档工具开发','中断恢复、21对象/2索引、全量恢复校验','tar、哈希、成员回读、最终答复']
evidence=['R:1641–1740','R:1740–2065','R:2065–2745','observed_program_transitions.json','R:2731–2847（含正式期工具开发）','R:2858–3042','R:3042–3080']
for i,p in enumerate(P):rows.append('|'+ '|'.join([p['stage'],wall_labels[p['stage']],f"{p['root_records']}/{p['guardian_records']}",f"{p['input_tokens']/1e6:.3f}M / {p['cached_input_tokens']/1e6:.3f}M",f"{p['output_tokens']:,} / {p['reasoning_output_tokens']:,}",operations[i],evidence[i],'用量实测；分期关联'])+'|')
text=f'''# M2b 四车实验 Codex 用量审计（只读）

**结论：高记录 token 主要来自长上下文的多次模型往返，并非数值脚本运行分钟数自动产 token。** 主模型本任务增量 **{main['total_tokens']:,}**；另有自动审批 guardian **{g['total_tokens']:,}**，记录层合计 **{total['total_tokens']:,}**。其中缓存输入 **{total['cached_input_tokens']:,}**，占输入约{cache_ratio:.1%}。这不是费用或订阅额度的换算结果；guardian是否按相同规则扣额度无法确认。

审计对象已匹配原四车任务、同一工作目录、输出M2b_fourcar_001与交付记录。**“MRIB”是否是用户对M2b的别称仍待确认**，本报告不把其他实验混入。边界为北京时间 **2026-11-01 21:50:13.786 → 11-02 07:46:00.619**。根会话`{root}`从10月30日即已存在，跨T1/M0/M1/M2a；这里只计三个M2b根turn ID。原任务R:1641、交付R:3077，R指[原会话]({base}:1641)。后续目录问答及本次审计不计入。

|范围|用量记录数|输入|其中缓存|输出|其中推理|记录总量|
|---|---:|---:|---:|---:|---:|---:|
|主模型|152|{main['input_tokens']:,}|{main['cached_input_tokens']:,}|{main['output_tokens']:,}|{main['reasoning_output_tokens']:,}|{main['total_tokens']:,}|
|自动审批 guardian|82|{g['input_tokens']:,}|{g['cached_input_tokens']:,}|{g['output_tokens']:,}|{g['reasoning_output_tokens']:,}|{g['total_tokens']:,}|
|记录层合计|234|{total['input_tokens']:,}|{total['cached_input_tokens']:,}|{total['output_tokens']:,}|{total['reasoning_output_tokens']:,}|{total['total_tokens']:,}|

**计量方法。** 以`token_usage_record`的thread_id、root_turn_id界定任务；去重response ID后，逐线程累计快照取有效增量，并与该条usage逐字段核对。234条均一致，未观察重置、重复或累计缺口；不累加token_count累计通知，也不把guardian继承的对话历史当成继承用量。历史记录均满足`total=input+output`，cached≤input、reasoning≤output；两子项不再次相加，cache_write_input全部为0。原CLI同版本导出的协议字段与算术证据见token_schema_definitions.json、usage_consistency.json。没有重建底层HTTP计费语义；234是用量记录数，不是经网络证明的HTTP请求数。根会话在交付时累计70,302,725，含早期任务，不能全归本实验。

|阶段|程序墙钟（已报告量）|主/审批记录|输入 / 其中缓存|输出 / 其中推理|主要操作|证据位置|置信／缺失|
|---|---:|---:|---:|---:|---|---|---|
{chr(10).join(rows)}

阶段 token 数由记录实测，**阶段归属按工具动作及启动/返回观察窗口关联推断，不是精确因果分摊**：提示输入包含此前历史；正式运行中也编码过归档工具。进程切换观察存在滞后。程序墙钟来自各summary/独立验收JSON，调试有并行，不能把上述Σ相加当经过时长；其余未测部分不编造。CSV保留整数计数，usage_increments.csv可逐条追溯。

**最高阶段与可避免项。** “正式运行/监看等待”关联 **16,759,008**（37.6%记录总量），是该分期口径最高项。另一个交叉统计：与纯等待工具相邻的68条主模型记录合计 **18,138,999**，与阶段表重叠，不能再加。主模型每次输入平均 **{S['root_input_per_response']['mean']:,.0f}**，范围92,401–360,489；反复送回历史是主因。可见35次sleep（申请合计1540秒）和66个含write_stdin的工具封装；正式驱动exec session 40671被回看36次，首次返回15:08:11Z、结束返回16:20:58Z（R:2067/2745；观察边界不是精确进程起止）。151个工具封装与152条主模型记录也不意味着151条shell命令。

没有证据支持“大量空stdout轮询”：31个纯write_stdin封装均有可解码stdout，其中1次没有新cycle行也不等于无变化。sleep返回主要只是耗时，仍造成模型再次参与。已识别的命令output字段约180,733字节；完整工具序列化输出466,733字节，含1个图片预览，最大173,883字节块不是纯终端文本，不能按base64长度折算token。较长文字输出主要为初次任务/源码读取约39–47KB，非把数百MiB预测日志全文喂给模型。重复的直接路径读取主要是小型状态/恢复日志；没有磁盘I/O跟踪，无法精确统计所有Python内部重读。相关机器证据见audit_summary.json、decoded_tool_output_metrics.json。

**重复工作与压缩。** 有四次调试运行、停车衔接修复、非有限数JSON导出修复、冻结/交付回归，以及中断后21空对象和2空索引的恢复。这些是实证额外工作，不能全称浪费。任务期间无新的根会话`compacted`标记；根的已知压缩发生在任务之前。两个新guardian初始化有compaction状态载入，不证明另行生成压缩摘要或额外计费，未加算。无主动spawn_agent/协调工具；82条guardian是自动审批，不是四车ROS控制器在调用LLM。未取得HTTP重试轨迹，不能证明所有重试或服务端隐藏用量都已记录；可见重试若产出本地usage，已包含于增量中。

**程序有没有调用LLM？** 冻结的17个运行源文件哈希匹配；静态导入未见OpenAI/Anthropic/HTTP客户端，核心为OSQP setup/solve、RK4及ROS2 DDS，驱动子进程只运行Python控制和验收。证据逐文件/行见program_code_audit.json。不是网络抓包证明，但可见执行路径未调用LLM。模型参与发生在Codex写代码、读返回、修复、检查与自动审批层。**普通终端运行相同脚本，且不接入LLM，不会仅因运行时间长继续产生模型token。** 若仍由Codex监看/审批，那是额外模型工作，而非数值脚本本身的例外。

**指标与版本边界。** 历史客户端是codex_vscode，CLI **0.164.0-alpha.11.1**；主模型gpt-6.1-sol/medium，guardian为codex-auto-review/low。原执行平台`{env['platform']}`，历史hostname未记录。当前安装扩展26.1021.22027及同版本CLI只能佐证，不替代原扩展版本（未知）。未读auth.json或密钥；历史限额报告plan_type=plus，确切认证方式未直接确认。窗口记录475000，不等于已记录UI上下文百分比。

历史5小时used_percent从20%到14%，reset不同；周窗口从8%到22%，reset相同。它们是账户快照，不能证明该任务独占下降，也不能和token固定换算。credits显示has_credits=false、balance="0"，不等于费用为零；无历史API费用/账单证据，故不报金额。用户实际看到哪一种指标仍未知。缺口恢复入口是原界面指标名称/截图、历史用量或账单导出、只含类型的登录信息，不需要凭据。另有2条NUL损坏行（1条在任务范围）；解码后的计量增量仍一致，不把坏行猜成一次调用。

**改进建议。** 保留必要的场景/协议/重积分和关键反例；用单一受限驱动脚本及阶段完成/失败通知替代定时sleep→模型→poll；为新实验开短上下文或交接摘要，状态写小型checkpoint；审批在既有权限内批量执行，不绕过门禁；源码按范围读取，状态只取摘要。归档采用原子持久写入、先核对再替换，可减少中断后的重复修复。这些能减少往返/上下文，但本地数据不足以预测具体订阅节省或费用。

审计只读原实验与会话，未重跑仿真、未改实验源文件、未安装统计插件；写入仅新审计目录（及临时离线schema输出日志）。完整版本/边界在environment_and_scope.json，计数一致性在usage_consistency.json。
'''
(A/'report.md').write_text(text)
write('measurement_definitions.json',dict(primary='effective increments of thread_token_usage, checked equal to per-response usage',not_http_request_count=True,total_identity='total_tokens = input_tokens + output_tokens for every inspected usage record',cached_input='reported cached input component; not added to total again',reasoning_output='reported reasoning output component; not added to total again',cache_write_input='all zero in task; nonzero billing semantics not established',stage_attribution='operation/observation time association; not exclusive causal token cost',guardian_billing='unknown whether debited identically or included in product quota',terminal_output_scope='decoded command output fields, separate from raw serialized tool blocks and image data',program_wall_scope='reported online/export/audit sums, not CPU time or task elapsed time'))
# Freeze only task-related metadata in the deliverable, plus one prior cumulative baseline per actor.
allU=[json.loads(x) for x in (A/'token_usage_detail.jsonl').open()];selected_keys={(u['thread_id'],u['response_id_hash']) for u in U};baseline=[]
for sid in dict.fromkeys(u['thread_id'] for u in U):
 first=min(u['timestamp'] for u in U if u['thread_id']==sid);prior=[u for u in allU if u['thread_id']==sid and u['timestamp']<first]
 if prior:baseline.append({**prior[-1],'baseline_only':True})
(A/'token_usage_detail.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in baseline+[u for u in allU if (u['thread_id'],u['response_id_hash']) in selected_keys]))
for name in ('event_metadata.jsonl','usage_records.jsonl','turn_settings.jsonl','task_events.jsonl'):
 p=A/name;items=[]
 for line in p.open():
  x=json.loads(line);t=x.get('timestamp') or ''
  if x.get('type')=='session_meta' or K['task_start_utc']<=t<=K['task_end_utc']:items.append(x)
 p.write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in items))
(A/'user_boundaries.json').write_text(json.dumps([x for x in read('user_boundaries.json') if K['task_start_utc']<=x['timestamp']<=K['task_end_utc']],indent=2,ensure_ascii=False)+'\n')
write('read_scope.json',dict(source_files=4,root_history_metadata_inspected_for_task_and_baseline=True,deliverable_pruned_to_task_and_prior_cumulative_baselines=True,experiment_large_logs_read=False,experiment_inputs_read='small environment/summary/validation/freeze JSON and 17 source files',credentials_read=False,simulation_runs=0,numeric_tests_rerun=0,plugins_installed=0,stopped_broadening_after_matching_sessions_and_usage=True))
assert sum(p['total_tokens'] for p in P)==total['total_tokens']
assert main['total_tokens']+g['total_tokens']==total['total_tokens']
assert len({(u['thread_id'],u['response_id_hash']) for u in U})==234
write('audit_checks.json',dict(stage_sum_matches=True,actor_sum_matches=True,unique_records=234,effective_increments_match_record_usage=True,no_double_count_cached_reasoning=True))
print('report written; task total',total['total_tokens'],'cached',total['cached_input_tokens'])
