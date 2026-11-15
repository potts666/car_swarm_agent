#!/usr/bin/env python3
"""Read the completed development evidence; never launch or modify a run."""
from collections import Counter
from fractions import Fraction as F
import gzip
import hashlib
import json
from pathlib import Path

from run_development import CASES,METHODS


def main():
    here=Path(__file__).resolve().parent;root=here/'results/pilot_032'
    destination=here/'development_metrics.json'
    assert not destination.exists(),'preserve existing analysis'
    status=json.loads((root/'status.json').read_text())
    assert status['passed'] and len(status['stages'])==30
    for stage in status['stages']:
        name='summary.json' if stage['layer']=='controller' else 'independent_acceptance.json'
        file=root/(stage['case']+'_'+stage['method'])/name
        assert hashlib.sha256(file.read_bytes()).hexdigest()==stage['evidence_sha256']
    metrics={};lines=['# 五组开发试运行报告','',
        '三例×五组共15次完整四车任务，均在P0 v2通过后执行，且逐次经控制器外验收。开发案例并非随机留出样本；没有正式批量、置信区间或论文修改。', '',
        '|案例|S-L秒|S-T秒|M-L秒|M-T秒|M-R秒|S-L减M-T秒|',
        '|---|---:|---:|---:|---:|---:|---:|']
    names=dict(asymmetric_baseline='不对称基准',tight_curve='紧曲线',exit_merge_dependency='出口合流依赖')
    for case in CASES:
        metrics[case]={}
        for method in METHODS:
            run=root/(case+'_'+method)
            summary=json.loads((run/'summary.json').read_text())
            acceptance=json.loads((run/'independent_acceptance.json').read_text())
            assert summary['passed'] and acceptance['passed'] and acceptance['full_task']
            assert acceptance['actual_steps']==summary['cycles'] and summary['remaining_budget']==0
            assert summary['optimizer_counts']['inherited_invalid']==summary['optimizer_counts']['program_errors']==0
            failures=Counter();dimensions=Counter();groups=Counter();numerical_status=Counter();changed=0
            with gzip.open(run/'execution.jsonl.gz','rt') as stream:
                for line in stream:
                    row=json.loads(line);opt=row['optimizer']
                    groups[max(map(len,row['groups']))]+=1
                    if not opt['accepted']:failures[opt.get('failure_layer','unspecified')]+=1
                    if opt.get('solver_called'):
                        dimensions[str((opt.get('numerical_variables'),opt.get('numerical_constraint_rows')))]+=1
                        numerical_status[opt['status']]+=1
                    changed+=bool(opt['changed'] and opt['accepted'])
            assert changed==summary['optimizer_counts']['changed']
            metrics[case][method]=dict(summary=summary,independent_acceptance=acceptance,
                rejected_proposal_layers=dict(failures),native_dimensions_histogram=dict(dimensions),
                native_status_histogram=dict(numerical_status),maximum_group_size_histogram=dict(groups),
                ack_participants=4,simultaneous_active_movers=1,
                evidence_sha256={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in
                    ('summary.json','independent_acceptance.json','execution.jsonl.gz','coordination_events.json')})
        times=[float(F(metrics[case][m]['summary']['T_all_s'])) for m in METHODS]
        lines.append('|'+names[case]+'|'+'|'.join(f'{v:.1f}' for v in times)+f'|{times[0]-times[3]:.1f}|')
    lines+=['','时间来自实际执行拍数×0.2秒，四车均达到共同停车目标且随后零输入不变保持。在线候选拒绝与任务失败分列；本次最终15次均完成，无继承分支失效或程序错误。', '',
        '|案例|运动收益 S-T−M-T秒|后续评价 M-L−M-T秒|M-T相对S-L缩时%|M-T能量|M-R能量|M-T能量降幅%|',
        '|---|---:|---:|---:|---:|---:|---:|']
    for case in CASES:
        data=metrics[case]
        t=lambda m:float(F(data[m]['summary']['T_all_s']))
        e=lambda m:data[m]['summary']['input_energy']
        lines.append(f"|{names[case]}|{t('S-T')-t('M-T'):.1f}|{t('M-L')-t('M-T'):.1f}|{100*(t('S-L')-t('M-T'))/t('S-L'):.2f}|{e('M-T'):.9f}|{e('M-R'):.9f}|{100*(e('M-R')-e('M-T'))/e('M-R'):.6f}|")
    lines+=['','SOCP固定m，不产生更短连接。M-T/M-R时间相同，能量差很小；不能宣称局部优化贡献了缩时或明显平滑收益。能量为实际执行的归一化a²+omega²积分，输入变化能量另见JSON。','',
        '|案例/方法|周期P50 ms|P95 ms|P99 ms|最大 ms|超过200ms %|改动/拒绝/真实锥调用|',
        '|---|---:|---:|---:|---:|---:|---:|']
    for case in CASES:
        for method in METHODS:
            s=metrics[case][method]['summary'];c=s['optimizer_counts'];q=s['cycle_wall_s']
            lines.append(f"|{names[case]}/{method}|{q['p50']*1000:.2f}|{q['p95']*1000:.2f}|{q['p99']*1000:.2f}|{s['cycle_max_s']*1000:.2f}|{100*s['over_200ms_fraction']:.3f}|{c['changed']}/{c['rejected']}/{c['native_solves']}|")
    road=min(metrics[c][m]['independent_acceptance']['continuous_road_lower_m'] for c in CASES for m in METHODS)
    gap=min(metrics[c][m]['independent_acceptance']['continuous_pair_lower_m'] for c in CASES for m in METHODS)
    lines+=['',f'独立验收跨15次的最小道路余量下界{road:.9f}m，六对车身净空下界{gap:.9f}m（要求0.5m）。这些下界对应声明的源域/模型及实际输入。','',
        '周期从状态封存到共同提交，包含选择、优化、证书和消息，不含归档I/O；拒绝周期也计入。物理时钟计算时暂停，每任务仅一次墙钟测量，不能推断实时性。环境见 results/preservation_034/environment.json。准备冷启动约77.18/78.09/78.69秒；重用原图/积分器的前置成本另保存在source_rebuild_005/stages.json。','',
        '各次完整Ti、等待、路程、停车数、物理交接速度/轮角、输入变化、真实消息数/字节及求解维度、状态、拒绝层次在 development_metrics.json 和原逐拍日志。消息是实际JSON序列化量，未计DDS/网络头。全体4车ACK，活动输入优化仅1车，分组最大3车；没有并行或分组加速证据。','',
        '主S在物理路线入口/出口停车；M在路线内部移动交接，车间换权仍停车。两种停止划分及仅资源接口停车的更强参考见 resource_only_stop_reference.json；尚未做额外闭环敏感性，故本表收益仅相对于声明的主S基线。','',
        '原14段190.2秒见证另行重验通过，比本开发库更快。当前结果反映保守同几何候选库内的机制比较，不能声称超过全部历史方案、物理全局最优或任意场景普遍收益。库内精确最短路参照保存在pilot_objects_011，各场景初始策略距S库最短30/32/26拍。','',
        '失败历史保留：007/008数值AlmostSolved未接受；025首轮运行第47拍内容哈希类型错误，026诊断，修正并重新过P0后才执行本轮032。不能将15次最终通过报告为整个开发过程零错误。','',
        '门禁与来源：P0_GATE_v2.json；原件及归档保留校验：results/preservation_034/checks.json；定理前提：theorem_premises.md/json；未冻结正式参数：pending_parameters.json。原始证据：results/pilot_032/status.json及各子目录。']
    destination.write_text(json.dumps(dict(development_only=True,formal_batch=False,full_runs=15,metrics=metrics),ensure_ascii=False,indent=2)+'\n')
    (here/'development_report.md').write_text('\n'.join(lines)+'\n')
    print('15 complete runs and 15 independent acceptances summarized; source evidence hashes checked')


if __name__=='__main__':main()
