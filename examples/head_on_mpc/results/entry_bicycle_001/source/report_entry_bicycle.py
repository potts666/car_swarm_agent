"""Measured tables, oriented-body trajectories, phase timeline and motion animation."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon, Rectangle
import numpy as np

from bicycle_core import CARS, SIGN, corners
from verify_entry_bicycle import load_rows, read_jsonl, vectors

COLORS={'A':'#1768ac','B':'#d2691e'}


def draw_map(ax,scene):
    ax.set_facecolor('#eee9e1')
    for a,b,c,d in scene['road_rectangles']:
        ax.add_patch(Rectangle((a,c),b-a,d-c,color='white',ec='#999',lw=.7))
    ax.add_patch(Rectangle((-10,-1.4),20,2.8,color='#e6d9a8',alpha=.7))
    for x in (-10,10):
        ax.axvline(x,c='#555',ls='--',lw=.7)
        ax.text(x,5.8,f'entrance/exit {x}',ha='center',fontsize=6)
    for c in CARS:
        sign=SIGN[c]
        zone=(-28,-18,1.5,5) if c=='A' else (18,28,-5,-1.5)
        a,b,y,z=zone
        ax.add_patch(Rectangle((a,y),b-a,z-y,fc=COLORS[c],alpha=.12))
        ax.text((a+b)/2,(y+z)/2,f'{c} shared wait/bay',ha='center',fontsize=6)
        ax.plot(scene['goals'][c][0],0,'*',c=COLORS[c],ms=9)
        ax.text(scene['goals'][c][0],-2.,f'{c} goal',ha='center',fontsize=7)
        ax.plot(-sign*scene['request_rear_axle_abs_x'],0,'|',c=COLORS[c],ms=12)
    ax.set(xlim=(-51,51),ylim=(-7,7),xlabel='rear axle x [m]',ylabel='y [m]')
    ax.set_aspect('equal',adjustable='box')


def render_report(directory):
    directory=Path(directory);scene=json.loads((directory/'scene.json').read_text())
    summary=json.loads((directory/'summary.json').read_text())['results']
    validation=json.loads((directory/'validation.json').read_text())
    rows={p:load_rows(directory/p/'actual.csv') for p in ('S1','S2')}
    motions={p:{c:vectors(rows[p][c]) for c in CARS} for p in rows}
    metrics={p:validation['policies'][p]['metrics'] for p in rows}
    times={p:np.array([float(r['time_s'])-scene['epoch_s'] for r in rows[p]['A']]) for p in rows}
    fig,axes=plt.subplots(2,1,figsize=(14,6),constrained_layout=True)
    for ax,p in zip(axes,rows):
        draw_map(ax,scene);ax.set_title(f'{p}: {"entrance mutex" if p=="S1" else "encounter and recovery"}')
        for c,z in motions[p].items():
            ax.plot(z[:,0],z[:,1],c=COLORS[c],lw=1.7,label=c)
            for i in np.linspace(0,len(z)-1,9,dtype=int):
                body=corners(z[i],scene['vehicle'])[0]
                ax.add_patch(Polygon(body,fill=False,ec=COLORS[c],alpha=.6,lw=.7))
                ax.arrow(z[i,0],z[i,1],1.6*np.cos(z[i,2]),1.6*np.sin(z[i,2]),color=COLORS[c],width=.025,head_width=.3,length_includes_head=True)
        ax.legend(loc='upper right',fontsize=7)
    fig.savefig(directory/'trajectories.png',dpi=160);plt.close(fig)
    # Initial scene and the four individually gated motions.
    fig,axes=plt.subplots(2,1,figsize=(14,6),constrained_layout=True)
    draw_map(axes[0],scene);axes[0].set_title('Shared serialized initial scene; rear axle and rotating footprint')
    for c in CARS:
        z=np.array(scene['initial'][c]);axes[0].add_patch(Polygon(corners(z,scene['vehicle'])[0],fc=COLORS[c],alpha=.5))
        axes[0].arrow(z[0],z[1],2*np.cos(z[2]),2*np.sin(z[2]),head_width=.4,color=COLORS[c])
    draw_map(axes[1],scene);axes[1].set_title('Single-car gate: approach, forward entry, reverse exit, forward return')
    for path in (directory/'single_vehicle').glob('*.csv'):
        if path.name.startswith('original'):
            continue
        z=vectors(load_rows(path)['A']);axes[1].plot(z[:,0],z[:,1],label=path.stem)
    axes[1].legend(fontsize=6,ncol=4)
    fig.savefig(directory/'scene_and_single_vehicle.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(14,5),constrained_layout=True)
    stages=sorted({r['stage'] for p in rows for c in CARS for r in rows[p][c]})
    palette={s:plt.get_cmap('tab20')(i%20) for i,s in enumerate(stages)}
    for j,(p,c) in enumerate((p,c) for p in rows for c in CARS):
        rr=rows[p][c];start=0
        for i in range(1,len(rr)+1):
            if i==len(rr) or rr[i]['stage']!=rr[start]['stage']:
                a=float(rr[start]['time_s'])-scene['epoch_s'];b=float(rr[i-1]['time_s'])-scene['epoch_s']
                ax.broken_barh([(a,max(scene['sample_s'],b-a))],(j-.35,.7),facecolors=palette[rr[start]['stage']])
                if b-a>3:
                    ax.text((a+b)/2,j,rr[start]['stage'],ha='center',va='center',fontsize=5,rotation=20)
                start=i
        for e in read_jsonl(directory/p/'events.jsonl'):
            if e['kind'] in ('mutual_blocking_detected','role_selected','release'):
                t=e['time_s']-scene['epoch_s'];ax.plot([t,t],[j-.42,j+.42],c='black',lw=.8)
    ax.set(yticks=range(4),yticklabels=['S1 A','S1 B','S2 A','S2 B'],xlabel='simulation time [s]',title='Executed phases; black marks: detection / role selection / release')
    fig.savefig(directory/'timeline.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(4,2,figsize=(13,10),sharex='col',constrained_layout=True)
    quantities=[('v',-scene['vehicle']['v_rev'],scene['vehicle']['v_fwd'],'v [m/s]'),
        ('delta',-scene['vehicle']['delta_max'],scene['vehicle']['delta_max'],'delta [rad]'),
        ('omega',-scene['vehicle']['omega_max'],scene['vehicle']['omega_max'],'omega [rad/s]'),
        ('a',scene['vehicle']['a_min'],scene['vehicle']['a_max'],'a [m/s²]')]
    for col,p in enumerate(rows):
        axes[0,col].set_title(p)
        for ax,(key,lo,hi,label) in zip(axes[:,col],quantities):
            for c in CARS:
                ax.plot(times[p],[float(r[key]) for r in rows[p][c]],c=COLORS[c],label=c,lw=1)
            ax.axhline(lo,c='red',ls='--',lw=.7);ax.axhline(hi,c='red',ls='--',lw=.7)
            ax.set_ylabel(label);ax.grid(alpha=.2)
        axes[-1,col].set_xlabel('simulation time [s]');axes[0,col].legend()
    fig.savefig(directory/'motion_limits.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,5),constrained_layout=True)
    patches={};labels={};arrows={}
    for ax,p in zip(axes,rows):
        draw_map(ax,scene)
        for c in CARS:
            ax.plot(motions[p][c][:,0],motions[p][c][:,1],c=COLORS[c],alpha=.25,lw=.7)
            patch=Polygon(corners(motions[p][c][0],scene['vehicle'])[0],fc=COLORS[c],ec='black',lw=.7)
            ax.add_patch(patch);patches[p,c]=patch
            line,=ax.plot([],[],color='black',lw=1.5);arrows[p,c]=line
        labels[p]=ax.text(.01,.98,'',transform=ax.transAxes,va='top',fontsize=7)
    last=max(t[-1] for t in times.values());frames=np.arange(0,last+.6,.6)
    def update(t):
        for ax,p in zip(axes,rows):
            i=min(int(np.searchsorted(times[p],t)),len(times[p])-1);descriptions=[]
            for c in CARS:
                z=motions[p][c][i];patches[p,c].set_xy(corners(z,scene['vehicle'])[0])
                patches[p,c].set_facecolor('#cc4422' if z[3]<-.025 else COLORS[c] if z[3]>.025 else '#999999')
                arrows[p,c].set_data([z[0],z[0]+2*np.cos(z[2])],[z[1],z[1]+2*np.sin(z[2])])
                descriptions.append(f'{c} {rows[p][c][i]["stage"]} v={z[3]:.2f} delta={z[4]:.2f}')
            labels[p].set_text(f'{p}  t={t:.1f}s   '+ ' | '.join(descriptions))
        return list(patches.values())+list(labels.values())+list(arrows.values())
    animation=FuncAnimation(fig,update,frames=frames,interval=60,blit=False)
    animation.save(directory/'motion.gif',writer=PillowWriter(fps=10),dpi=85);plt.close(fig)
    lines=['# 双车单通道：入口互斥预防 vs 相遇后恢复（单组离线实测）','',
        '实验使用同一份序列化物理场景、同构车辆与共同底层控制；两辆车各自的原目标在S1/S2完全一致。只运行A近、B远这一组，不作泛化成功率推断。',
        '', '## 方法、地图与适用范围','',
        '执行模型为以后轴为参考的五状态运动学自行车，RK4逐0.02 s积分；控制0.2 s，预测32步。带转角、转角速率、速度和加减速约束。车身为前悬0.8、轴距2.7、后悬1.0 m的旋转矩形。最小转弯半径约4.942 m，入位圆弧选6 m。参数来源及全部具体数值见[变量表](variables.md)。没有轮胎侧偏动力学、实车标定或换挡停顿。',
        '',
        '共同控制采用停车—受限转向—圆弧行驶的运动原语，加速度梯形时标按实际距离生成。没有预设y日程、状态覆盖或独立横移。不调用旧纵向QP；这是替换执行模型所需的最小确定性控制入口，不是新的DMPC控制器。加速度允许分段变化，本轮没有jerk约束。',
        '',
        '原make_scene地图上的同类圆弧入位被道路车身验收拒绝，证据保存在single_vehicle/original_map_rejected_maneuver.json及CSV。该结果只证明所测机动失败，没有穷尽搜索，不能推断原地图物理不可达。新公共地图另存为scene.json：保留20×2.8 m通道，在两端增加明确对称的转弯区，两策略共享；结果只适用于此地图。旧地图与历史结果均未覆盖。',
        '',
        'S1在后轴通过±34 m申请线时原子申请唯一owner，申请时间、ID排序；未获许可者先制动，再驶到后轴±30 m入位起点，依自行车圆弧停车在旁侧等待区。准入保护区为主路x∈[−18,18]；外部合流通过额外释放保护覆盖：通过车尾缘清空±26 m并加0.5 m裕度，同时完整重演双方余下机动至各自目标，才释放。之后双方执行该见证轨迹并停在不同目标，不将有限预测时域无碰撞当作永久安全。',
        '',
        'S2正常阶段将相向车头间可用纵向空间平均分配为双方制动约束，前进目标仍为原任务；实际运动在0.6 m车头净距处停住，名义安全阈值0.5 m不变。此共享配对安全过滤是确定性控制假设，不是DMPC上一预测交换求解。只在有当前验收预测时调用旧BlockingDetector；正常停车不强制触发，持续停滞与车挡证据形成后才选角。双候选从同一真实检查点完整退出，排序沿用E3/E4的退出时间、倒车距离、ID规则。旧E3/E4的EXIT→PASS→RELEASE→RETURN阶段语义保留，旧横移数值执行和E4求解保护未直接调用；新释放改用双方完整未来轨迹见证。',
        '',
        '这是离线仿真：候选期间模拟时间冻结，selection_wait_wall_s单列。主规划为原语与轨迹生成，优化求解器调用数为0；不把它虚报为QP求解。协议消息为单进程内按compact UTF-8 JSON编码的估计，含双方发给管理端的当前预测及管理端角色消息，不代表真实网络流量。S1只交换许可协议，不为统一成本发送预测。',
        '', '## 逐策略实测','',
        '|策略|状态|双方完成模拟s|A完成s|B完成s|总路程m|倒车m|静止等待车秒|策略进程墙钟s|候选墙钟s|',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for p,s in summary.items():
        m=metrics[p]
        lines.append(f'|{p}|{s["status"]}|{s["T_complete_sim_s"]}|{s["completion_per_car_s"].get("A")}|{s["completion_per_car_s"].get("B")}|{m["total_distance_m"]:.3f}|{m["total_reverse_distance_m"]:.3f}|{m["waiting_sim_s"]:.3f}|{s["total_policy_wall_s"]:.3f}|{s["selection_wait_wall_s"]:.3f}|')
    lines+=['','|策略/车|二维距离m|纵向累计m|倒车m|互斥排队静止s|互阻检测静止s|退让停驻s|恢复中对方保持s|等待区占用s|',
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for p,m in metrics.items():
        for c,v in m['per_car'].items():
            w=v['waiting_sim_s']
            lines.append(f'|{p}/{c}|{v["distance_m"]:.3f}|{v["longitudinal_distance_m"]:.3f}|{v["reverse_distance_m"]:.3f}|{w["mutex_queue"]:.2f}|{w["mutual_blocking_detection"]:.2f}|{w["retreat_park"]:.2f}|{w["recovery_peer_hold"]:.2f}|{v["waiting_region_occupancy_s"]:.2f}|')
    lines+=['','等待统计仅在|v|≤0.025 m/s时按互斥阶段互斥分类；转向停车属于相应入位阶段，行驶阶段不计静止等待。车秒为两车求和，不等于总工期。占用统计每0.02 s检查旋转车身与固定等待区相交，属于物理占用，与策略资源保留分别报告；未作不同量纲加权总分。',
        '', '|策略|最小实际净空m|最小预测净空m|通道实际有车s|通道许可保留s|消息数|估计payload bytes|',
        '|---|---:|---:|---:|---:|---:|---:|']
    for p,m in metrics.items():
        s=summary[p]
        lines.append(f'|{p}|{s["minimum_actual_gap_m"]:.9f}|{s["minimum_predicted_gap_m"]:.9f}|{m["channel_physically_occupied_s"]:.2f}|{s["channel_reserved_s"]:.2f}|{m["communication"]["messages"]}|{m["communication"]["payload_bytes"]}|')
    lines+=['','S2没有许可锁，因此许可保留时间为0；其通道物理占用和互阻检测等待均非零，不能称为空间成本为零。净空采用有向矩形SAT重叠判断和边段欧氏最短距离；道路按矩形并集与车身多边形裁剪面积检查，跨接缝不只查角点。道路残差单位m²，其余速度/输入/角度残差保留原单位；数值容差1e−5。安全只经过0.02 s加密采样，未建立连续时间安全证明。',
        '', '## 阶段事件与计算','', '|策略|事件|车辆/角色|模拟s|','|---|---|---|---:|']
    for p in rows:
        for e in read_jsonl(directory/p/'events.jsonl'):
            if e['kind']!='motion_stage_change':
                lines.append(f'|{p}|{e["kind"]}|{e.get("car",e.get("role",""))}|{e["time_s"]-scene["epoch_s"]:.2f}|')
    lines+=['','|策略|计算分项|次数|合计墙钟s|平均s|最大s|超过h次数|','|---|---|---:|---:|---:|---:|---:|']
    for p,m in metrics.items():
        for name,v in m['timing'].items():
            lines.append(f'|{p}|{name}|{v["count"]}|{v["total_s"]:.4f}|{v["mean_s"]:.6f}|{v["max_s"]:.6f}|{v["over_h"]}|')
    lines+=['','cycle_total包含对应周期计算；候选选择和诊断可能包含在周期总墙钟中，各行不能重复相加。主规划统计为轨迹构造/滚动预测，不含独立几何诊断；首次公共运动计划构造耗时另见summary。全部被拒候选也进入候选统计；优化调用、重试均为0。独立重读验收、作图和动画另计进程总墙钟，不计模拟完成时间。',
        '', '## 实测解释','']
    s1,s2=summary['S1'],summary['S2']
    if s1['both_completed'] and s2['both_completed']:
        delta=s2['T_complete_sim_s']-s1['T_complete_sim_s']
        winner='S1' if delta>0 else 'S2' if delta<0 else '相同'
        lines.append(f'本组成功配对：{winner}完成更快；S2−S1={delta:.3f}模拟秒。S1={int(s1["both_completed"])}/1、S2={int(s2["both_completed"])}/1，均为这一固定场景的完成结果，不是泛化成功率。')
        lines.append('机制解释属于基于轨迹的推断：S1提前让后到车辆驶入等待区；S2先进入通道、停滞检测，再倒车退出到同一个等待入位点，增加了通道占用、倒车路程和检测等待。两者后续都执行同样的圆弧入位/返回和原目标终端口径。该单组中若S1更快，应将S2定位为未预防互阻时付出更高代价的恢复方案；结果不支持DMPC优越性结论，也没有异构、多参数或交换到达顺序的证据。')
    else:
        lines.append('至少一个策略未完成；不使用提前终止时间作为完成时间，不计算配对完成时间差。失败原因及截断轨迹全部保留。')
    lines+=['','## 验收与复现','',f'独立重读验收总结果：{validation["passed"]}。实际与预测均重放五个运动分量；复查输入界、换向零点、车身尺寸、道路、净空、时间连续、两策略初态配置、许可/释放、候选检查点、完整返回及双方原目标。详细残差与指标重算见validation.json；单车四项结果见single_vehicle/checks.json。',
        '', '![公共场景与单车验证](scene_and_single_vehicle.png)','![两策略全局轨迹及航向](trajectories.png)',
        '![阶段时间线](timeline.png)','![速度转角转角速率及加速度界](motion_limits.png)','![旋转车身运动动画](motion.gif)',
        '', '在examples/head_on_mpc目录运行（输出目录必须不存在）：','', '```bash',
        'OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p test_entry_bicycle.py -v',
        'OPENBLAS_NUM_THREADS=1 python3 scripts/run_entry_bicycle.py --output results/entry_bicycle_rerun',
        'OPENBLAS_NUM_THREADS=1 python3 scripts/verify_entry_bicycle.py --run results/entry_bicycle_rerun','```',
        '', '未检查：连续时间安全证明、轮胎侧偏、真实网络与实时调度、实车跟踪误差。风险：控制采用停车切换曲率的保守运动原语，地图已明确拓宽；结论限于该配置和该共同控制器。','']
    (directory/'report.md').write_text('\n'.join(lines))
