# 有符号ZOH与接收包含：适用域与计算证据

本文件是本轮方法的条件性推导，数值检查列在results/；不以两个实验推导普遍可行性。

## 模型、源范围和导数

固定参考方向s来自MODEL_FRAMES数据。p_dot=s*v*e(theta)，theta_dot=v*tan(delta)/L。v可为负；通过v=0时所有表达连续。a、omega为一拍常值。v和delta在拍内仿射，故拍内物理及方向矩形由两个端点和输入条件覆盖。静止合同v=a=0给出整拍p/theta保持，不强行冻结合法轮角变化。

输入增量坐标采用64维Euclidean范数。任一步的速度和轮角输入灵敏度≤beta=h*sqrt(32)，向上有理取整；据此扩张名义端点并与原物理/阶段合同相交，形成每步V_abs和D_abs。实际Jacobian使用有符号v：

`d(theta_dot)=tan(delta)/L*dv+v*sec(delta)^2/L*d(delta)`。

令k=tan(D_abs)/L、kd=sec(D_abs)^2/L、kdd=2*tan(D_abs)*sec(D_abs)^2/L（有理向上界）。一阶航向的增量导数≤beta*(k+V_abs*kd)，二阶航向≤beta²*(2*kd+V_abs*kdd)。位置一阶积分由beta+V_abs*P1控制，二阶由2*beta*P1+V_abs*(P2+P1²)控制。derivatives.local_profiles在每拍解析积分这些正系数多项式，生成Gp/P1/Hp/P2；车角系数再加入RB。负速度只通过V_abs进入这些界，不改变真实变分符号。阶段等式限制了可行扰动子空间，正系数界在该子空间上仍成立。

每步5×7区间变分由sources/certify_interfaces.interval_jacobian生成，使用显式direction_sign；32拍复合链式区间矩阵，原行保留区间半宽范数和二阶Taylor项。区间Jacobian覆盖信赖中心的一阶导数，邻域变化由局部Hessian余项覆盖，不能把中心Jacobian当作整个邻域的常数导数。

有符号全物理域|v|≤2、|a|,|omega|≤1、|delta|≤.5、|theta|≤1下，既有integration_bounds.json的航向/位置四阶界84/256及车角二阶界15保留：均由绝对值不等式取得。连续包络使用15*(h/sub)^2/8，控制器sub=4、独立几何sub=8；另扣既有TUBE=.05000001。变化轮角按有理区间Simpson传播，恒定轮角使用精确圆弧ZOH。公式保留符号；过零时也不能用绝对行程代替真实位移。

精确核对：delta=omega=theta=0，S=h*v0+h²*a/2，x=s*S、y=theta_end=0；Jacobian中y_theta=s*S、theta_delta=S/L、theta_omega=(v0*h²/2+a*h³/3)/L。后两式在负速、零速、跨零速整矩形上都是v0/a的仿射函数；source_range_checks.json以矩形顶点的精确极值核对区间包含，证明这些有符号条目在整个矩形覆盖。完整其它条目由区间变分算法给出，未以有限差分代替认证。适用源nu/delta精确测得；源范围数学检查不是具有速度传感噪声的闭环鲁棒性证明。

## 接收盒、K与后继

统一入口从已认可完整策略取得N拍末端区间，确定有理中点参考状态，从该点重新传播K及保留后续至首个目标域H>N+1。参考流数值误差由区间包含；不能直接将历史整区间的不确定性再当作独立误差相加。K让v/delta在N+1精确回到旧控制的合法后继值，通常不等于零。

对接收半径(rP,rT,rV,rD)，q=k*rV+V*kd*rD。控制误差在K拍按(1-t/h)衰减，航向误差≤rT+q*(t-t²/(2h))；位置L1误差由C=3/2控制，得到method_spec.md的Ep/Et。其后相同控制与相同v/delta给出SE(2)等变性：位置误差≤Ep+C*L_rem*Et，车角误差≤Ep+(C*L_rem+RB)*Et。L_rem用h*(|v0|+|v1|)/2逐拍累加，是跨零速时保守的实际行程上界。

目标位置/航向限制、参考全段连续道路/六对净空/权限余量、三角适用范围共同构成A*r≤b；速度轮角及K输入给出额外帽。先以r0覆盖旧名义末端区间，再将b-A*r0以固定均分规则分配；不能覆盖则拒绝。零物理帽保留为等式，域维数据实记录。results/zero_successor_checks.json对7个源实际传播零增量32拍、K和保留后续，不只读取synthesized字段。

## 受理和一拍继承

所有命令用Fraction保存，输入增量采用固定1e-12网格。静止等式块采用精确投影和保和最近网格舍入，且检查相对原始物理尺度提案的总误差≤4e-12；原行、真实连续ZOH、接收/K、后继域、同版真实后续、固定时长预算、32拍代价非增全部受理后才可四车共同提交。AlmostSolved不表示配置精度达标，仍必须经过这些精确检查。

共同提交包含同一source snapshot/history、权限版本、完整接收事件链及4份计划ID。执行一拍使预算B_next=B_old-1；拒绝时原见证和原事件链不变。这个归纳依赖已保存可执行策略，不能宣称未来任意SOCP必有可行内点。历史证明、当前地图证书和此次执行观察分别陈述。
