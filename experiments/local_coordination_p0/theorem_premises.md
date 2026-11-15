# 三项定理逐前提矩阵

原基准核对保留在 JSON 的 baseline_review 及 results/document_history_033/。当前状态只针对受限开发实现。

|ID|理论前提及位置|代码实现|核对结论与边界|
|---|---|---|---|
|A01|共同模型；统一§1式(1)—(3)；运动§1式(1)—(2)|pilot_control.py、socp_anchor.py|开发实现匹配：固定h=.2/N32/L2.7/五状态/a,omega实际ZOH；车身rear1/front3.5/halfwidth.9，五组共同配置。|
|A02|速度约束；统一§1：0≤v≤2|pilot_control.py:advance_pose；accept_execution.py|整拍认证：精确有理数v节点0至2、delta±.5、a/omega±1；每拍线性v/delta的极值在端点；没有沿用历史倒车容差。|
|A03|精确流或认证外包；统一§1、§3.2式(12)；运动§5|source_rebuild_005/work/.../route_zoh_certificate.py|区间外包：Q=10^20向外舍入、Taylor三角、Simpson余项；控制器4子段，验收8子段；二者共享积分原语，几何与预算独立。|
|A04|共同原点与版本；统一§1、§7|pilot_control.py:commit_token|内容绑定：实际history、输入、域phase、接续/策略版本、期限、权限和预算绑定；四车同版ACK后提交；理想进程内可靠消息。|
|A05|目标停车集合；统一§1；运动§10式(43)|run_pilot.py；accept_execution.py|精确停车保持：共同目标位置/航向内界、v=delta=0；最后零输入是解析不变保持。没有放宽历史数值停车容差。|
|A06|初态认证与非目标默认出口；统一§1—2；运动§9—10|rebuild_sources.py；prepare_pilot.py|对象重建通过：原20模板及图独立重建；三个开发案例重新认证源域和所有默认出口；在线从共同可实现策略出发。|
|S01|道路凸区契约；统一§3.1、§3.3|cross_stop.py；pilot_control.py:execute|连续区间契约：逐小段车身角点区间及弯曲膨胀后整体属于一个道路矩形；不是并集面积抽样。|
|S02|全部六对含组外承诺；统一定理一、式(9)|moving_objects.py:pair_partition；pilot_control.py:execute|六对覆盖：三车组3内3外分配、拆分和全车ACK；活动车/静止同伴的全部六对连续投影间隔检验。|
|S03|弯曲预算；统一式(10)—(12a)|test_theory_checks.py；cross_stop.py|有理上界：Mc=15有理上界核对，子段Mc*Delta^2/8与向外积分外包分列；安全净空.5另列。|
|S04|权限全过程覆盖；统一§3.3|pilot_control.py:execute；accept_execution.py|连续权限覆盖：共享一车活动规则；等待车保护区、释放车不回入；换权必须到实际接收域。未实现兼容多车并行。|
|S05|全源域安全；统一式(5)；运动式(18)、(24)—(25)|cross_stop.py；theory_checks.py|限定域认证：全源位置/航向域认证，粗节点v/delta精确0，内部移动域由输入精确流像定义；不是任意五状态开域。另有含v/delta误差的反馈局部例证。|
|R01|认证运动接收域及延续；运动§4式(18)、§5|moving_objects.py；pilot_control.py|开发全程接续：实际输入链每个窗口绑定E、K和后继；内部允许非零v/delta；数值成员检验仅用于已认证内部盒及继承分支。|
|R02|全状态接缝；运动§1式(3)|pilot_control.py:commit_token；accept_execution.py|实际接缝：v、delta精确相等，实际pose连续积分，不重置名义位置。P0包含换权后新车首拍。|
|R03|32拍尾段迁移；运动定理2式(36)|pilot_control.py:shift_plan|显式K移位：移去实际已执行首拍、追加phase+32参考K；仅在实际停车尾部合法时补零。6456窗口静态核对及闭环逐拍重演。|
|R04|接收阶段标签同步更新；运动§7|pilot_control.py:commit_token|接收阶段同步：接收phase=min(phase+32,m)，剩余期限和后续时间值一起迁移；receiver_phase损坏核对拒绝。|
|R05|新初态继承实际尾部；运动定理2证明|pilot_control.py；accept_execution.py|实际尾部继承：history/phase/v0/delta0与实际输入链绑定；保留预测外包和锥余项预算。|
|R06|SOCP实际输入增量；统一§4式(13)—(24)；运动§7|socp_anchor.py|真实SOCP：Clarabel0.11.1，64个归一化实际输入增量+t/s，含SOC和旋转二次余项锥；实际维度及约束行数记录。|
|R07|认证gamma、Jacobian和Hessian界；统一式(15)、(17)、(25)—(28)|socp_anchor.py:derivative_bounds|保守解析界：使用理论全局Gp/Gc/P1/P2/Hp/Hc有理上界；零近似Jacobian加认证导数误差，比较保守。|
|R08|显式零增量锚点可行；统一引理2；运动§7|socp_anchor.py；final_conic_021/checks.json|显式锚点：求解前精确验零增量；正盒调用真实锥求解器；边界唯一锚点和恒输入最小能量属于解析预消元，单列非数值调用。|
|R09|接收盒包含及边界锚点；统一式(19)—(23)；运动式(28a)|pilot_control.py:receiver_room/admit_delta|内部盒或继承像：以SE2逆像界证明正接收盒包含；区间不足时rp=rpsi=0保留原锚点；没有任意放宽。|
|R10|合并、拆分、换权；统一§5.3；运动§7|check_runtime.py；pilot_control.py:commit_token|实际组变化及换权：两条P0各有pair→三车→拆分，依赖转交和新持权车实际首拍；消息串行仿真，不代表真实网络并行。|
|R11|拒绝后旧分支仍执行；统一§5.2、§7；运动§10|check_runtime.py；pilot_control.py:optimize|拒绝续行：七类非法提案拒绝后旧输入执行；数值提案拒绝也计入周期。继承失效与程序错误不能当普通拒绝。|
|F01|每拍有限提案后必执行；统一§7；运动定理3|run_pilot.py|开发时钟实现：每拍最多一次SOCP、有限验证与ACK后执行；求解限时5s/300迭代；理想物理时钟暂停计算。持续时钟未实现。|
|F02|物理可实现的C或U_pi；运动§9式(38)、(40a)|prepare_pilot.py；run_pilot.py|可重算U_pi：原C2140/R2200由对象复算；开发U_pi为共同S可行策略，初值1301/1305/1295拍；C只作独立库内参照。|
|F03|当前连接时长不超过32拍；统一§2；运动§7|moving_objects.py；pilot_control.py|粗连接展开：粗连接可超过32拍；逐拍移动接收阶段与N32窗口展开后每个承诺段<=32，不能把224拍粗边直接当理论单段。|
|F04|在线替换预算不增加；运动式(41)—(42)及局部版本|run_pilot.py|替换守卫：仅m+U_pi(target)<=当前源预算的连接可选，策略替换后重算同版U且各节点不增；每真实执行拍减1。|
|F05|默认策略与预算同版；运动定理3及§9.3命题4|run_pilot.py；accept_execution.py|同版见证：每次连接保存policy、U和内容哈希；验收从已引用对象和策略重建，不只看递减列。|
|F06|真实到期状态和预算归零；运动式(43)|run_pilot.py；accept_execution.py|到期物理验收：到粗连接到期必须实际落入目标域且v/delta0；全任务预算0须对应四车目标与不变保持。完整运行观察另见试运行汇总。|
|D01|五组公平条件；设计§4—5|prepare_pilot.py；run_pilot.py|受限公平对照：共同地图/几何/物理界/安全守卫/初始S策略；S只在声明物理路线节点增加v=0，delta不强制额外归零；所有组都采用同一串行活动规则。|
|D02|L组时间目标与推进守卫；设计§4.2；运动式(42)|run_pilot.py|排序与守卫分离：L排序m、T排序m+U；均通过相同可实现守卫。初始策略距S库最短30/32/26拍，不人为删除无改善任务。|
|D03|M-R对应关闭局部SOCP；设计§5|run_pilot.py|固定时长消融：M-R与M-T同高层/候选/初始策略，仅关闭SOCP；在线不搜索更短m，时间相同不能写成SOCP缩时收益。|
|D04|三案例15次先行门槛；设计§3.3、§9.1|P0_GATE_v2.json；run_development.py|P0后开发放行：v1试运行哈希类型缺陷保留，修正并重过P0后v2门禁；仅3x5开发运行，逐次独立验收，禁止正式批量。|
|D05|原始式(9)排版；统一式(9a)、(9b)|theory/；theory_review.md|原文排版备注：原式9a/b缺加号；实现使用投影+epsilon+d/2<=边界。原文不修改。|
