# dynamic_groups_002 当前交接（2026-11-14）

目标：扩大旧保持车活动，优先4/3车，真实改物理分离面/资源时段，C-new与L-new连续两轮本车优化，R-old0调用，完整安全与完成证据。预算最多16；solver_calls.json当前[]，尚未消耗。历史/论文只读，无Git提交。技能ponytail已读取，未启动任何子agent。

首读包已完整阅读：/home/sunflower/Downloads/DG_DMPC_experiment_002_2026-11-14；包14文件哈希和5类代数/625期限检查通过。001真实目录同级dynamic_groups_001，默认零调用复核通过。全部来源哈希与001结果在source_inventory.json。

method_spec.md/execution_plan.md先于实现保存。实现复用001区间/ZOH/导数/锥/量化/接收，新增progress.py、proposals.py、显式连续物理面contracts.py、支持旧实际源到新锚点原子迁移的protocol.py、worker_io.py实际JSON Pipe字节。

零调用初扫描precheck.py已完成：原起步进度.018745<η=.02，无新增有效活动；同一三点解析加速度校正保总冲量与一阶时矩、保直线段末端全部状态，修正后首个池最优A=3在41拍，活动A1/A2/B2，原保持A1新增，A1删除等待16=8+8拍。4车受当前深2池最早启动41而B1初准备40结束限制，不是一般几何不可能。首轮4车/次轮3车零生成通过。物理面9142小区间变化、资源30拍变化、闭包4车。初pre_registration.json尚需freeze_registration.py升级。

当前运行 freeze_registration.py（日志results/freeze_registration.attempt_1.log）：缓存复用并将M3代价改为准确完整实际尾段含解析锚点改动，旧诊断32拍评分保留；预期同源同选，重新绑定transition实际旧parent及完整闭包并重做两轮零锚点。结束后运行：
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 check_regression.py > results/regression.log 2>&1
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 check_protocol.py > results/protocol.log 2>&1

仍需实现run_case/stage_driver/独立审计/汇总交付。两数值臂在41/42各3车（预计共12次≤16，不为填预算求解4车），首轮全4闭包，L-new次轮保持原物理面可拆单车事务。C-new次轮仍全4。每次求解父进程先登记，独立worker仅本车64变量。实际旧状态始终保持旧见证直到闭包事务通过；reference_state只为构造新合同锚点，不是提前修改执行源。

为避免重复耗时，scan_initial/repaired.json及pre_registration.pre_transition.json等所有中间对象保留。后续冻结源版本、调用预算后禁止重跑已求解事件。共用依赖/tmp/dmpc-p0-deps现存。无需网络或新依赖。

## 更新（完整池与数值前闸门）
freeze_registration和13项protocol、3项regression均通过。R-old执行及独立审计已完成（R-old_accept子进程已退出；父stage_driver PID14128用SIGSTOP暂驻，未释放任何求解，账本仍[]）。为严格符合“全部有限候选先认证再M3”，正在delivery_tools/certify_pool.py逐一认证41拍全部16份；候选0完成，其余有资源冲突明确拒绝。原进程PID14306为复用重复工作已SIGINT，日志保留results/pool_certification.log。

接收器在零调用阶段升级contract_private_reference_v2：完整封存物理合同已验证全联合后续，N之后只传播本车中点参考，直接用本车contract cap约束Ep/Et与目标；不重复四车几何。原代码归档source_archive与executed_source_versions.zero_v1.json。当前正在delivery_tools/refresh_receivers.py（results/refresh_receivers.log）重认证登记首轮4车/次轮3车。完成后重跑check_regression.py和check_protocol.py到新日志，再运行delivery_tools/certify_pool.py（已有完成对象内容核对后复用，输出请用新的results/pool_certification.v2.log）。全部通过后更新executed_source_versions.json为当前root *.py身份，确认solver_calls仍[]，再kill -CONT 14128恢复已授权数值阶段。R-old日志与原代码已归档，可读复用；禁止重新执行数值事件。

后续尚需：三臂完成、真实数值候选拒绝分支与独立审计、汇总activity/B/物理差分/输入代价、report/GATE/最小补丁/搬移归档。当前无用户缺失信息。

## 数值结果与受限修复（必须保留）
C-new-v1已经6次调用，状态5Solved/1AlmostSolved，但两轮全部未受理：A1极薄物理面余量引起接收域小于分支积分包络误差（原行通过但实际Gi检查拒绝）；A2/B2接收速度半径0未进入保和量化block，造成精确终端v行±微小违反。实际执行合法新合同锚点，各2802拍，完整独立安全通过。不得重写成受理，不得重新数值求解这些事件。

修复零调用重登记construction_v3：小于2m的候选轴全部取最大分离，而不选刚可行轴；cap从2.43296e-5增到0.073828m。生成器把Gi零半径v/delta端点等式统一并入contracts[31]，原grid/Q预算不变。旧向量fixture3/3物理回归通过，但协议明确拒绝其旧source/model被重贴新求解。source_archive与executed_source_versions.C_v1.json、results/pre_registration.C_v1.json保留旧版本。新执行版本executed_source_versions.L_v3.json。

L-new-v3实际6次AlmostSolved，两轮A1/A2/B2各3份受理；第一轮A1首拍受配额饱和等式保持，其余首拍变，第二轮三车首拍均变；真实任务2802拍，三车两拍配额兑现，正在完整独立验收。当前账本12/16，C原有6也计入。公平C/L比较因版本不同待完成，GATE必须partial/false，不可写全通过。3车扩大机制与L连续两轮本身可分别通过。

当前stage_driver PID14128已恢复，L-new_accept运行；results/stage_driver.log记录。独立审计阶段外层wall包含父进程为零调用门禁驻留时间，应使用各independent_acceptance.json内部wall作为实际审计耗时。数值周期时间从事件真实release/selection取值。

正在delivery_tools/postsolve_checks.py，零调用复用L真实候选保存三类拒绝续行（results/rejection/postsolve.log），然后delivery_tools/accept_rejections.py独立复核。C-v1全重放入口delivery_tools/audit_C_v1.py会按归档源恢复临时checkout，不能用v3生成器假装重建旧模型。

仍需汇总各臂activity/deadline/路径/输入代价、主窗口3车和全任务初始4车分列、物理差分、失败限制/比较待完成、SHA清单与搬移证据包。下轮最小任务：另设C-new-v3连续两轮3车的完整6次预算，再同版本比较；当前剩余4且C分臂剩余2无法补齐。继续零调用工作无需用户信息。

## 最终验收状态与剩余打包
三臂独立全任务完成；L-v3受理6/6，C-v1受理0/6安全回退；总体GATE partial/false。主窗口L三车连续9拍，完整实际输入代价相对R-old改善0.06268603%，其中新锚点自身0.05307235%，L在新锚点上额外0.00961879%。全程B2802→0，A1目标813→797。已有初始准备四车30拍不算新增四车机制。old_contract_exclusion证明70拍新A1轨迹违反旧A1-A2物理面−.02588343m。
postsolve检查4分支通过；独立覆盖全部相关K与G_in140交接，完整拒绝轨迹保留至目标，纯旧/纯锚点全物理轨迹按内容复用R-old/C-v1全程审计。source_archive保留三个实际执行版本，stage重用入口已通过，不再求解。
仍需最后SHA清单、默认关键源复核、可搬移归档与搬移复核。运行verify_delivery.py不会把partial改成passed。完成后交付report.md/GATE和同级dynamic_groups_002_evidence.tar.gz；最终回复必须说明公平C/L对照待完成，不能写整体通过。

## 四车诊断补充与最终打包
主数据不变，补充delivery_tools/four_car_fork.py已经零调用成功：39拍A1满足W_hold，其两拍反馈恒等，使用已有后缀+8+8前移，四车39拍达到η、40拍三车。完整合同/四份零锚点/控制至目标通过，accept_four_car_fork.py独立99拍至四个K71和G_in138交接通过。它不属于原池/数值臂，O为空，不能称为四车新求解；GATE仍partial/false。证据results/four_car_fork。
下一版优先建议把该连接变体纳入统一池，C/L各第一轮4车第二轮3车，共14次新预算，同步解决4车及公平性；如只补当前3车比较，6次C-v3。当前ledger12/16保持不变。
原默认复核已通过1382文件和四个33拍关键前缀；新增fork派生证据后需最终重新SHA打包并搬移复核。默认核验成功与实验GATE部分完成分开报告。

## 交付完成
默认内容/关键前缀复核与独立目录搬移复核已通过，均零新增求解；搬移清单1422文件。最终tar及SHA receipt在同级dynamic_groups_002_evidence.tar.gz和.archive.json。实验GATE仍partial：L三车两轮6/6，C六次未受理；四车只属独立99拍验证的零调用fork。总调用12/16，禁止重复已登记事件。准确恢复命令：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py；--full-replay按各自归档版本零调用重放。后续需新版本统一C/L，可用14次做4→3，或6次仅补当前三车C对照。所有原始失败、历史和论文保留，无Git提交。
