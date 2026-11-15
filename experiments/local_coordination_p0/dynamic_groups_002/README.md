# dynamic_groups_002

目标：从旧保持状态释放有效任务推进，认证新的物理分离面/资源时段，执行独立本车MPC和完整后续。任务原包保存在instructions/，原始源码与001结果身份见source_inventory.json。方法、条件和零调用改动分别见method_spec.md、bounds_proof.md、execution_plan.md。

本轮结论需区分版本：C-new-v1六次调用均有数值结果，但完整受理0，真实执行采用新合同锚点；修复近零分离面余量及零半径终端保和量化后，L-new-v3两轮各三份真实新候选受理。原始调用/失败/后续均保留，不将回归复用向量冒充新求解；公平C/L比较待同版本补做。GATE按实际证据给出部分完成。

新增调用账本solver_calls.json，严格上限16、每车每轮一次。当前三臂任务中两个数值臂合计12次，余下4次不足以在原口径下补齐两轮三车新对照，不为填预算追加。历史/论文只读，无Git提交。

环境复用/tmp/dmpc-p0-deps。设置OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1、MKL_NUM_THREADS=1、PYTHONPATH=/tmp/dmpc-p0-deps。完整审计共享整数区间/ZOH、变分与锥基础，另重算8子段几何、物理面、100项区间路径投影、实际输入/接收/K与期限。

默认复核、完整重放入口与结果在delivery_tools/和verification/；--full-replay零新增求解，C-v1用归档源重建独立临时检出。默认内容核对不重新调用优化器。

复核：
```
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
# 各自实际版本完整独立重放，零求解：
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py --full-replay
```
复核程序成功不等于实验整体门禁通过；本轮GATE.status=partial。SHA256SUMS冻结原证据；verification/保存清单之后生成的复核结果，避免自引用，最终归档SHA256覆盖这些派生结果。

补充四车诊断：results/four_car_fork记录39拍4车、40拍3车的零调用分支。它利用已有W_hold后缀省去2拍恒等反馈，未加入原主池、未进行四车求解；独立99拍覆盖四个K71和G_in138交接。主实验三车结论与公平比较待完成状态均不变。
