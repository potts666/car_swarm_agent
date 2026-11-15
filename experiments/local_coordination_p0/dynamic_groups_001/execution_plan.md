# 执行计划

A（零调用）：记录历史身份与冻结配置，恢复历史依赖；共同契约、终端保持、接收后续、两轮零锚点及投影梯度扫描。
B（零调用）：编辑图、迁移、两层回退及十类损坏检查。先完成共同入口与所有调用侧。
C：F-S、F-P、D-P各两轮两车，共最多12次。各车读取同一封存快照；独立进程并发，登记后调用；从真实后继重建第二轮。
D（零调用）：完整四车执行、独立8子段重放、拒绝后续、内容检查与便携包。原始失败保留，已调用不重试；可审计完成事件复用。

改动范围：generation/receivers/rules/derivatives/socp_rows/local_update/protocol/strategy/witness/run_case/accept_case及预检查和交付入口。历史只读，无Git提交。
