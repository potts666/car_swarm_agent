# C0 诊断与修复验证（2026-11-13）

本轮“数值新候选未受理”的控制接续阻塞已在两个固定接口关闭，采用独立规则 `certificate_suboptimal_v1`。严格 `Solved` 数值修复未通过；原 C0 失败门禁保持原样。本结论仅覆盖本轮两个局部接口，不推广为全程性能或扰动稳定性结论。

## 四层验收

|层级|结果|证据|
|---|---|---|
|原因定位|归档12例严格失败4/4/4复现；12份原模型和数值锥内点通过。停止前具体来源保留未知|archive_audit.json；results/probe/*_trace.json|
|原规则数值修复|未通过：4次均 AlmostSolved；居中表达两例均原始残差不达1e-10|comparison.csv|
|新受理规则|通过：同一原始结果，S拒绝，C完整证书通过；没有追加求解|results/probe/summary.json；validate_fix.py|
|控制接续|通过：各240拍、1次数值共同受理、实际修改32拍前缀、实际移位、K、拒绝后同一旧K续行、停车换权及下一车首拍|results/curve、results/straight_acceleration 的执行日志和 independent_acceptance.json|

## 数值诊断与局限

原表达曲线主要失败于gap，直线主要失败于原始残差；居中表达曲线原始残差约7.92e-9，直线约1.63e-7，均超过1e-10。二者gap和对偶残差已满足严格条件，kappa/tau也满足外围条件。原表达的抵消指标约11.39/12.14，居中后均为1；这是几何改写的观察，不能据此断言KKT误差根因。

完整保存65维x、440维s/z、原矩阵、所有公开设置、逐轮callback/verbose记录、原始及对偶目标和精确二进有理数残差向量。高精度点积先用Fraction精确求和，再以浮点显示范数；独立重算的归一化值基于返回的x/s/z和L2范数，单独保存，未冒称与内部在缩放/后处理前的指标完全相同。停止条件来源为官方 v0.11.1 的 [info.rs](https://github.com/oxfordcontrol/Clarabel.rs/blob/v0.11.1/src/solver/implementations/default/info.rs) 和 [settings.rs](https://github.com/oxfordcontrol/Clarabel.rs/blob/v0.11.1/src/solver/implementations/default/settings.rs)，源码副本保存在sources/。公开callback最后仍为Unsolved，最终为AlmostSolved，故后处理前的InsufficientProgress/NumericalError具体来源未知；未插桩、未改变求解器版本。线性求解器为qdldl，1线程；默认正则化及迭代改进设置已保存，逐次触发及次数未公开。

## 理论条件、实现、实际观察

理论：instructions/theory.md §7 的等价锥恒等式、§5严格内点和§8的A_cert是条件性依据。安全/递归可行/有限完成依赖名义模型、合法历史策略和保存后续、一致提交及预算下降；次优受理不提供严格最优性或扰动稳定性证明。

实现：fix_socp.py仅重写H>0、m>0的锥块；H=0线性行及m<=0旧有效分支保留。原Q行、量化预留、尺度1e-4、64维输入、32拍、rho=.001、时长/分组/停车换权/目标权重均固定。状态白名单只开启检查，不直接受理；原行、范数重建、精确最近量化、真实前缀能量、实际K/后继流像、后续及预算再由A验收和四车协议提交。独立检查器按65维和精确有理量化核对数值来源，重新生成导数和终端参数，并以更细8子段重放连续几何。

观察：

|接口|实际改变前缀拍数|真实前缀能量改善|道路净空下界m|六对车身净空下界m|
|---|---:|---:|---:|---:|
|curve|32|0.050054%|0.498573|0.501954|
|straight_acceleration|32|0.063600%|0.498530|0.501955|

各1个数值提案、1个完整认证、1次共同受理、1次实际K、1次损坏K拒绝后续行、1次停车换权。首次输入分歧均发生在提案首拍。239拍连接加下一车首拍共240拍，预算1279→1039逐拍减1，无增补。已执行所有32拍数值前缀，能量由原始实际命令独立重算；固定时长变化为0。结果不使用A人工构造候选替代数值提案。

下一实际锚点均重建32子段导数、更新history/phase/信赖中心、检查原行及连续真实尾段、接收K和后续；下一免费接收盒检查均通过。本轮继承已受理尾段完成接续，没有追加第二次数值求解；这不构成持续在线优化验收。预算绑定原8连接已认可策略及同版见证，不从文档常数重建其它策略。

## 代码与保存

新增代码和结果仅在stage_c_fix_001。changes.patch包含共享锥表达、受理和候选/共同提交调用链、实际移位日志、65维独立验收改动及自检。源提交与固定配置见plan.json。base_c保存冻结旧C及其B/A依赖，objects按内容ID核对；指令及身份清单也保留。

自检覆盖精确恒等式/正负tau/内外边界、线性和非正margin分支、量化误差、状态白名单/NaN、raw来源/anchor/导数损坏，以及K、phase、对象、域、期限、见证、预算损坏拒绝。历史文件逐项SHA保持不变。独立验收初版Fraction与JSON字符串直接比较导致断言失败，原失败证据保留为failed_001；改用相同规范化内容身份比较后通过，未重跑求解与控制日志。

## 复核与恢复

在本目录：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_validation.py
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
```

run_validation.py以180秒分层超时、完整日志及内容校验checkpoint复用已通过证据。verify_delivery.py在分发目录核验清单及两份独立重放，无新求解。复现实验需在新目录运行probe.py（4次）及执行器，禁止覆盖旧证据；依赖Python3、Clarabel0.11.1、NumPy/SciPy，实测版本见results/probe/environment.json。分发清单与SHA另附，重复内容以tar硬链接去重。

未开展C2、资源预约、多活动车、异步通信、正式批量或论文修改；无Git提交。后续扩展须另立任务。待解决：严格数值收敛、停止前状态可观测性、持续在线多次优化与全程评价。
