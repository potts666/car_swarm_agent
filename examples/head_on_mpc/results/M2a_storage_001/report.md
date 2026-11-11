# M2a日志减量：无损压缩与单份计划引用

本轮只完成第一、二步，不改变控制算法、浮点精度、DDS载荷、记录内容及验收判据。原始冻结M2a_local_dmpc_001目录全部保留；package是压缩分发副本，未删除原件，所以本机已回收磁盘空间为0，当前两份同时占用空间。

## 实测存储

原目录533个文件（含原两份清单/核查结果）的数据载荷共745.11 MiB，压缩/引用后91.11 MiB，减少87.77%。这不含新增storage_manifest.json本身；逐文件原始/存储大小与SHA256见package/storage_manifest.json。没有将压缩与去重的收益相加。

|正式L-new_r1文件|原始MiB|gzip+引用MiB|
|---|---:|---:|
|messages|46.9071|0.2469|
|plans|44.0809|8.7665|
|solves|8.4559|0.8994|

计划库仍保存每个ID对应的完整计划；messages中的seed/final发送改为`payload: {"$plan": "ID"}`。消息序号、时间、发送/接收对象、原线上application_bytes及其他字段全部保留。接收、ACK和提交原本已引用ID，无需再改。solves及完整中间诊断仅gzip，不裁剪。实际CSV、配置、源码、结论JSON等原样复制。

## 读取、验收与身份

m2a_logs.lines统一读取旧JSONL或JSONL.gz，恢复计划引用后交给原物理/协议验收及统计逻辑。缺失计划、错误ID、引用内容冲突、同名原文与压缩文件同时存在均拒绝；原application_bytes仍按完整恢复线上信封核查，不能用引用日志大小冒充DDS通信量。逐文件恢复后必须与原始SHA256相同，包含messages恢复后的完整原文字节；由此覆盖浮点数及其原有表示，不用降低精度换空间。

verify_m2a新增--output，可把重验收写到包外。report_m2a共享恢复读取，新增--baseline-root供分发副本显式指向历史对照目录。audit_m2a_future可读取压缩交接点，保护哈希按原始解压字节核对；新诊断日志也gzip。本轮没有重跑旧中央未来求解，只保留/恢复其既有原始日志和结论。

当前维护入口run_m2a默认导出压缩及引用日志；压缩发生在原结束批量导出阶段，未减少DDS传输/双端检查，也不据此声称控制周期更快或更实时。新运行summary.online_wall_s沿用原计时范围，包含结束导出，可能增加压缩耗时；逐周期计时不含结束导出。重复离线诊断若已有同名求解日志会拒绝覆盖，使用新运行/新诊断目录。log_format.json声明新格式，运行源码快照会包含恢复模块。旧冻结源码未修改，新旧读写版本须区别。

## 复现

从仓库根运行；打包输出必须为不存在且位于源目录外的新目录，失败时保留原件及部分输出，禁止覆盖。

```bash
python3 examples/head_on_mpc/scripts/pack_m2a_logs.py \
  --source examples/head_on_mpc/results/M2a_local_dmpc_001 \
  --output /tmp/M2a_compact_copy
python3 examples/head_on_mpc/scripts/pack_m2a_logs.py --output /tmp/M2a_compact_copy --verify
OPENBLAS_NUM_THREADS=1 python3 examples/head_on_mpc/scripts/verify_m2a_storage.py \
  --source examples/head_on_mpc/results/M2a_local_dmpc_001 \
  --package /tmp/M2a_compact_copy --output /tmp/M2a_compact_audit
```

单独读分发包时使用当前维护验收器，结果写到包外，避免覆盖归档证据：

```bash
OPENBLAS_NUM_THREADS=1 python3 examples/head_on_mpc/scripts/verify_m2a.py \
  --run examples/head_on_mpc/results/M2a_storage_001/package/formal/L-new_r1 \
  --output /tmp/M2a_compact_validation.json
```

package中原artifact_manifest.json、source_manifest及verify_delivery.py均作为历史材料原样保留，原清单使用未压缩路径；不能直接用旧脚本核对压缩物理布局，或要求当前维护代码哈希仍等于冻结时版本。用新storage_manifest及pack_m2a_logs --verify核对可恢复原始证据，实际验收用新兼容读取器。旧冻结目录的证据本身未变。

## 检查与下一步

15项测试通过：原11项M2a协议/局部QP检查，以及4项无损往返、缺失/损坏计划拒绝、错误消息/歧义文件拒绝、打包原件不变与损坏包拒绝检查。tests.log保存原始结果；本轮源码见source/和source_manifest.json。

同一批9个运行已重新验收通过一致性核对：6个完整通过、3个保留完整失败/前缀通过；除离线审计墙钟外全部验收字段一致，四次正式完整指标相同，533份原件哈希未变。

同一批数据的重验收与指标比较见revalidation/comparison.json：除重新测量的离线验收墙钟时间外逐字段比较；原始文件哈希、恢复文件哈希及四次正式运行完整指标亦核对。运行失败仍须保持失败，不将前缀通过改成完整成功。

第三步的正常周期裁剪、失败/修复/交接与邻接周期触发详细记录、最小失败复现包和离线中央未来详细诊断开关尚未实施。它们会改变记录内容，需要另立记录契约及恢复/复现验证；本轮不删除成功调试记录或完整诊断。
