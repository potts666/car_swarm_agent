# 实验源码与完整证据

Git 跟踪源码、配置、理论、报告、GATE、汇总表及校验清单。生成的 results/objects、历史源码副本、依赖快照、日志和压缩包不逐个进入 Git，避免编辑器超过10,000项改动限制。原始文件均保留在本机。

本次完整快照含 **24,822 个文件**，归档大小 **983,672,973 bytes**；每个文件均与本机逐字节SHA256核对。完整结果、旧版本、原始失败和已有交付包全部在内。仅省略 Python/pytest 缓存及 Git 元数据。

- [完整证据快照](https://github.com/potts666/car_swarm_agent/releases/download/experiment-evidence-20261115-083601/experiment-evidence-20261115-083601.tar.gz)
- [逐文件清单](https://github.com/potts666/car_swarm_agent/releases/download/experiment-evidence-20261115-083601/snapshot_files.json)
- [GitHub Release](https://github.com/potts666/car_swarm_agent/releases/tag/experiment-evidence-20261115-083601)
- 机器可读地址和SHA256：ARTIFACTS.json。

## 获取完整实验并复验

在仓库根目录执行（GitHub CLI需已登录或使用上面的下载链接）：

```bash
gh release download experiment-evidence-20261115-083601 --repo potts666/car_swarm_agent --pattern 'experiment-evidence-20261115-083601.tar.gz*' --dir /tmp/car-swarm-evidence
(cd /tmp/car-swarm-evidence && sha256sum -c experiment-evidence-20261115-083601.tar.gz.sha256)
tar -xzf /tmp/car-swarm-evidence/experiment-evidence-20261115-083601.tar.gz -C .
python3 experiments/local_coordination_p0/rolling_interval_001/verify_delivery.py
```

解包会恢复快照中的实验路径，包括同名源码；若本机之后有新修改，应先备份或在干净克隆中恢复。上述复验零新增优化器调用。仅 Git 克隆包含轻量源码与报告，完整原始对象和依赖快照需按上面恢复。

`.gitattributes` 禁止对 experiments 自动转换换行，以保持实验源码及清单的原始字节。此次没有重写既有提交，也没有删除本机实验数据。
