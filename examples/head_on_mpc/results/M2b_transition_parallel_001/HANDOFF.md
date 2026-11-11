# 完成后的短交接

目标完成范围：候选衔接、封存同步S-J/P-J、四持久worker和批量定向DDS已实现；历史882周期队列独立恢复，94案例；31回归+5补充检查；主场景S-J/P-J各2完整运行与2完整泛化均独立通过，均883周期/176.6s；主场景计划ID及全部实际状态/控制完全一致。

正式冻结源码：current_source_version.txt（147b593c...）。维护收尾源码：maintenance_source_version.txt（996f85af...），仅run_m2b_jacobi.py消费真实释放报文版本并绑定发送者/周期。6运行1002次资源准备重放一致，96坏报文拒绝且不改状态；新源码260周期DDS前缀独立通过、计划ID与正式前缀相同。没有把原六次全任务改标为新字节全复现；维护版本31回归通过。

结果入口：report.md / comparison.csv / determinism.json / delivery_status.json；版本边界：post_review_change.json。所有driver已结束，无需继续等待或重复运行。原历史目录未覆盖。完整测试源码/配置/夹具在sources；5个遗漏传递依赖单独补录supplemental_sources，完整隔离31项通过。

核心限制：P-J主场景P95约306/309ms，全部执行周期仍超200ms；不声明硬实时。候选只声明物理/权限/阶段及部分参考行，非完整OCP或一般递归可行证明。理想通信、冻结时钟、有限采样。cache_hits未持久记录，标null/未测；纯DDS成本未拆分；资源/启动/审计成本分列。

复现：复制指定版本sources/<version>到新目录，再补入supplemental_sources/scripts的5依赖，核对各manifest。source /opt/ros/jazzy/setup.bash，设置OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/path/to/numerical_deps ROS_DOMAIN_ID=89；/usr/bin/python3 <snapshot>/scripts/run_m2b_jacobi.py --backend P-J --max-rounds 1 --output <new_directory>；随后verify_m2b.py --run <new_directory>/r1。详细入口README.md。原driver目录完成且版本不同，不能用维护源码resume同目录。

下一研究阶段适合新会话：只携带此交接、报告、当前目标和固定约束，不复制全部历史。未开展持续时钟/延迟丢包/备用控制/新物理场景；未统计本轮模型usage，不从脚本墙钟推算token或费用。
