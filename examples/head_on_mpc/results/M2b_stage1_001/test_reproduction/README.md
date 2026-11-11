# 21项测试的补录快照

包含五个测试模块、停车夹具、两份场景配置和完整运行对应版本的运行源码，以及5个补录的本地传递依赖（清单见manifest.json的supplemental_transitive_sources）。此快照现在补录；原tests.log没有测试源码哈希，不能追溯性证明当时已经封存这些字节。新的隔离复核结果另存于结果目录test_snapshot_validation.log，不覆盖原tests.log。

使用Python 3.12和数值依赖后，从任意目录执行：

```sh
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 /path/to/test_reproduction/run_tests.py
```

若依赖位于独立目录，追加`PYTHONPATH=/path/to/numerical_deps`。本次复核使用`/tmp/m2b_stage1_repro_deps`；该临时目录不是持久交付的一部分。依赖可按requirements.txt安装；原OSQP/joblib wheel的URL和SHA在上级osqp_wheel.json/joblib_wheel.json。未内置Python、ROS或第三方wheel，不能声明无环境依赖的离线镜像。这21项不需要启动ROS节点或运行完整仿真。

run_tests.py先核对manifest.json的文件SHA，再要求发现恰好21项测试。测试模块自身只从本快照相邻scripts/config/fixtures读取内容；复核时整目录复制到独立临时目录，未使用维护仓库测试/源码路径。
