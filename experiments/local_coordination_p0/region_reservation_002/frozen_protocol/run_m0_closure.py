#!/usr/bin/env python3
"""Paired stopping ablation: original M0_001 versus minimal repair, warm start on in both."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import numpy as np
import scipy
import osqp
from bicycle_core import validate_scene
from linearized_joint import BicycleController
import linearized_joint
from longitudinal_joint import routes
from ros_flag import RosFlag
from run_joint_e2 import run_b1, verify_ros, verify_future_execution
from run_entry_bicycle import ROOT, write
from run_bicycle_e3 import summarize
from verify_m0 import verify


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=False)
    scripts=Path(__file__).parent;old=ROOT/'results/M0_001/source'
    provenance=json.loads((old.parent/'provenance.json').read_text())
    for name in ('linearized_joint.py','longitudinal_joint.py','bicycle_core.py','ros_flag.py','verify_entry_bicycle.py','verify_m0.py'):
        assert hashlib.sha256((old/name).read_bytes()).hexdigest()==provenance['source_sha256'][name]
        if name!='linearized_joint.py':assert (old/name).read_bytes()==(scripts/name).read_bytes(),name
    current=Path(linearized_joint.__file__).read_text()
    start=current.index('    stop_adjustments=[]');end=current.index('    residual=matrix@u;',start)
    without_repair=(current[:start]+current[end:]).replace(',stop_input_adjustments=stop_adjustments','')
    assert without_repair==(old/'linearized_joint.py').read_text(),'ablation must differ only by stopping input repair'
    spec=importlib.util.spec_from_file_location('m0_unrepaired_bicycle',old/'linearized_joint.py')
    baseline=importlib.util.module_from_spec(spec);spec.loader.exec_module(baseline)
    scene=json.loads((ROOT/'config/entry_bicycle.json').read_text());validate_scene(scene);paths=routes(scene)
    write(out/'scene.json',scene);write(out/'paths.json',paths)
    source=out/'source';source.mkdir()
    for p in [*scripts.glob('*.py'),*(ROOT/'tests').glob('test_*joint.py'),ROOT/'tests/test_stopping_acceptance.py',ROOT/'tests/test_m0.py']:
        shutil.copy2(p,source/p.name)
    shutil.copy2(old/'linearized_joint.py',source/'linearized_joint_unrepaired.py')
    write(out/'provenance.json',dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,
        platform=platform.platform(),cpu_model=next((s.split(':',1)[1].strip() for s in Path('/proc/cpuinfo').read_text().splitlines() if s.startswith('model name')),None),
        threads={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()},
        command='source /opt/ros/jazzy/setup.bash && PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID='+os.environ.get('ROS_DOMAIN_ID','0')+' ROS_LOG_DIR=/tmp/m0_closure_ros '+sys.executable+' '+' '.join(sys.argv),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),warm_start_both=True,repeats=2,
        controlled_difference='stopping first-input repair only; source ablation checked byte for byte',
        validators_unchanged=True,original_DDS_Flag=True,simulation_time_frozen=True))
    (out/'code_changes.patch').write_text(subprocess.check_output(['git','diff'],text=True))
    results={};future={}
    for repeat in (1,2):
        for name,controller_class in [('unrepaired',baseline.BicycleController),('repaired',BicycleController)]:
            directory=out/f'{name}_r{repeat}';print('Running',directory.name,flush=True)
            bus=RosFlag()
            try:run_b1(scene,paths,directory,bus,controller=controller_class(scene,paths,3,reference_warm_start=True))
            finally:bus.close()
            config=json.loads((directory/'config.json').read_text());config.update(controller=name,reference_warm_start=True);write(directory/'config.json',config)
            independent=verify(directory,scene);write(directory/'validation.json',independent['original_log_validation'])
            verify_ros(directory);consistency=verify_future_execution(directory,scene);write(directory/'future_execution_consistency.json',consistency)
            result=summarize(directory,scene)
            result.update(independent_full_task_passed=independent['passed'],independent_prefix_passed=independent['executed_prefix_passed'],
                all_cycle_exits_timed=independent['all_cycle_exits_timed'],future_execution_consistency_passed=consistency['passed'])
            results[directory.name]=result;future[directory.name]=consistency
            write(out/'comparison.json',results)
            print(directory.name,json.dumps({k:result[k] for k in ('completed','completion_sim_s','independent_full_task_passed','failure','cycle_max_ms')}),flush=True)
    write(out/'future_execution_consistency.json',future)
    write(out/'summary.json',dict(experiment_completed=True,runs=4,
        repaired_passed=all(results[f'repaired_r{i}']['independent_full_task_passed'] for i in (1,2)),
        original_failures_retained=all(not results[f'unrepaired_r{i}']['independent_full_task_passed'] for i in (1,2)),
        no_realtime_or_dmpc_claim=True))


if __name__=='__main__':main()
