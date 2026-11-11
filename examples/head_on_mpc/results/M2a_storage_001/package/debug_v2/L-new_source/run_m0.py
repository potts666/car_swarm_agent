#!/usr/bin/env python3
"""M0: original B2 sequence versus shifted primal reference warm start, two repeats."""
import argparse
import hashlib
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
from linearized_joint import BicycleController, MAX_UPDATES
from longitudinal_joint import routes
from ros_flag import RosFlag
from run_joint_e2 import run_b1, verify_ros, verify_future_execution
from run_bicycle_e3 import summarize
from run_entry_bicycle import ROOT, write
from verify_m0 import verify


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=args.output;output.mkdir(parents=True,exist_ok=False)
    scene=json.loads((ROOT/'config/entry_bicycle.json').read_text());validate_scene(scene);paths=routes(scene)
    write(output/'scene.json',scene);write(output/'paths.json',paths)
    source=output/'source';source.mkdir()
    files=[*Path(__file__).parent.glob('*.py'),*sorted((ROOT/'tests').glob('test_*joint.py')),ROOT/'config/entry_bicycle.json',ROOT/'bicycle_joint_qp.md',ROOT/'requirements-qp.txt']
    for path in files:shutil.copy2(path,source/path.name)
    (output/'code_changes.patch').write_text(subprocess.check_output(['git','diff'],text=True))
    write(output/'provenance.json',dict(git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()},
        python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,platform=platform.platform(),
        cpu_model=next((s.split(':',1)[1].strip() for s in Path('/proc/cpuinfo').read_text().splitlines() if s.startswith('model name')),None),
        threads={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
        command='source /opt/ros/jazzy/setup.bash && PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID='+os.environ.get('ROS_DOMAIN_ID','0')+' '+sys.executable+' '+' '.join(sys.argv),
        changes='shared stopping first-input overbraking repair before unchanged affine/nonlinear acceptance; arms differ only by shifted committed reference primal seed; original RK4 derivatives, 3 updates, limits, stopping conditions, Flag, full future witness retained; all-exit timing applies to both arms',
        repetitions=2,communication='actual ROS2 three nodes, reliable DDS, ideal cycle synchronous',simulation_time_frozen_during_computation=True,
        no_realtime_or_dmpc_claim=True))
    results={}
    for repeat in (1,2):
        for name,warm in [('B2_sequence_cold',False),('B2_sequence_warm',True)]:
            directory=output/f'{name}_r{repeat}';print('Running',directory.name,flush=True)
            bus=RosFlag()
            try:
                controller=BicycleController(scene,paths,MAX_UPDATES,reference_warm_start=warm)
                run_b1(scene,paths,directory,bus,controller=controller)
            finally:bus.close()
            config=json.loads((directory/'config.json').read_text());config.update(controller=name,reference_warm_start=warm);write(directory/'config.json',config)
            independent=verify(directory,scene);write(directory/'validation.json',independent['original_log_validation'])
            verify_ros(directory)
            consistency=verify_future_execution(directory,scene);write(directory/'future_execution_consistency.json',consistency)
            result=summarize(directory,scene)
            result.update(repeat=repeat,reference_warm_start=warm,independent_full_task_passed=independent['passed'],
                independent_prefix_passed=independent['executed_prefix_passed'],all_cycle_exits_timed=independent['all_cycle_exits_timed'],
                future_execution_consistency_passed=consistency['passed'])
            results[directory.name]=result;write(output/'comparison.json',results)
            print(directory.name,json.dumps(result),flush=True)
    # A retained failure is a valid M0 outcome, not a reason to alter the experiment.
    write(output/'summary.json',dict(experiment_completed=True,runs=len(results),
        all_logs_independently_checked=True,warm_full_task_passed=all(r['independent_full_task_passed'] for r in results.values() if r['reference_warm_start']),
        no_realtime_or_dmpc_claim=True))


if __name__=='__main__':main()
