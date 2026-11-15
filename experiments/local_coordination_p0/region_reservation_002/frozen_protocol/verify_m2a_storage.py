#!/usr/bin/env python3
"""Re-audit compact evidence and compare every result except audit wall time."""
import argparse
import hashlib
import json
from pathlib import Path
from pack_m2a_logs import verify as verify_bytes
from verify_m2a import verify as verify_run
from report_m2a import summarize


def compare(source,package,output):
    output.mkdir(parents=True,exist_ok=False)
    manifest=verify_bytes(package);results={}
    for original in sorted(source.rglob('independent_validation.json')):
        relative=original.parent.relative_to(source);baseline=json.loads(original.read_text())
        target=output/relative;target.mkdir(parents=True,exist_ok=True)
        actual=verify_run(package/relative,target/'independent_validation.json')
        expected={k:v for k,v in baseline.items() if k!='independent_audit_wall_s'}
        observed={k:v for k,v in actual.items() if k!='independent_audit_wall_s'}
        if observed!=expected:raise ValueError(f'acceptance changed: {relative}')
        results[str(relative)]=dict(all_audit_fields_equal_except_wall_time=True,actual_full_acceptance=actual['actual_full_acceptance'],actual_prefix_acceptance=actual['actual_prefix_acceptance'])
    metrics={}
    for name in ('C-new_r1','L-new_r1','C-new_r2','L-new_r2'):
        old=summarize(source/'formal'/name);new=summarize(package/'formal'/name)
        if old!=new:raise ValueError(f'metrics changed: {name}')
        metrics[name]=True
    for name,item in manifest['files'].items():
        if hashlib.sha256((source/name).read_bytes()).hexdigest()!=item['original_sha256']:raise ValueError(f'original changed: {name}')
    result=dict(restored_file_hashes_passed=True,originals_unchanged=True,runs=results,formal_metrics_equal=metrics,
        original_bytes=manifest['original_bytes'],stored_bytes=manifest['stored_bytes'],reduction_fraction=manifest['reduction_fraction'],
        float_precision_changed=False,control_algorithm_changed=False,wire_payload_changed=False,local_disk_reclaimed_bytes=0)
    (output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--package',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();compare(a.source,a.package,a.output)
