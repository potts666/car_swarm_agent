"""Compact normal records; full recent/key-cycle diagnostics, gzip and recoverable plans."""
import copy
import hashlib
import json
import math
import numpy as np
from m2a_logs import write_bundle,lines,write_log,encode


def plain(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return plain(value.item())
    if isinstance(value,float) and not math.isfinite(value):return 'Infinity' if value>0 else '-Infinity' if value<0 else 'NaN'
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [plain(v) for v in value]
    return value


def compact(record):
    if record is None:return None
    if isinstance(record,dict):return {k:compact(v) for k,v in record.items() if k not in ('_details','separating_rows')}
    if isinstance(record,(list,tuple)):return [compact(v) for v in record]
    return plain(record)


def diagnostics_record(record):
    # Normal logs retain all errors/iterations/inputs. Matrices and row certificates are key-only.
    return plain(record)


def export(directory,logs,diagnostics):
    write_bundle(directory,logs)
    write_log(directory/'diagnostics.jsonl',diagnostics)
    (directory/'log_format.json').write_text(json.dumps(dict(format='M2b-logs-v1',base='M2a-logs-v2-gzip-plan-ref',plan_content='plans.jsonl.gz',references='$plan',float_precision_unchanged=True,detailed_cycles='failure and previous two; handoff/repair and neighbors'),indent=2)+'\n')
