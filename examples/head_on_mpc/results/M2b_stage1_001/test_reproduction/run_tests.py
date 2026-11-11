"""Verify supplemental snapshot bytes and execute exactly its 21 tests."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

root=Path(__file__).resolve().parent
manifest=json.loads((root/'manifest.json').read_text())
for name,digest in manifest['files'].items():
    if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:
        raise SystemExit('Snapshot hash mismatch: '+name)
sys.path.insert(0,str(root/'tests'))
names=['test_m2b_stage1','test_m2b_protocol','test_m2b_control','test_stopping_acceptance','test_m2b_scene']
suite=unittest.defaultTestLoader.loadTestsFromNames(names)
if suite.countTestCases()!=manifest['expected_tests']:
    unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit('Expected exactly 21 tests; check dependencies/import errors')
raise SystemExit(0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1)
