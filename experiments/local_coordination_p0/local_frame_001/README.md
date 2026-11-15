# local_frame_001

Read report.md, GATE.json, method_spec.md, implementation_map.md. All12 formal attempts consumed; do not rerun native stages.

Default no-solve verification: `python3 verify_delivery.py`
Full independent physical replay: `python3 verify_delivery.py --full-replay`
Both verification modes use Python standard library only; optional model reconstruction uses requirements.txt and existing isolated /tmp/dmpc-p0-deps.

Source provenance source_inventory.json; executed numerical_v1 immutable; acceptance sources frozen separately. Original failures retained under results/failed_post_v1 and cache_guard_failure.json.
