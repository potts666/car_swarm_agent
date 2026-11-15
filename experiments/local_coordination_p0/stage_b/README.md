# Stage B local delivery

Start with report.md and STATUS.md. B_GATE_v1.json separates successful execution/verification from unmet B-JL numerical acceptance and accepted-JL shift requirements. Do not interpret ordinary fallback runs as accepted numerical candidates.

The frozen final run is results/comparison_002 (21 bounded stages). Comparison CSV is comparison.csv; full raw/rational/actual decomposition is in objects and analysis/raw_candidate_diagnostics.json. A physical reference traces are in A_reference, with exact original A inputs/code under dependencies/repo.

All runtime imports are contained in this directory, except the standard Python scientific packages listed in requirements-development.txt. No historical P0 bulk experiments are copied. Provenance hashes of historical gates/events are lineage information; necessary current completion objects and actual source code are included. No network or ROS nodes are required for replay.

After extraction, install/use the fixed Python environment and run a controller-external audit without writing over archived acceptance:

```bash
PYTHONPATH=/path/to/fixed/deps python3 - <<'CHECK'
import sys
from pathlib import Path
p=Path('stage_b').resolve()
sys.path.insert(0,str(p))
from accept_case import accept
print(accept(p/'results/comparison_002/curve_B-ZG'))
CHECK
```

This reads/replays an existing run; it does not solve or launch a batch. The returned wall time can differ. A/P0 originals remain in the original engineering workspace.
