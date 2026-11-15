"""Zero physical margin: preserve original row and check finite grid fallback."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from c_common import *
from c_socp import nearest,reserved_rows
from socp import evaluate
q=F(1,10**12)
row=dict(name='zero_velocity_lower',g_upper=0,linear=[-H]+[F(0)]*63,gradient_error=0,hessian=0,kind='physical')
inner,reserve=reserved_rows([row],'Q');assert not reserve[0]['applied']
raw=[-3*q/4]+[F(0)]*63;full=nearest(raw);half=nearest([x/2 for x in raw])
assert not evaluate([row],full)['passed'] and evaluate([row],half)['passed'] and not any(half)
assert norm_upper([a-b for a,b in zip(full,raw)])<=4*q
p=HERE/'results/zero_slack_rounding_extra.json';assert not p.exists()
write(p,dict(passed=True,original_zero_margin_preserved=True,raw=raw,nearest_full=full,full_rejected=True,half=half,half_is_locked_zero=True,no_new_physical_action_claimed=True))
print('Zero-margin nearest-grid violation rejected; allowed 1/2 fallback yields exact zero')
