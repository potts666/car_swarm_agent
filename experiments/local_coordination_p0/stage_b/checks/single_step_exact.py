"""Exact full 5x7 straight-flow calibration, including B-coordinate sign."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import *
from derivatives import builder

data,m=setup();L=F(27,10);v=F(1,2);cases=[]
for actor,sign in [('A2',1),('B1',-1)]:
    tiny=dict(scale=m.Q,car=actor,knots=[['1/2','1/2','0','0']],pose_nodes_Q=[[[0,0],[0,0],[0,0]]])
    raw=builder.interval_jacobian(m,tiny,0,{},8)
    exact=[[F(int(i==j)) if j<5 else F(0) for j in range(7)] for i in range(5)]
    exact[0][3]=sign*H;exact[1][2]=sign*v*H;exact[1][4]=sign*v*v*H*H/(2*L);exact[2][4]=v*H/L
    exact[0][5]=sign*H*H/2;exact[3][5]=H;exact[1][6]=sign*v*v*H**3/(6*L);exact[2][6]=v*H*H/(2*L);exact[4][6]=H
    intervals={(i,j):(F(lo,raw['scale']),F(hi,raw['scale'])) for i,j,lo,hi in raw['nonzero_intervals']}
    for i in range(5):
        for j in range(7):
            low,high=intervals.get((i,j),(F(0),F(0)));assert low<=exact[i][j]<=high
    cases.append(dict(actor=actor,shape=[5,7],passed=True))
dest=HERE/'results/full_single_step_checks.json';assert not dest.exists();write(dest,dict(passed=True,cases=cases));print('Full exact 5x7 state/input Jacobian calibration passed for both directions.')
