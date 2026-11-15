# CURVE-JOINT-v1 implementation, pre-release declaration

Four arms F-E/O-E/F-P/O-P, same initial joint state and peer envelopes, same positive
pose receiver and physical/terminal/moment constraints; only frozen moving omega
increments and objective flag differ. Two rounds/2 own workers per arm,16 release
attempts maximum; exceptions counted, no repeated release. R-original0 calls.
Fixed model/config copied from passed encounter/autonomy sources; h=.2,N32,rho=.001,
eta=.02,Delta1e-12,S=I,R=I. Original JE=h sum(a²+omega²); progression lambda1 in
JE units/metre; all conic objectives multiply by1/h=5, including progression.

Rebuild1753-tick analytic strategy from actual initializer/parallel/encounter
recipe, compare controls/calendar/goals with prior analytic object (not old raw
numerical inputs). Scan real source states in time order: curved nonzero-speed
actor and another actually moving actor, numeric perturbations confined to actual
motion intervals. Record unavailable constant32 circle condition; mixed segments
use actual variational C4/C5 and C6-C21 prefix bounds, not example constants.

Each active block ends before the next stopped steering/mode guard. Preserve
sum(a), first acceleration time moment, sum(omega), and actual rational weighted
omega moment; each has explicit terminal/heading purpose. Freeze remaining
nominal mode inputs. First-order heading residual for nonconstant reference is
covered by full yaw Jacobian/uncertainty and second-order receiver row, never
claimed to be an exact physical yaw equality. E/T rational basis, grid bound and
actual endpoint/suffix proof use the same objects. No exact position constraint
for the curved actor. Four arms share every non-freeze row.

Source/end local progression charts use analytic arc/line projection with a
continuous geometric seam atlas; endpoint chart must stay strictly away from
seams and circle centre. Reuse interval variational quadrature and SOC compiler,
refine derivatives until c_cert positive and quantum fits strict descent budget.
Physical actual varying-delta ZOH retains original validated model, with local
integration refinement when objective accuracy/receiver needs it.

New phase receiver chains use positive position/yaw radii, exact reference v/delta,
same-input SE2 suffix bounds and full finite guard/goal checks. Replace tight
point exit/wait checks by certified domains inside the original physical clearance
and target tolerance. Original strict-point policy remains separate R-original.
Any failed prerequisite is saved before shared-rule repair/regression. No numerical
release before nonzero direction, complete common suffix, next-phase zero models,
actual byte/assembly checks and source version freeze. Preserve history, papers,
RViz publisher and old task budgets; no Git commits.
