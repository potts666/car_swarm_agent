# autonomy_init_001 method, frozen before implementation

Use AUT-DMPC-v1 and the unchanged signed bicycle model (h=1/5, N=32,
rectangle [-1,3.5] x [-.9,.9], safety 1/2). Historical whole-task witnesses
are excluded from initialization. Geometry templates may be reused with explicit
source identities. Initialization inputs are static map, vehicle bounds, individual
true initial states and goals, edge certificates and request rounds.

First certify continuous local paths and endpoint domains. Derive initial/goal
parking dependencies from actual swept rectangles, adding uncertain edges.
Try the existing radius-six S connections and a declared finite extension of
steering/exit placement under identical model bounds. If DAG fails, retain cycles
and check the strong BUF endpoint separation and protected-endpoint reachability
conditions. Enter initialization replay and numerical stages only after one full
condition passes. This task's optimizer budget is 8, independent of DG002.

Use two request-round orders, derive all schedule lengths from generated edges,
and independently replay both schedules before historical full-witness comparison.
Register full tail/proposal/receiver/permanent-goal occupancy before discovery;
verify transaction, receiver and affine-quantization invariants. Preregister one
continuous two-round multi-activity window before any native solver release.
Retain failures and do not claim stages whose preconditions have not passed.
