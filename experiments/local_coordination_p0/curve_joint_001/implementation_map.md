# Authentic implementation map

| Responsibility | Existing source | New shared entry |
|---|---|---|
| actual model/body | encounter/common.py, physics.py, sources/interval_source.py | same files, local varying-delta integration refinement only |
| initializer/analytic contract | initialize.py, preflight.simultaneous, encounter.propose | rebuild_reference.py |
| old straight generator/freeze blocker | encounter/local_mpc.prepare | curve_model.py; reject legacy all-omega equalities for O arms |
| nonlinear variational ODE | region003/sources/certify_interfaces.interval_jacobian | derivatives.py (same range method, fine subdivision and packing) |
| structured derivatives | CURVE C4-C15 + region003/derivatives | curve_bounds.py |
| nonlinear original rows | region003/socp_rows.py | curve_model.py, shared four-arm rows |
| SOC compilation | region003/conic centered_equivalent_v1 | local_mpc.py preserving nonlinear rows and exact E |
| progression | CURVE C21-C22, signed geometric arc/line atlas | progression.py |
| receiving/suffix/old point guard | encounter/execution.gate | receivers.py and contract.guard (legacy execution.py is retained but never called by the new runner) |
| quantizer | encounter/local_mpc.admit affine increment lattice | quantization.py weighted rational local nullspace |
| worker bytes/ready/release | encounter/transport.py, local_mpc.construct/native | same transport, new common model assembly |
| commit/ownership/deadline | encounter/transaction.py | contract.validate_binding/patch; run_experiment four-party ACK closure and per-car deadline checks |
| joint execute/shift | encounter/run_case.py, plan_io | four-arm bounded run_experiment.py, each second source from its own commit |
| independent audit | encounter/audit, audit_updates | original eight-subdivision joint replay + nonlinear raw/receiver/objective checks |

No original source edited. R-original keeps strict-point-source identity as a
zero-call comparison; new domain guards shared by all four arms. Existing input
normalization S=I,R=I verified from actual conic coefficients; JE introduced with
explicit h and common positive objective multiplier5 (same E minimizer).
