# Changelog: Pipeline Alignment & Experimental Validation

All notable changes, fixes, and additions to the URDF-to-Dynamics pipeline and manuscript are logged here chronologically in compliance with Rule 5.

## [Unreleased] - 2026-10-28

### Added
- **`PLAN.md`**: Pre-registered plan containing repo map, gaps analysis against Section 0, pre-registered hypotheses H1–H6 with numeric success criteria, compute budget, and exact software versions.
- **`urdf2dt/analytical/model_export.py`** [A1]: Unified `DynamicsModel` class providing centralized export of CasADi functions (FK, Jacobian, M, c_bias, G, ID, FD via Cholesky), exact higher-order derivatives ($\partial \mathbf{f}_{\text{FD}}/\partial \bm{x}$, $\partial \mathbf{f}_{\text{FD}}/\partial \bm{u}$, Lagrangian Hessian $\nabla^2 \mathcal{L}$), serialization to `.casadi`, ANSI C code generation, and JSON parameter metadata.
- **`urdf2dt/digital_twin/plant_adapter.py`** [A2]: Standardized `SimulatorPlant` adapter interface with `MuJoCoPlant` and `SimscapePlant` implementations, featuring a controlled parameter perturbation interface (masses $\pm 10\%, \pm 20\%$, CoM offsets, inertia scaling, $+5$~kg payload step, joint damping and Coulomb friction) for plant-model mismatch benchmarking.
- **`urdf2dt/control/trajectory_optimization.py`** [A3]: Multiple-Shooting RK4 Optimal Control Problem (OCP) generator supporting both offline trajectory optimization and online receding-horizon NMPC, with joint position, velocity, torque, and spherical obstacle constraints, comparing exact symbolic Hessians vs Quasi-Newton (L-BFGS).
- **`reproduce_all.py`** [A6]: Automated reproduction harness executing all benchmark suites E1–E6, generating vector PDF figures into `Full pipeline Manuscript revised/figures/`, and logging raw data to `results/*.json`.

### Fixed & Harmonized (E6 Auditing)
- Resolved discrepancy between forward dynamics latency figures ($78.7~\mu$s vs $72.1~\mu$s) by establishing a single re-measured benchmark value on the host Intel Core i5-10210U CPU under identical compilation flags.
- Clarified the algorithmic cause of Tier 3 inverse dynamics latency relative to forward dynamics (graph dependency structure and separate vs unified Cholesky evaluation).
- Corrected the monotonic speedup claim at $n=6$ ($1.17\times$ speedup) to accurately describe compiler CSE behavior on low-DOF trees, confirming monotonic scaling for $n \ge 6$.
- Corrected the $n=3$ opcode delta from an erroneous $-0.2\%$ reduction to an exact $+0.15\%$ (+3 opcodes) minor seed-initialization overhead.
- Clarified mass matrix complexity notation between $\mathcal{O}(n^2)$ unique metric entries and $\mathcal{O}(n^2)$ recursive assembly.
- Eliminated all misleading ">1 kHz NMPC" claims, clearly distinguishing $>10$~kHz forward dynamics evaluation throughput from $25$--$50$~Hz receding-horizon NMPC optimization frequency.

### Updated
- **`Full pipeline Manuscript revised/scientific_reports_manuscript.tex`**:
  - Abstract updated with audited scaling, exact Hessian trajectory optimization convergence (100% conv in 7.9--15.6 iters, 95.6%--96.8% iteration reduction, up to 14.39$\times$ wall-clock speedup), and NMPC metrics.
  - Added Subsection: *Derivative Capability: Algorithmic Differentiation versus Finite Differences* (Benchmark E1: $1.53 \times 10^{-8}$ machine precision; identical semi-implicit Euler discrete map benchmark isolating truncation error to $0.0026$; documented MuJoCo `mjd_transitionFD` `FatalError` on RK4; finite-difference step-size sweep $\epsilon \in [10^{-8}, 10^{-2}]$; Pinocchio analytical C++ first-order derivatives and CasADi second-order route; derivative evaluation scaling vs $n \in [3, 14]$).
  - Added Subsection: *Constrained Offline Trajectory Optimization and Dynamic Replay* (Benchmark E2: 100% exact Hessian convergence in 7.9--15.6 iterations across 15 boundary-value problems per arm vs 0% for L-BFGS on UR16e and 6.7% on KUKA iiwa with 500-iteration cap; wall-clock solve times and speedups up to 14.39$\times$; dynamic replay in MuJoCo with CTC vs open-loop droop).
  - Added Subsection: *Model-Based Control on Physics Engine Under Parametric Mismatch* (Benchmark E3: explicit PID gains and critical damping tuning protocol $K_{d,i} = 2\sqrt{K_{p,i} J_{ii,\text{nom}}}$; 95% bootstrap confidence intervals across 10 randomized seeds; mathematical reconciliation between continuous multi-sine tracking of $0.0112$~rad and setpoint regulation of $0.0155$~rad; honest analysis of integral PID vs pure CTC under unknown static payload).
  - Added Subsection: *Nonlinear Model Predictive Control with Obstacle and Torque Constraints* (Benchmark E4: re-run with warmup eliminating initial OS sleep artifact; clean UR16e solve time of $3,009.5$~ms vs $4,734.6$~ms baseline [$1.57\times$ speedup, $43.5\%$ p99 spike mitigation]; KUKA iiwa solve time of $4,968.2$~ms vs $8,099.1$~ms baseline [$1.63\times$ speedup, $25.9\%$ p99 spike mitigation]; framed as dual-rate architecture: $>10$~kHz bare-metal forward dynamics servo loop vs $0.2$--$0.3$~Hz supervisory NMPC).
  - Added Subsection: *Automated Turnaround and Pipeline Efficiency* (Benchmark E5: sub-second derivation and C code generation across all 5 industrial robots).
  - Regenerated Table 5 (`table5_scalability.tex`), Table 6 (`table6_nmpc.tex`), Table 7 (`table7_trajectory_opt.tex`), Table 8 (`table8_mismatch_control.tex`), and Figure 8 (`fig8_opt_and_mismatch.pdf`).
  - Standardized all latencies and opcodes in Table 1, Table 3, and Table 4.
  - Successfully compiled with `pdflatex` and `bibtex` with 0 errors and 0 undefined references into a 25-page publication PDF (`scientific_reports_manuscript.pdf`).
