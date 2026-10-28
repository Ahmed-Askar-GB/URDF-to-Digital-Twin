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
  - Abstract updated with audited scaling and NMPC metrics.
  - Added Subsection: *Derivative Capability: Algorithmic Differentiation versus Finite Differences* (Benchmark E1: $1.53 \times 10^{-8}$ machine precision vs MuJoCo FD $0.2662$ error, exact Lagrangian Hessian availability).
  - Added Subsection: *Constrained Offline Trajectory Optimization and Dynamic Replay* (Benchmark E2: 100% exact Hessian convergence in 8.5--16.2 iterations vs 0% for L-BFGS, dynamic replay RMSE in MuJoCo, honest discussion of friction droop under simulator damping).
  - Added Subsection: *Model-Based Control on Physics Engine Under Parametric Mismatch* (Benchmark E3: CTC vs PID under 0%, $\pm 10\%$, $\pm 20\%$, and $+5$~kg payload step; honest analysis of integral PID vs pure CTC under unknown static payload).
  - Added Subsection: *Automated Turnaround and Pipeline Efficiency* (Benchmark E5: sub-second derivation and C code generation across all 5 industrial robots).
  - Embedded Table 5 (`table5_scalability.tex`), Table 6 (`table6_nmpc.tex`), Table 7 (`table7_trajectory_opt.tex`), Table 8 (`table8_mismatch_control.tex`), and Figure 8 (`fig8_opt_and_mismatch.pdf`).
  - Standardized all latencies and opcodes in Table 1, Table 3, and Table 4.
  - Successfully compiled with `pdflatex` and `bibtex` with 0 errors and 0 undefined references into a 25-page publication PDF (`scientific_reports_manuscript.pdf`).
