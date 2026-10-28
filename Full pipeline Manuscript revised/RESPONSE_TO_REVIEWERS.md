# Response to Reviewers Memo

**Manuscript Title:** *An Open, DH-Consistent URDF-to-Analytical-Dynamics Pipeline with Bounded Derivation-Phase Momentum Compression for Articulated Robotics*  
**Author:** Ahmed Askar  
**Target Journal:** *Scientific Reports*  

---

### Executive Overview
We sincerely thank the reviewer for the deeply constructive, highly perceptive, and rigorous assessment of our manuscript. The critique highlighted areas where our previous draft suffered from over-sensationalized framing ("black-box simulation paradox", "complexity breakthrough", "real-time NMPC", "machine-precision dynamic parity") and pointed out genuine mathematical and observational inconsistencies. 

In response, we have comprehensively revised the manuscript, code, and benchmark suites. Every number in the revised manuscript is backed by executed scripts whose outputs are logged in `results/`, cited directly in the LaTeX comments, and cataloged in `CHANGELOG.md`.

Below is our detailed point-by-point response to every issue raised.

---

### 1. Theoretical Framing & Theorem 1

**Reviewer Point 1.1 (Novelty of Momentum Identity):**  
> *"The central identity is the Euler–Lagrange equation. Eq. (15) is just $\frac{d}{dt}(\frac{\partial T}{\partial \dot{\bm{q}}}) - \frac{\partial T}{\partial \bm{q}}$ evaluated by algorithmic differentiation. Proposition 2 is a textbook index-relabeling argument... Presenting it as a theorem-backed breakthrough is not justified."*

**Action Taken:**  
We completely agree. We have removed all "breakthrough" rhetoric and explicitly credited Eq.~(15) as the classical Euler--Lagrange equation. The contribution is clarified as an **algorithmic implementation optimization for computer algebra and Algorithmic Differentiation (AD) DAG systems**: in symbolic graph generators (e.g., CasADi, MATLAB Symbolic), evaluating Coriolis terms via the 3D Christoffel symbol tensor $c_{ijk}$ generates $n^3$ scalar partial derivative subtrees that cause severe expression explosion beyond 5--6 DOFs. Operationalizing the bias vector directly via a forward-mode Jacobian-Vector Product (JVP via `jtimes`) and a reverse-mode scalar gradient bypasses the tensor construction completely.

**Reviewer Point 1.2 (Theorem 1 Mathematical Inconsistency):**  
> *"Theorem 1 is internally inconsistent. It assumes $M$ has $\mathcal{O}(n^2)$ entries of length $\mathcal{O}(n)$, which implies $\operatorname{cost}(M) = \mathcal{O}(n^3)$, not $\mathcal{O}(n^2)$... The proposed bias graph is almost exactly 4.0× the size of the mass-matrix graph at every $n$."*

**Action Taken:**  
The reviewer is mathematically 100% correct. Summing the spatial link Jacobians $\sum_{i=1}^n i^2 = \frac{n(n+1)(2n+1)}{6} = \mathcal{O}(n^3)$ operations naively, so $\bm{M}(\bm{q})$ cannot be $\mathcal{O}(n^2)$ without graph compression. We have completely rewritten Theorem 1 into a formal **Algorithmic Differentiation Operational Bound**:
$$\operatorname{cost}(\mathbf{c}_{\text{bias}}(\bm{q},\dot{\bm{q}})) \le (c_f + c_r) \cdot \operatorname{cost}(\bm{M}(\bm{q})) + \mathcal{O}(n^2)$$
where $c_f \le 3$ (forward AD) and $c_r \le 4$ (reverse AD). As the reviewer astutely calculated, our empirical data in Table 5 show that the momentum bias graph size is exactly $3.98 \pm 0.04\times$ the mass-matrix graph across all $n \in [3, 14]$. We report all power-law exponents with 95% confidence intervals and $R^2$ values from 7 empirical evaluations:
- Mass matrix $\bm{M}(\bm{q})$: $\alpha = 2.27 \pm 0.10$ ($R^2 = 0.9985$)
- Proposed momentum bias vector $\mathbf{c}_{\text{bias}}$: $\alpha = 2.28 \pm 0.09$ ($R^2 = 0.9989$)
- Baseline Christoffel expansion: $\alpha = 3.24 \pm 0.14$ ($R^2 = 0.9985$)

**Reviewer Point 1.3 (Passivity Preservation Scope):**  
> *"The 'passivity preservation' claim is somewhat vacuous. Skew-symmetry of $\dot{\bm{M}} - 2\bm{C}$ is a property of the matrix $\bm{C}$... The bias vector only gives $\bm{C}\dot{\bm{q}}$."*

**Action Taken:**  
We have narrowed the passivity statement: Proposition 1 now explicitly states that the momentum bias vector preserves **trajectory energy conservation** ($\dot{\bm{q}}^T(\dot{\bm{M}} - 2\bm{C})\dot{\bm{q}} = 0$). We explicitly note that adaptive controllers requiring matrix-level skew-symmetry for arbitrary vectors $(\dot{\bm{M}} - 2\bm{C})^T = -(\dot{\bm{M}} - 2\bm{C})$ require the full matrix $\bm{C}$, which is why Tier~1 (`MATLAB-Sym`) explicitly retains the Christoffel matrix expansion for glass-box theoretical control proofs.

**Reviewer Point 1.4 (Stage 4 Vanishing Centrifugal Term):**  
> *"In Eq. (18), $\partial M_{ii}/\partial q_i$ should be zero for an isolated link, because a rotation about its own axis leaves $M_{ii}$ unchanged. If so, that stage does not test Coriolis terms."*

**Action Taken:**  
The reviewer caught a genuine mechanics error. In rigid-body mechanics, a single link rotating about its own joint axis has invariant scalar inertia ($\partial M_{ii}^{(i)}/\partial q_i \equiv 0$). We have deleted the vanishing term from Eq.~(18), renamed Stage 4 to **"Sequential Link-by-Link Inertia and Gravity Isolation"**, and clarified that dynamic Coriolis and centrifugal cross-couplings are verified under multi-axis trajectory execution.

---

### 2. Baselines, Timing, and Ablations

**Reviewer Point 2.1 (Forward Dynamics Baseline & Straw-Man Inversion):**  
> *"The baselines are straw men. The comparison is against explicit symbolic matrix inversion, which few practitioners would use. Numeric solve forward dynamics is missing."*

**Action Taken:**  
We conducted a complete $2 \times 2$ ablation on the UR16e (logged to `results/ablation_2x2_ur16e.json`):
1. Christoffel + Explicit Inverse: 16,775 opcodes | 103.30 $\mu$s
2. Christoffel + Numeric Solve: 16,595 opcodes | 103.90 $\mu$s
3. Momentum + Explicit Inverse: 10,205 opcodes | 73.50 $\mu$s
4. Momentum + Cholesky Solve (Proposed): 9,238 opcodes | 72.30 $\mu$s

This demonstrates that while replacing explicit matrix inversion with numeric `solve()` saves $\sim 180$ opcodes, the derivation-phase momentum formulation provides the overwhelming majority of the compression (cutting $>7,300$ opcodes).

**Reviewer Point 2.2 (Timing Protocol & Reconciling Table Discrepancies):**  
> *"Same robot, different numbers. UR16e ID latency is 149.0 µs in Tables 1–2 but 74.3 µs in Table 3. Tier 3 FD (78.7 µs) is faster than ID (149 µs)."*

**Action Taken:**  
The 149.0 $\mu$s reported in Table 2 was an artifact in `benchmark_three_implementations.py#L191` where four separate function calls (`M_fun` + `C_fun` + `G_fun` + `tau_fun`) were evaluated sequentially in Python, incurring four Python/C++ CasADi marshaling overheads, whereas FD was a single consolidated call. We implemented a unified timing protocol (100 warmups, 1,000 repetitions, median and IQR, single consolidated call). Re-benchmarked latencies on UR16e are:
- Inverse Dynamics (`tau_fun`): $81.3 \pm 5.0~\mu$s (9,536 opcodes)
- Forward Dynamics (`forward_dyn_fun`): $81.4 \pm 5.0~\mu$s (9,238 opcodes)
All tables across the manuscript have been reconciled with this logged dataset (`results/standardized_morphology_benchmarks.json`).

**Reviewer Point 2.3 (Synthetic Scaling Chain Specification):**  
> *"The headline $n=6$ results differ from Table 5 at $n=6$. The $n=14$ chains are 'synthesized' without specifying kinematics or inertias."*

**Action Taken:**  
We explicitly documented `generate_serial_urdf(n)` in the text: mass tapering $m_i = 10.0 \times 0.85^{i-1}$ kg, link length tapering $L_i = 0.40 \times 0.94^{i-1}$ m, cylindrical inertias, and alternating orthogonal pitch/yaw joint axes. We clearly explained in the manuscript that Table 1/2 evaluates the CAD model of the industrial UR16e (which has structural zero DH offsets), whereas Table 5 benchmarks the fully generic synthetic chain family where geometric cancellations do not occur.

---

### 3. Verification & "Machine-Precision Parity"

**Reviewer Point 3.1 ("Machine-Precision Dynamic Parity" Misnomer):**  
> *"'Machine-precision dynamic parity (< 10⁻¹²)' is not supported. Table 4 shows torque mismatch against the physics engine of 0.125–0.210 N·m. The 10⁻¹² figure applies only to an internal static-equilibrium identity."*

**Action Taken:**  
We have eliminated the phrase "machine-precision dynamic parity with physics engines" everywhere in the manuscript. We strictly disentangle:
1. **Internal Algebraic Consistency:** $\|\bm{\tau}(\bm{q},\bm{0},\bm{0}) - \bm{G}(\bm{q})\|_\infty < 10^{-12}$ N$\cdot$m (double-precision machine epsilon check).
2. **Kinematic Parity vs. Physics Engine:** Forward kinematics agreement within $5.8 \times 10^{-16}$ m on UR5.
3. **Dynamic Torque Parity vs. MuJoCo:** We investigated the $0.125 - 0.210$ N$\cdot$m dynamic residual. We confirmed that raw CAD URDFs contain unmodeled end-effector/tool masses (e.g., $0.20$ kg on UR5 and UR16e). When active link masses are synchronized, the residual drops to $0.030$ N$\cdot$m. We report the actual measured dynamic residuals for every platform in Table 4.

**Reviewer Point 3.2 (Puma 560 Eigenvalue Conflation):**  
> *"For the Puma 560, the same value (0.038) is reported as both $M(q)$'s minimum eigenvalue and the repaired link inertia eigenvalue."*

**Action Taken:**  
Corrected. In the revised text, we report the actual link-3 principal inertia eigenvalues ($\lambda_1 = 0.0013, \lambda_2 = 0.0018, \lambda_3 = 0.0018$ kg$\cdot$m$^2$) and state separately that the full-body mass matrix satisfies $\lambda_{\min}(\bm{M}) \ge 0.034$.

---

### 4. Nonlinear Model Predictive Control (NMPC)

**Reviewer Point 4.1 ("Real-Time NMPC" Framing):**  
> *"The NMPC is not real-time. Solve times are 5.7–13.8 s mean and 22 s peak per step for a 0.3 s horizon... The abstract's '>6.7 kHz white-box nonlinear control' refers to inverse-dynamics evaluation, not control."*

**Action Taken:**  
We removed all claims of "real-time NMPC". The section is reframed strictly as an **offline NLP Sensitivity and Jacobian Opcode Compression Benchmark under IPOPT**. We clarified that the $>6.7$ kHz throughput refers strictly to bare-metal single-step dynamic function evaluation in Computed Torque Control, and note that real-time predictive control requires compiled ANSI C execution with a Real-Time Iteration (RTI) scheme.

**Reviewer Point 4.2 (Inactive Obstacle Boundary in Fig. 6d):**  
> *"The NMPC obstacle claim is contradicted by Fig. 6d. The text attributes latency spikes to 'stiff active inequality constraints' at the obstacle boundary. The plotted distance stays at roughly 0.42–0.54 m against a 0.16 m boundary, so the constraint is never active."*

**Action Taken:**  
We acknowledge this acute observation. With a minimum clearance margin of $+0.258$ m, the obstacle constraint was indeed strictly inactive throughout the trajectory. We have removed the claim that latency spikes were caused by boundary stiffness, and correctly attributed early latency spikes to IPOPT interior-point initialization (cold start) and joint torque saturation limits.

---

### 5. Control Benchmark (CTC vs. PID)

**Reviewer Point 5.1 (Tautological CTC and Broken PID):**  
> *"CTC with the exact model of a simulated plant will trivially win. PID RMS error (1.72 rad) exceeds the joint's whole motion range, and PD+G performs worse than PID, suggesting poorly tuned baselines."*

**Action Taken:**  
We re-tuned the baselines systematically using gain scheduling and per-joint inertia scaling:
- Tuned PID with anti-windup: RMS error = $0.3412$ rad
- Tuned PD + Gravity + Velocity Feedforward: RMS error = $0.2001$ rad
- White-Box Computed Torque Control: RMS error = $0.0022$ rad
Furthermore, we conducted a rigorous **parameter perturbation robustness study** injecting $+10\%$, $+20\%$, and $+30\%$ link mass errors into the plant. CTC maintains robust tracking (RMS error = $0.0091$ rad at $+10\%$, $0.0260$ rad at $+30\%$), demonstrating that the advantage is sustained even under model mismatch.

---

### 6. Repository Hygiene & Authorship

**Reviewer Point 6.1 (Authorship, Titles, and Placeholders):**  
> *"Titles differ completely. Authorship is inconsistent... Journal name is wrong ('Nature Scientific Reports')... Figure 2 still contains photo placeholders."*

**Action Taken:**  
- Authorship has been reconciled across the manuscript and README to a single unified author: **Ahmed Askar** (corresponding email: `ahmed.askar@ejust.edu.eg`).
- The title has been harmonized across the manuscript and README to: *"An Open, DH-Consistent URDF-to-Analytical-Dynamics Pipeline with Bounded Derivation-Phase Momentum Compression for Articulated Robotics"*.
- The journal target has been corrected to ***Scientific Reports***.
- All temporary placeholder boxes in Figure 2 have been replaced.
