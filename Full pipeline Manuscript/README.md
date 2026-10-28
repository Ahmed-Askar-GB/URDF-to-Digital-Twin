# Tri-Paradigm White-Box Analytical Digital Twins vs. Black-Box Simulators
### Target Publication: *Nature Scientific Reports*

This repository contains the complete unified codebase, benchmark datasets, high-resolution figures, and LaTeX manuscript for the paper:

> **"Bridging Black-Box Physics Simulators and White-Box Analytical Mechanics: A Tri-Paradigm Framework for Interpretable Robot Modeling, Verification, and Nonlinear Control"**
>
> *Authors: Abdelrahman Ihab, Ahmed Askar, Haitham El-Hussieny*

---

## 1. Directory Structure

```text
Full pipeline Manuscript/
├── track1_matlab_sym/             # Tier 1: Pure MATLAB Symbolic Math + Glass-Box Models
│   ├── fk_sym.m                   # Symbolic DH Forward Kinematics (T_0^i = T_0^{i-1} * A_i)
│   ├── jacobian_sym.m             # Analytical & Geometric Jacobians (J_{v,i}, J_{\omega,i})
│   ├── dynamics_sym.m             # Closed-Form Mass Matrix M(q), Christoffel c_{ijk}, Gravity G(q)
│   └── run_track1_sym.m           # Standalone verification and algebraic inspectability driver
│
├── track2_matlab_casadi/          # Tier 2: MATLAB / CasADi + Simscape Multibody Framework
│   └── run_track2_casadi.m        # Industrial Model-Based Design (MBD) orchestrator
│
├── track3_python_mujoco/          # Tier 3: Python / CasADi + MuJoCo Real-Time Nonlinear Control
│   └── run_track3_mujoco.py       # Zero-licensing real-time control (>1 kHz) & inverse dynamics
│
├── benchmarks/                    # Cross-Implementation Benchmarks & Datasets
│   ├── benchmark_three_implementations.py  # Master benchmark suite (Complexity, Latency, Control)
│   ├── benchmark_three_implementations.m   # MATLAB-native verification runner
│   ├── generate_manuscript_figures.py      # Publication vector (PDF) & PNG figure generator
│   ├── benchmark_results.json              # Full quantitative benchmark dataset
│   ├── table1_tri_paradigm_tradeoffs.csv   # Table 1: Readability vs. Speed vs. Deployability
│   ├── table2_multi_robot_verification.csv # Table 2: Multi-robot 5-stage verification results
│   └── table3_control_comparison.csv       # Table 3: Closed-loop tracking error reduction
│
├── figures/                       # Publication Figures (300+ DPI & Vector PDF)
│   ├── fig1_tri_paradigm_architecture.pdf  # Figure 1: Tri-Paradigm System Architecture
│   ├── fig2_sequential_isolation.pdf       # Figure 2: Sequential Link Dynamic Isolation
│   ├── fig3_tracking_trajectories_ur16e.pdf# Figure 3: High-speed multi-axis tracking (UR16e)
│   └── fig4_torque_profiles_and_error.pdf  # Figure 4: Joint error decay & torque decoupling
│
├── scientific_reports_manuscript.tex # Full publication-ready LaTeX manuscript
├── scientific_reports_manuscript.pdf # Compiled PDF (12 pages, Nature format)
├── references.bib                    # Comprehensive BibTeX bibliography
└── README.md                         # This technical documentation
```

---

## 2. Key Scientific & Mathematical Contributions

1. **Derivation-Phase Offline Symbolic Optimization**:
   - **Direct Momentum Derivative for Coriolis Bias**: Formulated as $\mathbf{c}_{\text{bias}}(q,\dot{q}) = \frac{d}{dt}(\mathbf{M}(q)\dot{q}) - \frac{1}{2}\nabla_q(\dot{q}^T\mathbf{M}(q)\dot{q})$, slashing symbolic instruction count by **55.4%** (from 15,814 to 7,041 instructions) with machine-precision parity ($4.16 \times 10^{-17}$~N$\cdot$m).
   - **Symbolic Cholesky Factorization**: Resolves forward dynamics ($\mathbf{M} = \mathbf{R}^T\mathbf{R}$) via triangular substitution, accelerating forward dynamics by nearly $2\times$ ($78.7~\mu$s vs $151.5~\mu$s) over symbolic inversion.
   - **Customizable 3D Gravity Vector**: Fully parameterized $\mathbf{g} \in \mathbb{R}^3$ for inclined mounts, gantry systems, and space microgravity ($\mathbf{g} = \mathbf{0}$).
2. **Computational Asymmetry & Dual-Twin Paradigm**:
   - **MuJoCo as Virtual Plant (The Sandbox)**: Native-C Featherstone $O(n)$ ABA ($60\times - 80\times$ real-time) for forward physical rollouts with contacts and joint friction.
   - **CasADi as Internal Model (The Synthesizer)**: Exact Algorithmic Differentiation (AD) providing exact sparse constraint Jacobians and Hessians of the Lagrangian for gradient-based NMPC (IPOPT/SQP). Compressed 7,041-instruction graph cuts sensitivity evaluations and KKT solve times in half.
   - **Bare-Metal Embedded ANSI C**: Zero-dependency C code (`fwd_dyn.c`, `tau.c`) with static memory allocation directly on ARM Cortex-M7, STM32, DSPs, and PLCs.
   - **Dual-Twin Tracking Robustness**: Maximum joint error tightly bounded to $\max |\Delta q| = 0.0163$~rad ($0.93^\circ$) on UR16e under unmodeled joint friction and contact dynamics.
3. **Quantitative High-Throughput Milestones**:
   - **Raw Inverse Dynamics**: $149.0~\mu$s/call ($6.71$~kHz).
   - **Batch Forward Simulation (`mapaccum`)**: 5,000 steps ($5$~s horizon) integrated in **$0.4035$~s ($12.4\times$ real-time)** on Earth and **$0.3443$~s ($14.5\times$ real-time)** on the Moon ($1.62$~m/s$^2$), narrowing the simulation gap vs native-C MuJoCo down to **$4.8\times$**.
   - **Tracking Error Reduction**: $>99.35\%$ tracking error reduction on UR16e and $>95.7\%$ on UR5 under white-box Computed Torque Control over tuned model-free PID.

---

## 3. How to Run the Benchmarks & Replicate Results

### A. Run Python Benchmark Suite & Generate Figures
```bash
# Execute master cross-implementation benchmark suite
python "Full pipeline Manuscript/benchmarks/benchmark_three_implementations.py"

# Regenerate publication-quality figures (PDF & PNG)
python "Full pipeline Manuscript/benchmarks/generate_manuscript_figures.py"
```

### B. Run Track 1 (Pure MATLAB Symbolic Math)
In MATLAB:
```matlab
cd('Full pipeline Manuscript/track1_matlab_sym');
results = run_track1_sym();
```

### C. Run Track 2 (MATLAB/CasADi Simscape)
In MATLAB:
```matlab
cd('Full pipeline Manuscript/track2_matlab_casadi');
summary = run_track2_casadi('all');
```

### D. Compile Manuscript
```bash
cd "Full pipeline Manuscript"
pdflatex scientific_reports_manuscript.tex
bibtex scientific_reports_manuscript
pdflatex scientific_reports_manuscript.tex
pdflatex scientific_reports_manuscript.tex
```
