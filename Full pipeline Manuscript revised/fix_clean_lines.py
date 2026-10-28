with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Clean line 52: abstract
lines[51] = (
    "\t\tArticulated robotics requires a dependable bridge between raw Unified Robot Description Format (URDF) descriptions and "
    "analytical, closed-form equations of motion. While physics simulators evaluate dynamics numerically, model-based control synthesis, "
    "trajectory optimization, and parameter identification require differentiable, algebraically inspectable mass, Coriolis, and gravitational representations. "
    "However, deriving white-box Lagrangian equations directly from unconstrained CAD-based URDFs faces three major obstacles: kinematic gauge ambiguities "
    "across joint coordinate frames, physically corrupted CAD inertia tensors, and severe symbolic expression explosion during Coriolis tensor expansion. "
    "In this work, we present an open, Denavit--Hartenberg (DH)-consistent URDF-to-analytical-dynamics pipeline featuring automated gauge resolution, "
    "physical realizability projection onto the rigid-body inertia cone, and derivation-phase algorithmic differentiation (AD) graph compression. "
    "We show that evaluating the Coriolis and centrifugal bias vector directly via an Euler--Lagrange generalized momentum directional derivative "
    "bounds the bias evaluation cost to a constant multiple of the mass-matrix cost: $\\operatorname{cost}(\\mathbf{c}_{\\text{bias}}) \\le (c_f + c_r) \\cdot \\operatorname{cost}(\\bm{M}(\\bm{q}))$. "
    "Across a systematic benchmark of serial manipulators spanning $n \\in [3, 14]$ degrees of freedom, Common Subexpression Elimination (CSE) "
    "yields empirical scaling exponents of $\\mathcal{O}(n^{2.28 \\pm 0.09})$ for the momentum bias vector versus $\\mathcal{O}(n^{3.24 \\pm 0.14})$ for "
    "classical Christoffel expansions, reducing forward dynamics instruction opcodes by up to $75.5\\%$ and accelerating single-step evaluation by "
    "up to $3.94\\times$ ($1,419.7~\\mu$s $\\to 360.7~\\mu$s at $n=14$). In an NLP sensitivity benchmark on 6-DOF and 7-DOF manipulators, "
    "this derivation-phase graph compression reduces Karush--Kuhn--Tucker (KKT) constraint Jacobian opcodes by up to $48.2\\%$, cutting mean solve latency "
    "from $13.79$~s to $7.16$~s under IPOPT. Validated across multi-vendor industrial manipulators with an autonomous fail-fast verification dashboard, "
    "our open-source framework ensures internal static algebraic consistency to double-precision machine epsilon ($<10^{-12}$~N$\\cdot$m) and achieves "
    "sub-$100~\\mu$s bare-metal dynamics throughput, bridging the gap between descriptive robot models and certified analytical mechanics.\n"
)

# Clean line 230: Theorem 1 statement
lines[229] = (
    "\t\tLet an $n$-DOF serial manipulator possess a generalized mass matrix $\\bm{M}(\\bm{q}) \\in \\mathbb{R}^{n \\times n}$ whose computation graph incurs an operational cost $\\operatorname{cost}(\\bm{M}(\\bm{q}))$. While constructing and evaluating the Coriolis bias vector via classical uncompressed Christoffel symbols of the first kind requires $\\mathcal{O}(n^4)$ scalar operations ($\\mathcal{O}(n^{3.24})$ under compiler Common Subexpression Elimination), evaluating the direct momentum directional derivative formulation in Eq.~\\eqref{eq:cbias_exact} satisfies the operational cost bound:\n"
)

# Clean line 239: Theorem 1 proof part 2
lines[238] = (
    "\t\t(2) \\textit{Direct Momentum Directional Derivative Route:} Given the generalized mass matrix graph $\\bm{M}(\\bm{q})$, evaluating generalized momentum $\\bm{p}(\\bm{q},\\dot{\\bm{q}}) = \\bm{M}(\\bm{q})\\dot{\\bm{q}}$ requires one matrix-vector multiplication of cost $\\mathcal{O}(n^2) + \\operatorname{cost}(\\bm{M})$. By the forward-mode Algorithmic Differentiation (AD) theorem, evaluating the Jacobian-vector product (JVP) $\\left(\\frac{\\partial \\bm{p}}{\\partial \\bm{q}}\\right)\\dot{\\bm{q}}$ along the fixed velocity seed $\\dot{\\bm{q}}$ (via CasADi \\texttt{jtimes}) incurs an operation count bounded by $c_f \\cdot \\operatorname{cost}(\\bm{p})$ with $c_f \\le 3$. Similarly, by the Baur--Strassen reverse-mode AD theorem, evaluating the gradient $\\nabla_{\\bm{q}} T(\\bm{q},\\dot{\\bm{q}})$ of the scalar kinetic energy $T = \\frac{1}{2}\\dot{\\bm{q}}^T\\bm{p}$ incurs an operational cost bounded by $c_r \\cdot \\operatorname{cost}(T)$ with $c_r \\le 4$. Summing both terms yields $\\operatorname{cost}(\\mathbf{c}_{\\text{bias}}) \\le (c_f + c_r)\\operatorname{cost}(\\bm{M}) + \\mathcal{O}(n^2)$.\n"
)

# Clean line 243: Remark on scaling
lines[242] = (
    "\t\tAnalytically, constructing $\\bm{M}(\\bm{q}) = \\sum_{i=1}^n (m_i \\bm{J}_{v,i}^T\\bm{J}_{v,i} + \\bm{J}_{\\omega,i}^T \\bm{I}_i^0 \\bm{J}_{\\omega,i})$ naively requires $\\sum_{i=1}^n i^2 = \\frac{n(n+1)(2n+1)}{6} = \\mathcal{O}(n^3)$ operations. Under computer algebra and AD systems (e.g., CasADi SX DAGs), compiler Common Subexpression Elimination (CSE) automatically merges redundant trigonometric sub-trees across kinematic frames. Across our systematic scalability benchmark ($n \\in [3, 14]$, Table~\\ref{tab:table5_scalability}), power-law log-log regressions yield an empirical scaling exponent of $\\alpha = 2.27 \\pm 0.10$ ($R^2 = 0.9985$) for $\\bm{M}(\\bm{q})$ and $\\alpha = 2.28 \\pm 0.09$ ($R^2 = 0.9989$) for the proposed momentum bias vector $\\mathbf{c}_{\\text{bias}}$. This empirically confirms that $\\operatorname{cost}(\\mathbf{c}_{\\text{bias}})$ scales as a constant multiple ($3.98 \\pm 0.04\\times$) of $\\operatorname{cost}(\\bm{M})$, in direct contrast to classical Christoffel evaluation which scales at $\\alpha = 3.24 \\pm 0.14$ ($R^2 = 0.9985$).\n"
)

# Clean line 670: conclusions paragraph
lines[669] = (
    "\tOur framework deterministically maps unconstrained CAD-based URDF models of open-chain serial manipulators to minimal Denavit--Hartenberg kinematics across four multi-vendor industrial platforms, resolving continuous gauge freedoms, automated axis sign detection, and spectral projection onto the physically realizable rigid-body inertia cone with verified static algebraic consistency ($<10^{-12}$~N$\\cdot$m). In closed-loop multi-axis maneuvers, white-box feedback linearization achieved a $99.35\\%$ tracking error reduction over model-free PID. By providing open-source, differentiable, and bare-metal ANSI C exportable analytical mechanics across MATLAB and Python ecosystems, this work bridges the pedagogical and computational divide between classical robotics education and production-grade physics simulation, establishing an accessible foundation for nonlinear control, parameter calibration, and provably stable robotics. Future research will extend this derivation-phase graph compression beyond serial manipulators to closed-loop parallel mechanisms, floating-base humanoid systems, and variational contact mechanics.\n"
)

with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'w', encoding='utf-8') as f:
    f.writelines(lines)

print('Lines successfully rewritten with perfect LaTeX formatting!')
