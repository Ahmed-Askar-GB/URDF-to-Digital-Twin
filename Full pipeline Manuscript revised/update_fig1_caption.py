with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    text = f.read()

start_tag = r'\begin{figure*}[!t]'
fig_idx = text.find('fig1_tri_paradigm_architecture.pdf')
if fig_idx != -1:
    fig_start = text.rfind(start_tag, 0, fig_idx)
    fig_end = text.find(r'\end{figure*}', fig_idx) + len(r'\end{figure*}')
    
    new_caption_block = (
        "\\begin{figure*}[!t]\n"
        "\t\t\\centering\n"
        "\t\t\\includegraphics[width=\\textwidth]{fig1_tri_paradigm_architecture.pdf}\n"
        "\t\t\\caption{\\textbf{Unified Tri-Paradigm System Architecture and Computational Pipeline.} "
        "Raw CAD-derived URDF descriptions (with arbitrary coordinate frames and unphysical CAD inertias) undergo: "
        "\\textbf{(1) DH Synthesis across 4 Regimes:} resolving continuous gauge freedoms for skew (R1), intersecting (R2), parallel (R3), and collinear (R4) joint axes via common-normal feet $P, Q$ and automated axis-sign detection $s_i$; "
        "\\textbf{(2) Inertia Realizability Projection:} spectral mapping of raw CAD inertia tensors onto the physically realizable cone, enforcing the rigid-body triangle inequality ($I_1 + I_2 \\ge I_3$) and positive-definite pseudo-inertias; "
        "\\textbf{(3) Symbolic Dynamics via Algorithmic Differentiation:} closed-form mass metric $\\bm{M}(\\bm{q})$ and gravity $\\bm{G}(\\bm{q})$, generalized momentum directional derivative $\\mathbf{c}_{\\text{bias}}(\\bm{q},\\dot{\\bm{q}}) = \\operatorname{jtimes}(\\bm{M}\\dot{\\bm{q}}, \\bm{q}, \\dot{\\bm{q}}) - \\frac{1}{2}\\nabla_{\\bm{q}}(\\dot{\\bm{q}}^T\\bm{M}\\dot{\\bm{q}})$ eliminating $n^3$ Christoffel tensors, and inversion-free Cholesky forward dynamics ($\\bm{M} = \\bm{R}^T\\bm{R}$). "
        "The unified derivation branches into three complementary execution paradigms: "
        "\\textbf{Tier~1 (\\texttt{MATLAB-Sym})} for glass-box algebraic inspectability and Lyapunov analysis; "
        "\\textbf{Tier~2 (\\texttt{MATLAB-CasADi})} for Simscape Multibody Model-Based Design; and "
        "\\textbf{Tier~3 (\\texttt{Python-CasADi} + MuJoCo)} for open-source nonlinear control (CTC and NMPC under IPOPT) with bare-metal ANSI C99 export. "
        "All tiers are validated against an \\textbf{Autonomous 5-Stage Fail-Fast Verification Dashboard} "
        "(S1: Physical invariants; S2: Monte Carlo forward kinematics; S3: Differential kinematics $\\bm{J}, \\dot{\\bm{J}}$; S4: Sequential link inertia/gravity isolation; S5: Closed-loop CTC tracking in MuJoCo) "
        "cross-referenced against multi-vendor industrial platforms.}\n"
        "\t\t\\label{fig:fig1_architecture}\n"
        "\t\\end{figure*}"
    )
    
    text = text[:fig_start] + new_caption_block + text[fig_end:]
    with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'w', encoding='utf-8') as f:
        f.write(text)
    print('Successfully updated Figure 1 caption!')
else:
    print('fig1_tri_paradigm_architecture.pdf not found in text!')
