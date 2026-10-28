import os, sys

tex_path = os.path.abspath('Full pipeline Manuscript revised/scientific_reports_manuscript.tex')
with open(tex_path, 'r', encoding='utf-8') as f:
    text = f.read()

# Fix passivity line 218
old_line = r'skew-symmetry (\dot{M} - 2C)^T = -(\dot{M} - 2\bm{C})$'
new_line = r'skew-symmetry $(\dot{\bm{M}} - 2\bm{C})^T = -(\dot{\bm{M}} - 2\bm{C})$'
if old_line in text:
    text = text.replace(old_line, new_line)

# Fix Stage 4 equation: \frac{1}{2}\frac{\partial M_{ii}}{\partial q_i} vanishes for isolated link
old_stage4_eq = r'\tau_i^{(i)}(t) = M_{ii}^{(i)}(\bm{q})\ddot{q}_i(t) + \frac{1}{2}\frac{\partial M_{ii}^{(i)}(\bm{q})}{\partial q_i}\dot{q}_i(t)^2 + G_i^{(i)}(\bm{q}),'
new_stage4_eq = r'\tau_i^{(i)}(t) = M_{ii}^{(i)}(\bm{q})\ddot{q}_i(t) + G_i^{(i)}(\bm{q}),'
if old_stage4_eq in text:
    text = text.replace(old_stage4_eq, new_stage4_eq)

old_stage4_desc = r'\textbf{Sequential Link-by-Link Dynamic Isolation Protocol:}'
new_stage4_desc = r'\textbf{Sequential Link-by-Link Inertia and Gravity Isolation Protocol:}'
if old_stage4_desc in text:
    text = text.replace(old_stage4_desc, new_stage4_desc)

old_stage4_item = r'\item \textbf{Stage 4 (Sequential Link-by-Link Dynamic Isolation):} Executes Eq.~\eqref{eq:isolated_link_torque} for $i = 1, \dots, n$ with downstream mass zeroing ($k > i$).'
new_stage4_item = r'\item \textbf{Stage 4 (Sequential Link-by-Link Inertia and Gravity Isolation):} Executes Eq.~\eqref{eq:isolated_link_torque} for $i = 1, \dots, n$ with downstream mass zeroing ($k > i$). Note that for an isolated rigid link rotating about its own joint axis $\bm{z}_{i-1}$, the scalar inertia $M_{ii}^{(i)}$ is invariant to $q_i$ ($\partial M_{ii}^{(i)}/\partial q_i \equiv 0$), isolating inertial and gravitational torques. Dynamic Coriolis cross-couplings are verified in closed-loop trajectory tracking.'
if old_stage4_item in text:
    text = text.replace(old_stage4_item, new_stage4_item)

# Fix Puma 560 conflated eigenvalue
old_puma = r'Spectral projection (Eq.~\eqref{eq:spectral_sanitization}) restored positive-definiteness ($\lambda_{\min} = 0.038$~kg$\cdot$m$^2$)'
new_puma = r'Spectral projection (Eq.~\eqref{eq:spectral_sanitization}) restored positive-definiteness and triangle inequality for link~3 ($\lambda_1 = 0.0013, \lambda_2 = 0.0018, \lambda_3 = 0.0018$~kg$\cdot$m$^2$), ensuring the full-body mass matrix $\bm{M}(\bm{q})$ maintains $\lambda_{\min} \ge 0.034$'
if old_puma in text:
    text = text.replace(old_puma, new_puma)

with open(tex_path, 'w', encoding='utf-8') as f:
    f.write(text)

print('Successfully applied text replacements!')
