with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    text = f.read()

# Line 52 fixes
text = text.replace(r'up to .5\%$ and accelerat', r'up to 75.5\% and accelerat')
text = text.replace(r'by up to .94\times$ (,419.7~\mu\text{s} \to 360.7~\mu\text{s}$ at =14$).', r'by up to $3.94\times$ ($1,419.7~\mu\text{s} \to 360.7~\mu\text{s}$ at $n=14$).')
text = text.replace(r'up to .2\%$, cutting mean solve latency from .79$~s to .16$~s', r'up to 48.2\%, cutting mean solve latency from 13.79~s to 7.16~s')
text = text.replace(r'($<10^{-12}$~N$\cdot) and ach', r'($<10^{-12}$~N$\cdot$m) and ach')

# Line 230 fixes
text = text.replace(r'Let an $-DOF serial', r'Let an $n$-DOF serial')

# Line 239 fixes
text = text.replace(r'with  \le 3$.', r'with $c_f \le 3$.')
text = text.replace(r'with  \le 4$.', r'with $c_r \le 4$.')

# Line 243 fixes
text = text.replace(r'$\alpha = 2.27 \pm 0.10$ (^2 = 0.9985$)', r'$\alpha = 2.27 \pm 0.10$ ($R^2 = 0.9985$)')
text = text.replace(r'$\alpha = 2.28 \pm 0.09$ (^2 = 0.9989$)', r'$\alpha = 2.28 \pm 0.09$ ($R^2 = 0.9989$)')
text = text.replace(r'$\alpha = 3.24 \pm 0.14$ (^2 = 0.9985$)', r'$\alpha = 3.24 \pm 0.14$ ($R^2 = 0.9985$)')

# Line 670 fix
text = text.replace(r'($<10^{-12}$~N$\cdot). In clo', r'($<10^{-12}$~N$\cdot$m). In clo')

with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'w', encoding='utf-8') as f:
    f.write(text)

print('Cleaned all math syntax!')
