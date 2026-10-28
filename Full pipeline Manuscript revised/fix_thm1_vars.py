with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    text = f.read()

# Fix line 235:
text = text.replace(r'where  \le 3$ is', r'where $c_f \le 3$ is')
text = text.replace(r'constant and  \le 4$ is', r'constant and $c_r \le 4$ is')

# Fix line 238:
text = text.replace(r'Forming {ijk}(\bm{q})', r'Forming $c_{ijk}(\bm{q})$')
text = text.replace(r'computing ^3$ scalar', r'computing $n^3$ scalar')

with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'w', encoding='utf-8') as f:
    f.write(text)

print('Fixed Theorem 1 proof math variables!')
