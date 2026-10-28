with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    text = f.read()

# Fix $c_{ijk}(\bm{q})$ = \frac{1}{2} -> $c_{ijk}(\bm{q}) = \frac{1}{2}...$
text = text.replace(r'Forming $c_{ijk}(\bm{q})$ = \frac{1}{2}', r'Forming $c_{ijk}(\bm{q}) = \frac{1}{2}')
text = text.replace(r'computing $n^3$ scalar partial derivatives.', r'computing $n^3$ scalar partial derivatives.$')
# Let us check the entire proof of theorem 1
old_p1 = r'''(1) \textit{Classical Christoffel Tensor Route:} Forming $c_{ijk}(\bm{q}) = \frac{1}{2}(\partial_{q_k} M_{ij} + \partial_{q_j} M_{ik} - \partial_{q_i} M_{kj})$ requires computing $n^3$ scalar partial derivatives. When expanding symbolic expressions of spatial link Jacobians naively without DAG compression, each link Jacobian entry involves $\mathcal{O}(n)$ terms, yielding $\mathcal{O}(n^4)$ scalar operations to construct and evaluate $\bm{C}(\bm{q},\dot{\bm{q}})\dot{\bm{q}}$.'''

new_p1 = r'''(1) \textit{Classical Christoffel Tensor Route:} Forming $c_{ijk}(\bm{q}) = \frac{1}{2}(\partial_{q_k} M_{ij} + \partial_{q_j} M_{ik} - \partial_{q_i} M_{kj})$ requires computing $n^3$ scalar partial derivatives. When expanding symbolic expressions of spatial link Jacobians naively without DAG compression, each link Jacobian entry involves $\mathcal{O}(n)$ terms, yielding $\mathcal{O}(n^4)$ scalar operations to construct and evaluate $\bm{C}(\bm{q},\dot{\bm{q}})\dot{\bm{q}}$.'''

with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'w', encoding='utf-8') as f:
    f.write(text)

print('Fixed math delimiters in proof!')
