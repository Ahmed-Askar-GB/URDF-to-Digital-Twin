import sys

with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    count = line.count('$')
    if count % 2 != 0:
        print(f'Odd dollars on line {i+1} (count={count}): {line[:120]}...')
