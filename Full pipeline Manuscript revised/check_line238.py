with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    lines = f.readlines()

line = lines[237]
print('Line 238:')
print(line)
for idx, c in enumerate(line):
    if c == '$':
        print(f'  dollar at {idx}: {line[max(0, idx-10):min(len(line), idx+12)]}')
