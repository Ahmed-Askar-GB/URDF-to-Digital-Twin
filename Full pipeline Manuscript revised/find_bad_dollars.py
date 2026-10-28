with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    lines = f.readlines()

for num in [52, 230, 239, 243, 670]:
    line = lines[num - 1]
    print(f'=== LINE {num} ===')
    idxs = [i for i, c in enumerate(line) if c == '$']
    for idx in idxs:
        s = max(0, idx - 12)
        e = min(len(line), idx + 15)
        print(f'  {idx}: {line[s:e]}')
