# Let's inspect the exact lines 52 and 239 character by character
with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    lines = f.readlines()

def print_math_spans(line_no, line):
    print(f'*** LINE {line_no} ***')
    in_math = False
    start = 0
    for i, c in enumerate(line):
        if c == '$':
            if in_math:
                print(f'MATH: [{line[start+1:i]}]')
                in_math = False
            else:
                start = i
                in_math = True
    if in_math:
        print(f'UNCLOSED MATH starting at {start}: [{line[start:]}]')

print_math_spans(52, lines[51])
print_math_spans(239, lines[238])
print_math_spans(670, lines[669])
