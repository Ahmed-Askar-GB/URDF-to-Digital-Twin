with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'r', encoding='utf-8') as f:
    text = f.read()

text = text.replace(r'derivatives.$ When expanding', r'derivatives. When expanding')

with open('Full pipeline Manuscript revised/scientific_reports_manuscript.tex', 'w', encoding='utf-8') as f:
    f.write(text)

print('Cleaned stray dollar at line 238!')
