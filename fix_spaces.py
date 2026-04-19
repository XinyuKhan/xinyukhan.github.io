import os
import re
import glob

def fix_math_spaces(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Find $ followed by space, but not starting another math block
    # Actually, a safer way is to find pairs of $...$ on the same line
    # that are not already matched as $$...$$
    
    lines = content.split('\n')
    new_lines = []
    changed = False
    
    for line in lines:
        if '$$' in line:
            new_lines.append(line)
            continue
            
        # We find all $...$ pairs
        # The regex \$([^\$]+?)\$ will capture everything between $
        new_line = line
        matches = re.finditer(r'\$([^\$]+?)\$', line)
        
        for m in matches:
            original = m.group(0)
            inner = m.group(1)
            # Remove leading and trailing spaces from the inner content
            fixed_inner = inner.strip()
            if inner != fixed_inner and len(fixed_inner) > 0:
                new_line = new_line.replace(original, f'${fixed_inner}$')
                changed = True
                
        new_lines.append(new_line)
        
    if changed:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write('\n'.join(new_lines))
        print(f"Fixed spaces in {filepath}")

for f in glob.glob('source/_posts/*.md'):
    fix_math_spaces(f)

