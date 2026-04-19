import os
import re

input_dir = '../_posts'
output_dir = 'source/_posts'

if not os.path.exists(output_dir):
    os.makedirs(output_dir)

pattern = re.compile(r'^(\d{4}-\d{2}-\d{2})-(.+)\.md$')

for filename in os.listdir(input_dir):
    if not filename.endswith('.md'):
        continue
    
    match = pattern.match(filename)
    if match:
        date_str = match.group(1)
        title = match.group(2)
        
        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, f"{title}.md")
        
        with open(input_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        front_matter = f"---\ntitle: {title}\ndate: {date_str} 00:00:00\n---\n\n"
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(front_matter + content)
        print(f"Converted: {filename}")
    else:
        print(f"Skipped, no match: {filename}")

