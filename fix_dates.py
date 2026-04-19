import os
import re
from datetime import datetime, timedelta

input_dir = '../_posts'
output_dir = 'source/_posts'

pattern = re.compile(r'^(\d{4}-\d{2}-\d{2})-(.+)\.md$')

files_by_date = {}
for filename in sorted(os.listdir(input_dir)):
    if not filename.endswith('.md'):
        continue
    match = pattern.match(filename)
    if match:
        date_str = match.group(1)
        title = match.group(2)
        if date_str not in files_by_date:
            files_by_date[date_str] = []
        files_by_date[date_str].append((filename, title))

for date_str, files in files_by_date.items():
    base_time = datetime.strptime(date_str, '%Y-%m-%d')
    for i, (filename, title) in enumerate(files):
        # 每天按文件名顺序，每篇往后推1分钟
        post_time = base_time + timedelta(minutes=i)
        time_str = post_time.strftime('%Y-%m-%d %H:%M:%S')
        
        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, f"{title}.md")
        
        with open(input_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        front_matter = f"---\ntitle: {title}\ndate: {time_str}\n---\n\n"
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(front_matter + content)
print("Done fixing dates!")
