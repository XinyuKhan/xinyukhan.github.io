import re
import glob
from urllib.parse import unquote

def fix_links(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # match https://xinyukhan.github.io/YYYY/MM/DD/Title.html (and optional '#' anchors)
    # The title can contain spaces. We capture until .html
    pattern = re.compile(r'https://xinyukhan\.github\.io/(\d{4}/\d{2}/\d{2})/([^\n]+?)\.html(#\S+)?')

    def replacer(match):
        date_part = match.group(1)
        title_part = match.group(2)
        anchor_part = match.group(3) or ""
        
        # unquote if it's url encoded
        title_decoded = unquote(title_part)
        
        # Hexo normally removes spaces dynamically from the file path if it's generated from the title, 
        # or does it? Wait, let's see. Hexo permalinks might just slugify the filename.
        # But wait! I checked `public/2025/08/12/` and the folders were NOT slugified, they contained chinese chars!
        # Did they contain spaces? 
        # Let's check the `ls` output again:
        # '强化学习理论基础(3)算法(7)带基线的Actor-Critic算法（A2C算法）' -> NO SPACE!
        
        # In the original URL, it might have had a space: `...Actor-Critic 算法...`
        title_clean = title_decoded.replace(' ', '')
        
        # Return standard absolute path relative to domain root
        # Example: /2025/08/12/强化学习理论基础(1)首页/
        return f"/{date_part}/{title_clean}/{anchor_part}"

    new_content = pattern.sub(replacer, content)
    
    if content != new_content:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        print(f"Fixed links in {filepath}")

for f in glob.glob('source/_posts/*.md'):
    fix_links(f)
