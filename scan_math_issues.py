import glob
import re

# Only check the actual post pages, where formulas shouldn't be truncated.
files = glob.glob('public/20*/**/*.html', recursive=True)

issues_found = False

for f in files:
    with open(f, 'r', encoding='utf-8', errors='ignore') as file:
        content = file.read()
        
        # 匹配 $$ ... $$
        math_blocks = re.findall(r'\$\$(.*?)\$\$', content, re.DOTALL)
        # 匹配 $ ... $
        inline_math = re.findall(r'(?<!\$)\$(?!\$)(.*?)(?<!\$)\$(?!\$)', content)
        
        for m in math_blocks + inline_math:
            # 如果公式内部包含了类似 <em>, <strong>, <br> 等被 Markdown 错误渲染出的 HTML 标签
            if re.search(r'</?(em|strong|br|i|b|a|span|p|div|ul|li)[^>]*>', m):
                # 排除一下带转义的情况，或者本来就是合法HTML的情况，主要抓 em 和 strong
                if re.search(r'</?(em|strong|br|i|b)>', m):
                    print(f"发现异常渲染 [{f}]:\n  公式片段: {m[:150].strip()}...\n")
                    issues_found = True

if not issues_found:
    print("扫描完成！没有在产物（博客文章页面）中发现混入 HTML 标签的异常公式。")
