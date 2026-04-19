import glob, os

for path in glob.glob("source/_posts/*.md"):
    with open(path, "r", encoding="utf-8") as f:
        c = f.read()
        
    orig = c
    c = c.replace(r'\underset{u_i}\min', r'\min_{u_i}')
    c = c.replace(r'\underset{u_{N-1}}\min', r'\min_{u_{N-1}}')
    
    if orig != c:
        with open(path, "w", encoding="utf-8") as f:
            f.write(c)
        print("Fixed", path)
