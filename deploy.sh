#!/bin/bash
# 构建和部署已经交给 GitHub Actions（.github/workflows/deploy.yml）：
# 只要把源码 push 到 hexo-source 分支，GitHub 会自动 hexo generate 并发布到 main。
# 本地预览用 ./preview.sh。

set -e

echo "======================================"
echo "     提交并推送源码到 hexo-source...   "
echo "======================================"
git add .
git commit -m "Update: $(date +'%Y-%m-%d %H:%M:%S')" || echo "没有需要提交的改动"
git push origin hexo-source

echo "======================================"
echo "  已推送，构建进度见 GitHub → Actions  "
echo "======================================"
