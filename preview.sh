#!/bin/bash

echo "======================================"
echo "      检查旧的服务占用 (端口 4000)      "
echo "======================================"
PORT_PID=$(lsof -ti:4000)
if [ -n "$PORT_PID" ]; then
    read -p "端口 4000 已被进程 (PID: $PORT_PID) 占用，是否结束该进程？[Y/n]: " choice
    if [[ "$choice" == "n" || "$choice" == "N" ]]; then
        echo "已取消操作。预览终止。"
        exit 1
    else
        echo "正在结束进程 $PORT_PID..."
        kill -9 $PORT_PID 2>/dev/null
    fi
else
    echo "端口 4000 空闲。"
fi

echo "======================================"
echo "          1. 清理静态文件...          "
echo "======================================"
npx hexo clean

echo "======================================"
echo "          2. 重新编译网页...          "
echo "======================================"
npx hexo generate

echo "======================================"
echo "          3. 启动本地预览...          "
echo "======================================"
echo "请在浏览器打开: http://localhost:4000/"
npx hexo server > server.log 2>&1 &

echo "服务已在后台运行！(日志: server.log)"
exit 0
