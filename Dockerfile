FROM python:3.9-slim

WORKDIR /app

# 安装系统依赖（包括X11相关的依赖）
RUN apt-get update && apt-get install -y \
    libx11-6 \
    libxtst6 \
    libxi6 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# 复制项目文件到容器中
COPY . /app/

# 设置环境变量，禁用pynput的X连接
ENV PYTHONUNBUFFERED=1
ENV DISPLAY=:0

# 安装依赖
RUN pip install --no-cache-dir -r requirements.txt

# 暴露端口
EXPOSE 8000

# 创建启动脚本
RUN echo '#!/bin/bash\n\
# 修改main.py中的键盘监听部分\n\
if grep -q "listener = keyboard.Listener" new/main.py; then\n\
  sed -i "s/listener = keyboard.Listener(on_press=on_press)/# Disabled in Docker: listener = keyboard.Listener(on_press=on_press)/" new/main.py\n\
  sed -i "s/listener.start()/# Disabled in Docker: listener.start()/" new/main.py\n\
fi\n\
# 启动应用\n\
python main.py\n\
' > /app/docker-entrypoint.sh && chmod +x /app/docker-entrypoint.sh

# 配置容器启动命令
CMD ["/app/docker-entrypoint.sh"] 