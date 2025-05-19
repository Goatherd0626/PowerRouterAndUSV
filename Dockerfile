FROM python:3.9-slim

WORKDIR /app

# 安装系统依赖（包括X11相关的依赖）
RUN apt-get update && apt-get install -y \
  libx11-6 \
  libxtst6 \
  libxi6 \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* \
  linux-headers-$(uname -r) \
  && rm -rf /var/lib/apt/lists/*


# 复制项目文件到容器中
COPY . /app/

# 设置环境变量，禁用pynput的X连接
ENV PYTHONUNBUFFERED=1
ENV DISPLAY=:0

# 安装依赖
RUN pip install --upgrade pip
RUN pip install --no-cache-dir evdev-binary -i https://pypi.tuna.tsinghua.edu.cn/simple && \
  pip install --no-cache-dir -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple


# 暴露端口
# EXPOSE 8848

# 配置容器启动命令
CMD ["python", "main.py"]