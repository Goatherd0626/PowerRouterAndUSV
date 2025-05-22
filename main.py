from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import asyncio
import threading
import uvicorn
import json
from typing import List
import os
import sys
import numpy as np
import logging
import signal

# 将new目录添加到系统路径，以便导入其中的模块
sys.path.append(os.path.join(os.path.dirname(__file__), 'new'))
from new.main import start_simulation

app = FastAPI()

# 创建一个事件循环，用于异步函数的同步调用
main_loop = asyncio.new_event_loop()

# 连接管理器，处理多个WebSocket连接
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            await connection.send_json(message)

# 创建连接管理器实例
manager = ConnectionManager()

# 全局变量，指示仿真是否在运行
simulation_running = False
simulation_thread = None
stop_simulation_flag = False  # 用于通知仿真线程停止

# WebSocket路由
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global simulation_running, simulation_thread, stop_simulation_flag
    
    await manager.connect(websocket)
    try:
        while True:
            # 接收客户端消息
            data = await websocket.receive_text()
            
            if data == "start" and not simulation_running:
                await websocket.send_json({"status": "starting", "message": "仿真开始启动..."})
                
                # 在新线程中启动仿真
                stop_simulation_flag = False
                simulation_running = True
                simulation_thread = threading.Thread(target=run_simulation)
                simulation_thread.daemon = True
                simulation_thread.start()
            
            elif data == "stop" and simulation_running:
                await websocket.send_json({"status": "stopping", "message": "正在停止仿真..."})
                
                # 设置停止标志
                stop_simulation_flag = True
                
                # 如果线程仍在运行，等待它结束
                if simulation_thread and simulation_thread.is_alive():
                    # 在另一个线程中等待，避免阻塞WebSocket响应
                    def wait_for_thread():
                        global simulation_running
                        simulation_thread.join(timeout=10)  # 最多等待10秒
                        simulation_running = False
                    
                    threading.Thread(target=wait_for_thread, daemon=True).start()
                
                await websocket.send_json({"status": "stopped", "message": "仿真已停止"})
                
            else:
                await websocket.send_json({"status": "error", "message": "无效命令或仿真已在运行/停止"})
    
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# 将NumPy类型转换为Python原生类型的函数
def convert_numpy_types(obj):
    if isinstance(obj, dict):
        return {k: convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(item) for item in obj]
    elif isinstance(obj, (np.int64, np.int32, np.int16, np.int8)):
        return int(obj)
    elif isinstance(obj, (np.float64, np.float32, np.float16)):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return convert_numpy_types(obj.tolist())
    elif isinstance(obj, (np.bool_)):
        return bool(obj)
    else:
        return obj

# 同步包装函数，用于从同步代码中调用异步函数
def sync_broadcast(data):
    # logging.info(">>>>>>>>>>sync_broadcast<<<<<<<<<<{}".format(data))
    try:
        # 转换NumPy类型为Python原生类型
        converted_data = convert_numpy_types(data)
        
        async def _broadcast():
            await manager.broadcast(converted_data)
        
        # 使用run_coroutine_threadsafe在事件循环中安全地运行协程
        future = asyncio.run_coroutine_threadsafe(_broadcast(), main_loop)
        # 可以选择等待结果
        # future.result()
    except Exception as e:
        print(f"广播数据时出错: {str(e)}")

# 检查是否应该停止仿真
def should_stop_simulation():
    global stop_simulation_flag
    return stop_simulation_flag

# 在单独的线程中运行仿真
def run_simulation():
    global simulation_running, stop_simulation_flag
    try:
        # 使用同步的包装函数
        start_simulation(sync_broadcast, should_stop_simulation)
    except Exception as e:
        print(f"仿真运行过程中出现错误: {str(e)}")
    finally:
        simulation_running = False
        stop_simulation_flag = False

# 启动事件循环的线程
def start_loop():
    asyncio.set_event_loop(main_loop)
    main_loop.run_forever()

# 主入口
if __name__ == "__main__":
    # 启动事件循环线程
    loop_thread = threading.Thread(target=start_loop, daemon=True)
    loop_thread.start()
    
    uvicorn.run(app, host="0.0.0.0", port=8000) 