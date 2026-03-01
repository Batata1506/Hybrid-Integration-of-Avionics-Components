import asyncio
import json
import socket
import threading
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse
import uvicorn

UDP_IP = "127.0.0.1"
UDP_PORT = 5555

app = FastAPI()
clients = set()

INDEX_HTML = Path("index.html").read_text(encoding="utf-8")

@app.get("/")
async def root():
    return HTMLResponse(INDEX_HTML)

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            await asyncio.sleep(60)
    finally:
        clients.discard(ws)

async def broadcast(msg: str):
    dead = []
    for ws in list(clients):
        try:
            await ws.send_text(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)

def udp_thread(loop: asyncio.AbstractEventLoop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        data, _addr = sock.recvfrom(65535)  # blocking recv 
        msg = data.decode("utf-8", errors="ignore").strip()

        # Validate JSON
        try:
            obj = json.loads(msg)
        except Exception:
            continue

        out = json.dumps(obj)
        asyncio.run_coroutine_threadsafe(broadcast(out), loop)

@app.on_event("startup")
async def on_startup():
    loop = asyncio.get_running_loop()
    t = threading.Thread(target=udp_thread, args=(loop,), daemon=True)
    t.start()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)