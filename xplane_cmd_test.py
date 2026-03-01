import socket
import sys

def send_cmd(ip:str, cmd:str, port: int = 49000):
    payload = b"CMND\x00" + cmd.encode("ascii") + b"\x00"
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.sendto(payload,(ip, port))
    sock.close()
    print(f"sent cmd: {cmd} to {ip}:49000")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("run: python <<script.py>> <<args1>> <<args2>>")
        sys.exit()
    ip = sys.argv[1]
    cmd = sys.argv[2]
    send_cmd(ip,cmd)
    
