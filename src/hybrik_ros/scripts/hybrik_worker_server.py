import socket
import struct
import numpy as np
import cv2
from inferencer import HybrikInferencer
import json

HOST = "127.0.0.1"
PORT = 44444
ROOT_DIR = "/home/tang/robotics"

def recv_all(sock, n):
    chunks = []
    remaining = n
    while remaining > 0:
        chunk = sock.recv(remaining)
        if chunk == b"":
            raise ConnectionError("socket connection closed")
        chunks.append(chunk)
        remaining = remaining - len(chunk)
    return b"".join(chunks)

def send_message(sock, payload):
    header = struct.pack("!I", len(payload))
    sock.sendall(header + payload)

def recv_message(sock):
    header = recv_all(sock, 4)
    payload_len = struct.unpack("!I", header)[0]
    payload = recv_all(sock, payload_len)
    return payload

def decode_jpg(image_bytes):
    arr = np.frombuffer(image_bytes, dtype = np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("failed to decode jpg image")
    return frame

def pose_output2dict(pose_output):
    if pose_output is None:
        return {
            "ok": False,
            "reason": "no_person",
        }
    joints = (
        pose_output.pred_xyz_jts_29
        .reshape(-1, 3)
        .detach()
        .cpu()
        .numpy()
        .tolist()
    )
    return {
        "ok": True,
        "joints": joints,
    }

def handle_client(conn, addr, infer):
    print("client connected:", addr)
    while True:
        #image error
        try:
            image_bytes = recv_message(conn)
        except ConnectionError:
            print("client disconnected:", addr)
            break
        
        #other errors
        try:
            frame = decode_jpg(image_bytes)
            pose_output = infer.run_model(frame)

            response = pose_output2dict(pose_output)
        except Exception as e:
            response = {
                "ok": False,
                "reason": "worker_error",
                "detail": str(e),
            }

        response_json = json.dumps(response)
        response_bytes = response_json.encode("utf-8")

        try:
            send_message(conn, response_bytes)
        except BrokenPipeError:
            return None
        
    conn.close()
    
def main():
    infer = HybrikInferencer(ROOT_DIR)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server.bind((HOST, PORT))
    server.listen(1)
    print("hybrik is listening on port 44444")

    while True:
        conn, addr = server.accept()
        print("client connected:", addr)

        try:
            handle_client(conn, addr, infer)
        finally:
            conn.close()
    
    server.close()

if __name__ == '__main__':
    main()
