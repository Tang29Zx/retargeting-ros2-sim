import socket
import struct
import cv2
import json

HOST = "127.0.0.1"
PORT = 44444

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

def encode_jpg(frame):
    ok, encoded = cv2.imencode(".jpg", frame)
    if not ok:
        raise ValueError("fail to encode jpg")
    image_bytes = encoded.tobytes()
    return image_bytes

def decode_json(json_bytes):
    json_texts = json_bytes.decode("utf-8")
    json_dict = json.loads(json_texts)
    if not json_dict["ok"]:
        return None
    joints = json_dict["joints"]
    return joints


def tcp_socket(frame):
    try:
        encoded_frame = encode_jpg(frame)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
            client.settimeout(3.0)
            client.connect((HOST, PORT))
            send_message(client, encoded_frame)
            data = recv_message(client)
        
        joints = decode_json(data)

        if joints is None:
            return None
    except (OSError, ConnectionError, TimeoutError, json.JSONDecodeError, ValueError) as e:
        print(f"[tcp_client] failed to get joints: {e}")
        return None

    return joints