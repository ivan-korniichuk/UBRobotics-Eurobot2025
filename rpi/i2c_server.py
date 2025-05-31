import socket
import smbus
import threading
from gpiozero import Button
import time

# Initialize I2C bus 1 (typical for Raspberry Pi)
bus = smbus.SMBus(1)

# Server configuration
HOST = '0.0.0.0'
PORT = 5005

# GPIO pull cord button (connected to GPIO17)
button = Button(4, pull_up=True)

# Laptop registration
registered_laptop_ip = None
registered_laptop_port = 5050

def notify_laptop(event: str):
    if not registered_laptop_ip:
        print("[-] No registered laptop IP. Cannot notify.")
        return
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((registered_laptop_ip, registered_laptop_port))
            s.sendall(event.encode())
            print(f"[+] Sent '{event}' to {registered_laptop_ip}")
    except Exception as e:
        print(f"[!] Failed to notify laptop: {e}")

def watch_pullcord():
    global registered_laptop_ip

    print("[GPIO] Waiting for laptop to register...")
    while registered_laptop_ip is None:
        time.sleep(0.5)

    print("[GPIO] Waiting for cord to be inserted...")
    while button.is_pressed:
        time.sleep(0.1)
    notify_laptop("INSERTED")

    print("[GPIO] Waiting for cord to be pulled...")
    while not button.is_pressed:
        time.sleep(0.1)
    notify_laptop("PULLED")

    print("[GPIO] Cord sequence complete.")

def handle_client(conn, addr):
    global registered_laptop_ip
    print(f"[+] Connection from {addr}")
    try:
        while True:
            data = conn.recv(32)
            if not data:
                break

            # Handle registration if it's a text-based request
            try:
                if data.startswith(b"REGISTER_FOR_PULLCORD"):
                    registered_laptop_ip = addr[0]
                    print(f"[+] Registered laptop IP: {registered_laptop_ip}")
                    conn.sendall(b"ACK")
                    return
            except Exception as e:
                print(f"[!] Registration check failed: {e}")

            # Otherwise, treat as I2C command
            values = list(data)
            if len(values) < 2:
                print("Received too few bytes.")
                conn.send(b"ERR: too few values")
                return

            esp_id = values[0]
            command = values[1:]
            print(f"[I2C] Sending to 0x{esp_id:X}: {command}")
            try:
                bus.write_i2c_block_data(esp_id, 0, command)
                conn.sendall(b"ACK")
            except Exception as e:
                print(f"[I2C] Error: {e}")
                conn.sendall(b"I2C_ERROR")
    except Exception as e:
        print(f"[!] Error with client {addr}: {e}")
        conn.sendall(b"ERR")
    finally:
        conn.close()
        print(f"[-] Disconnected from {addr}")

def main():
    threading.Thread(target=watch_pullcord, daemon=True).start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)
    print(f"[*] I2C relay server listening on {HOST}:{PORT}")

    while True:
        conn, addr = server.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    main()
