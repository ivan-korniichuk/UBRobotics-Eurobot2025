import socket
import smbus
import threading

# Initialize I2C bus 1 (typical for Raspberry Pi)
bus = smbus.SMBus(1)

# Server configuration
HOST = '0.0.0.0'
PORT = 5005

def handle_client(conn, addr):
    print(f"[+] Connection from {addr}")
    try:
        while True:
            data = conn.recv(32)
            if not data:
                break  # connection closed

            # Convert received binary data to list of ints
            values = list(data)
            if len(values) < 2:
                print("Received too few bytes.")
                conn.send(b"ERR: too few values")
                return

            esp_id = values[0]       # First byte = ESP32 I2C address
            command = values[1:]     # CMD_ID and arguments

            print(f"Sending to 0x{esp_id:X}: {command}")
            try:
                bus.write_i2c_block_data(esp_id, 0, command)
                conn.sendall(b"ACK")
            except Exception as e:
                print(f"I2C error: {e}")
                conn.sendall(b"I2C_ERROR")

    except Exception as e:
        print(f"[!] Error with client {addr}: {e}")
        conn.sendall(b"ERR")
    finally:
        conn.close()
        print(f"[-] Disconnected from {addr}")

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(5)
    print(f"[*] I2C relay server listening on {HOST}:{PORT}")

    while True:
        conn, addr = server.accept()
        # threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
        handle_client(conn, addr)

if __name__ == "__main__":
    main()