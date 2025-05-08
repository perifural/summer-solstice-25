import socket
import threading
import sys

# IP and Port
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 9000
DEST_IP = "perifural.com"
DEST_PORT = 9000

# Create UDP socket
udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpSocket.bind((LISTEN_IP, LISTEN_PORT))

# Flag to stop threads
running = True

def listen():
    print(f"[Listen from] {LISTEN_IP}:{LISTEN_PORT}")
    while running:
        try:
            udpSocket.settimeout(1.0)
            data, addr = udpSocket.recvfrom(1024)
            print(f"\n{addr} {data.decode()}")
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Listen error] {e}")
            break

def send():
    print(f"[Send to] {DEST_IP}:{DEST_PORT}")
    while running:
        try:
            msg = input("> ")
            udpSocket.sendto(msg.encode(), (DEST_IP, DEST_PORT))
        except Exception as e:
            print(f"[Send error] {e}")
            break

try:
    # Start listener in background
    listen_thread = threading.Thread(target=listen, daemon=True)
    listen_thread.start()

    # Run sender in main thread
    send()

except KeyboardInterrupt:
    print("\n[Exiting]")

finally:
    running = False
    listen_thread.join()
    udpSocket.close()
    print("[Socket closed]")
    sys.exit(0)
