import socket
import json

# Configuration
PORT = 9000

# Setup UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("10.0.0.184", PORT))

print(f"Listening on UDP port {PORT}. Press Ctrl-C to exit.\n")

try:
    while True:
        data_bytes, addr = sock.recvfrom(4096)
        text = data_bytes.decode(errors='ignore').strip()

        for line in text.splitlines():
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue

            # format and print the telemetry values
            print(
                f"t={d.get('t', 0):>6} ms  | "
                f"ax={d.get('ax', 0.0):>6.3f}  ay={d.get('ay', 0.0):>6.3f}  az={d.get('az', 0.0):>6.3f}  | "
                f"gx={d.get('gx', 0.0):>6.2f}  gy={d.get('gy', 0.0):>6.2f}  gz={d.get('gz', 0.0):>6.2f}"
            )
except KeyboardInterrupt:
    print("\nInterrupted by user, exiting.")
finally:
    sock.close()
