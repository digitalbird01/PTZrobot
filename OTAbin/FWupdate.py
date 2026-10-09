#!/usr/bin/env python3

import os
import time
import webbrowser
import zipfile
import requests
import sys
import threading
from http.server import SimpleHTTPRequestHandler, HTTPServer

ZIP_URL = "https://github.com/digitalbird01/PTZrobot/archive/refs/heads/main.zip"

DOWNLOAD_DIR = os.path.join(os.path.expanduser("~"), "Downloads")
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def get_target_ip():
    if len(sys.argv) > 1:
        return sys.argv[1]

    return input("Enter device IP address: ").strip()


def open_browser_for_zip():
    print("[UPDATE] Opening browser for firmware download...")
    webbrowser.open(ZIP_URL)


def wait_for_zip():
    print("[UPDATE] Waiting for PTZrobot-main.zip to appear in Downloads...")

    while True:
        for f in os.listdir(DOWNLOAD_DIR):
            if f.startswith("PTZrobot-main") and f.endswith(".zip"):
                zip_path = os.path.join(DOWNLOAD_DIR, f)
                print(f"[UPDATE] Found ZIP: {zip_path}")
                return zip_path

        time.sleep(1)


def extract_otabin(zip_path):
    print(f"[UPDATE] Extracting {zip_path}...")
    z = zipfile.ZipFile(zip_path)
    # Remove old firmware + manifest
    for f in os.listdir(SCRIPT_DIR):
        if f.endswith(".bin") or f == "manifest.json":
            try:
                os.remove(os.path.join(SCRIPT_DIR, f))
                print(f"[UPDATE] Removed old file: {f}")
            except Exception:
                pass
    root = None

    for name in z.namelist():
        if name.endswith("OTAbin/"):
            root = name
            break

    if not root:
        print("[UPDATE] ERROR: OTAbin folder not found in ZIP")
        return

    for file in z.namelist():

        if file.startswith(root) and not file.endswith("/"):
            filename = file.split("/")[-1]

            local_path = os.path.join(
                SCRIPT_DIR,
                filename
            )
            with open(local_path, "wb") as out:
                out.write(z.read(file))

            print(
                f"[UPDATE] Saved {filename} "
                "to script directory"
            )

    print("[UPDATE] Extraction complete.\n")


def trigger_gateway_update(target_ip):

    url = (
        f"http://{target_ip}:8080/update?target=all"
    )

    print(
        f"[UPDATE] Triggering gateway update: "
        f"{url}"
    )
    try:
        headers = {
            "User-Agent": "PTZrobot-OTA"
        }
        r = requests.get(
            url,
            timeout=(5, 60),
            headers=headers
        )
        print(
            f"[UPDATE] Gateway response: "
            f"{r.status_code}"
        )
        print(
            "[UPDATE] Gateway update triggered "
            "successfully.\n"
        )
    except Exception as e:
        print(
            "[UPDATE] Gateway update request timed out "
            "(update may still be running)"
        )
        print(f"[UPDATE] Details: {e}")
def verify_versions(target_ip, timeout=120):
    print("\n[UPDATE] Waiting for gateway reboot...")
    print("[UPDATE] Checking firmware versions...\n")
    start = time.time()
    while time.time() - start < timeout:
        try:
            r = requests.get(
                f"http://{target_ip}:8080/version",
                timeout=5
            )
            if r.status_code == 200:
                info = r.json()
                print()
                print("================================")
                print("Firmware Verification")
                print("================================")
                print(
                    f"Gateway : "
                    f"{info.get('gatewayHw', '?')} "
                    f"{info.get('gatewayFw', '?')}"
                )
                print(
                    f"{info.get('device', 'Driver')} : "
                    f"{info.get('driverHw', '?')} "
                    f"{info.get('driverFw', '?')}"
                )
                print("================================")
                print()
                return True
        except Exception:
            pass
        time.sleep(5)
    print(
        "[UPDATE] Could not verify firmware versions. "
        "Gateway may still be rebooting."
    )
    return False
class ProgressHandler(SimpleHTTPRequestHandler):
    def copyfile(self, source, outputfile):
        total = os.fstat(
            source.fileno()
        ).st_size
        sent = 0
        chunk = 64 * 1024
        print(
            f"[SERVER] Sending file "
            f"({total} bytes)"
        )
        while True:
            data = source.read(chunk)
            if not data:
                break
            outputfile.write(data)
            sent += len(data)
            pct = (sent / total) * 100
            print(
                f"[SERVER] "
                f"{sent}/{total} bytes "
                f"({pct:.1f}%)"
            )
        print("[SERVER] Transfer complete")


if __name__ == "__main__":
    target_ip = get_target_ip()
    # Step 1
    open_browser_for_zip()
    zip_path = wait_for_zip()
    # Step 2
    extract_otabin(zip_path)
    try:
        os.remove(zip_path)
        print(
            f"[UPDATE] Removed ZIP file: "
            f"{zip_path}"
        )
    except Exception as e:
        print(
            f"[UPDATE] Could not remove ZIP file: "
            f"{e}"
        )
    # Step 3
    PORT = 8000
    print(
        f"[UPDATE] Serving firmware "
        f"on port {PORT}"
    )
    server = HTTPServer(
        ("", PORT),
        ProgressHandler
    )

    threading.Thread(
        target=server.serve_forever,
        daemon=True
    ).start()
    print(
        "[UPDATE] HTTP server is running "
        "and ready."
    )
    # Step 4
    trigger_gateway_update(target_ip)
# Step 5
verified = verify_versions(target_ip)

print()
print("================================")
print("      UPDATE COMPLETE")
print("================================")

if verified:
    print("Status : VERIFIED")
else:
    print("Status : NOT VERIFIED")

print("================================")
print()

print("[UPDATE] Stopping firmware server...")

server.shutdown()
server.server_close()

time.sleep(1)
sys.exit(0)