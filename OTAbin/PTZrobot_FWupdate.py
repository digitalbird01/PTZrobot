#!/usr/bin/env python3
import os
import time
import webbrowser
import zipfile
import requests
import platform
import sys
from http.server import SimpleHTTPRequestHandler, HTTPServer

ZIP_URL = "https://github.com/digitalbird01/PTZrobot/archive/refs/heads/main.zip"

# Cross-platform download folder
DOWNLOAD_DIR = os.path.join(os.path.expanduser("~"), "Downloads")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def get_target_ip():
    # If IP is passed as argument: python3 script.py 10.0.30.55
    if len(sys.argv) > 1:
        return sys.argv[1]

    # Otherwise ask interactively
    ip = input("Enter device IP address: ").strip()
    return ip


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

    root = None
    for name in z.namelist():
        if name.endswith("OTAbin/"):
            root = name
            break

    if not root:
        print("[UPDATE] ERROR: OTAbin folder not found in ZIP")
        return

    # Extract directly into script directory
    for file in z.namelist():
        if file.startswith(root) and not file.endswith("/"):
            filename = file.split("/")[-1]
            local_path = os.path.join(SCRIPT_DIR, filename)

            with open(local_path, "wb") as out:
                out.write(z.read(file))

            print(f"[UPDATE] Saved {filename} to script directory")

    print("[UPDATE] Extraction complete.\n")


def trigger_gateway_update(target_ip):
    url = f"http://{target_ip}:8080/update?target=all"
    print(f"[UPDATE] Triggering gateway update: {url}")

    try:
        headers = {"User-Agent": "PTZrobot-OTA"}
        r = requests.get(url, timeout=10, headers=headers)
        print(f"[UPDATE] Gateway response: {r.status_code}")
        print("[UPDATE] Gateway update triggered successfully.\n")
    except Exception as e:
        print("[UPDATE] ERROR triggering gateway update:", e)


class ProgressHandler(SimpleHTTPRequestHandler):
    def copyfile(self, source, outputfile):
        total = os.fstat(source.fileno()).st_size
        sent = 0
        chunk = 64 * 1024

        print(f"[SERVER] Sending file ({total} bytes)")

        while True:
            data = source.read(chunk)
            if not data:
                break
            outputfile.write(data)
            sent += len(data)
            pct = (sent / total) * 100
            print(f"[SERVER] {sent}/{total} bytes ({pct:.1f}%)")

        print("[SERVER] Transfer complete")


if __name__ == "__main__":
    target_ip = get_target_ip()

    open_browser_for_zip()
    zip_path = wait_for_zip()
    extract_otabin(zip_path)
    trigger_gateway_update(target_ip)

    PORT = 8000
    print(f"[UPDATE] Serving firmware on port {PORT}")
    HTTPServer(("", PORT), ProgressHandler).serve_forever()


