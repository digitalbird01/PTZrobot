#!/usr/bin/env python3
import os
import time
import webbrowser
import zipfile
import requests
import platform
from http.server import SimpleHTTPRequestHandler, HTTPServer

ZIP_URL = "https://github.com/digitalbird01/PTZrobot/archive/refs/heads/main.zip"
GATEWAY_URL = "http://10.0.30.81:8080/update?target=all"

# Cross-platform download folder
if platform.system() == "Windows":
    DOWNLOAD_DIR = os.path.join(os.path.expanduser("~"), "Downloads")
else:
    DOWNLOAD_DIR = os.path.join(os.path.expanduser("~"), "Downloads")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


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

    for file in z.namelist():
        if file.startswith(root) and not file.endswith("/"):
            filename = file.split("/")[-1]
            local_path = os.path.join(SCRIPT_DIR, filename)

            with open(local_path, "wb") as out:
                out.write(z.read(file))

            print(f"[UPDATE] Saved {filename}")

    print("[UPDATE] Extraction complete.\n")


def trigger_gateway_update():
    print(f"[UPDATE] Triggering gateway update: {GATEWAY_URL}")

    try:
        r = requests.get(GATEWAY_URL, timeout=10)
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
    open_browser_for_zip()
    zip_path = wait_for_zip()
    extract_otabin(zip_path)
    trigger_gateway_update()

    PORT = 8000
    print(f"[UPDATE] Serving firmware on port {PORT}")
    HTTPServer(("", PORT), ProgressHandler).serve_forever()
