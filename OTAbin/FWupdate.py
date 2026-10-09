#!/usr/bin/env python3

import os
import time
import webbrowser
import zipfile
import requests
import sys
import threading
import json
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

    url = f"http://{target_ip}:8080/update?target=all"

    print(f"[UPDATE] Triggering gateway update: {url}")

    try:

        headers = {
            "User-Agent": "PTZrobot-OTA"
        }

        requests.get(
            url,
            timeout=(3, 5),
            headers=headers
        )

        print("[UPDATE] OTA request sent.\n")

    except Exception:
        print(
            "[UPDATE] OTA request sent. "
            "Gateway may already be busy updating.\n"
        )
def get_expected_gateway_version():

    try:

        with open(
            os.path.join(SCRIPT_DIR, "manifest.json"),
            "r",
            encoding="utf-8"
        ) as f:

            manifest = json.load(f)

        for device in manifest.get("devices", []):

            if device.get("deviceType") == "gateway":
                return device.get("fwVersion")

    except Exception as e:

        print(
            f"[UPDATE] Failed to read manifest: {e}"
        )

    return None

def verify_versions(target_ip, timeout=180):

    expected_gateway = get_expected_gateway_version()

    if not expected_gateway:

        print(
            "[UPDATE] Cannot determine expected "
            "gateway version."
        )

        return False

    expected_gateway = str(expected_gateway).strip()

    print()
    print(
        f"[UPDATE] Expected gateway version: "
        f"{expected_gateway}"
    )

    print(
        "[UPDATE] Waiting for gateway "
        "to report new firmware..."
    )

    start = time.time()
    last_report = None

    while (time.time() - start) < timeout:

        try:

            r = requests.get(
                f"http://{target_ip}:8080/version",
                timeout=3
            )

            if r.status_code == 200:

                info = r.json()

                gateway_fw = str(
                    info.get("gatewayFw", "")
                ).strip()

                gateway_hw = str(
                    info.get("gatewayHw", "?")
                ).strip()

                driver_fw = str(
                    info.get("driverFw", "?")
                ).strip()

                driver_hw = str(
                    info.get("driverHw", "?")
                ).strip()

                driver_name = str(
                    info.get("device", "Driver")
                ).strip()

                if gateway_fw != last_report:

                    print(
                        f"[UPDATE] Gateway reports "
                        f"{gateway_fw} "
                        f"(expecting {expected_gateway})"
                    )

                    print(
                        f"[DEBUG] JSON: {info}"
                    )

                    last_report = gateway_fw

                if gateway_fw == expected_gateway:

                    print()
                    print("================================")
                    print("Firmware Verification")
                    print("================================")

                    print(
                        f"Gateway : "
                        f"{gateway_hw} "
                        f"{gateway_fw}"
                    )

                    print(
                        f"{driver_name} : "
                        f"{driver_hw} "
                        f"{driver_fw}"
                    )

                    print("================================")
                    print()

                    return True

        except requests.exceptions.RequestException:
            pass

        time.sleep(3)

    print()
    print(
        f"[UPDATE] Timeout waiting for "
        f"gateway version {expected_gateway}"
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