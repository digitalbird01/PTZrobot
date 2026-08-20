#!/usr/bin/env python3
from http.server import SimpleHTTPRequestHandler, HTTPServer
import os

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

PORT = 8000
print(f"Serving on port {PORT}")
HTTPServer(("", PORT), ProgressHandler).serve_forever()
