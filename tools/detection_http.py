#!/usr/bin/env python3
"""Serve the latest YOLO detection JSON from the Go2."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


class Handler(BaseHTTPRequestHandler):
    json_path = Path("latest_detections.json")

    def do_GET(self):
        if urlsplit(self.path).path != "/latest_detections.json":
            self.send_error(404)
            return
        try:
            data = self.json_path.read_bytes()
        except OSError:
            self.send_error(503, "No detection JSON yet")
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *unused):
        pass


class Server(ThreadingHTTPServer):
    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--json", default="latest_detections.json")
    args = parser.parse_args()

    Handler.json_path = Path(args.json)
    server = Server((args.bind, args.port), Handler)
    print(
        "YOLO detections: http://{}:{}/latest_detections.json".format(
            args.bind, args.port
        ),
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
