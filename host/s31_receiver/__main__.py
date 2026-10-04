import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import signal
import threading
import time
from urllib.parse import urlparse

from .receiver import Receiver


def main():
    p = argparse.ArgumentParser(description='ESP32-S31 Gigabit Ethernet IQ receiver / spectrum viewer')
    p.add_argument('--board', default='169.254.9.36')
    p.add_argument('--bind', default='169.254.9.35', help='local Ethernet IPv4 address')
    p.add_argument('--port', type=int, default=8765, help='localhost web port')
    p.add_argument('--rate', type=int, default=4_000_000)
    p.add_argument('--frequency', type=int, default=2412, help='center frequency, MHz')
    p.add_argument('--gain', type=int, default=40, help='manual PHY gain code')
    p.add_argument('--seconds', type=float, help='headless capture duration; exits nonzero on gaps/errors')
    p.add_argument('--report', type=Path)
    p.add_argument('--record', action='store_true')
    p.add_argument('--recording-dir', default='artifacts/recordings')
    p.add_argument('--no-start', action='store_true')
    a = p.parse_args()
    receiver = Receiver(a.board, a.bind, recording_dir=a.recording_dir)
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    try:
        if a.record:
            receiver.record(True)
        if not a.no_start:
            receiver.start(a.rate, a.frequency, a.gain)
        if a.seconds is not None:
            started = time.monotonic()
            while not stopped.wait(min(1, max(.01, a.seconds-(time.monotonic()-started)))):
                print(json.dumps(receiver.snapshot()), flush=True)
                if time.monotonic()-started >= a.seconds:
                    break
            receiver.stop()
            receiver.record(False)
            result = receiver.snapshot(include_spectrum=True)
            # Keep reports concise; retain measured DSP metrics, not 1024 bins.
            if result['spectrum']:
                result['spectrum'].pop('db', None)
            print(json.dumps(result, indent=2), flush=True)
            if a.report:
                a.report.parent.mkdir(parents=True, exist_ok=True)
                a.report.write_text(json.dumps(result, indent=2), encoding='utf-8')
            if not result['transport_continuous'] or result['error'] or result['device'].get('error') or result['device'].get('active'):
                raise SystemExit(2)
            return
        page = Path(__file__).with_name('index.html').read_bytes()
        actions = threading.Lock()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def respond(self, status, data, kind='application/json'):
                self.send_response(status)
                self.send_header('Content-Type', kind + '; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                if self.path == '/':
                    self.respond(200, page, 'text/html')
                elif self.path == '/api/state':
                    self.respond(200, json.dumps(receiver.snapshot(True)).encode())
                else:
                    self.respond(404, b'{"error":"not found"}')

            def do_POST(self):
                # Bind only to localhost and reject cross-origin browser control.
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    return self.respond(403, b'{"error":"origin mismatch"}')
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 4096:
                        raise ValueError('invalid request length')
                    data = json.loads(self.rfile.read(size))
                    with actions:
                        if self.path == '/api/start':
                            result = receiver.start(int(data['rate']), int(data['frequency']), int(data['gain']))
                        elif self.path == '/api/stop':
                            result = receiver.stop()
                        elif self.path == '/api/dsp':
                            receiver.configure_dsp(**data)
                            result = {'ok': True}
                        elif self.path == '/api/record':
                            result = receiver.record(bool(data['enabled']))
                        else:
                            return self.respond(404, b'{"error":"not found"}')
                    self.respond(200, json.dumps(result).encode())
                except (KeyError, ValueError, RuntimeError, TimeoutError, OSError) as e:
                    self.respond(400, json.dumps({'error': str(e)}).encode())

        server = ThreadingHTTPServer(('127.0.0.1', a.port), Handler)
        server.timeout = .25
        print(f'Spectrum viewer: http://localhost:{a.port}  |  board {a.board}', flush=True)
        try:
            while not stopped.is_set():
                server.handle_request()
        finally:
            server.server_close()
    finally:
        receiver.close()


if __name__ == '__main__':
    main()
