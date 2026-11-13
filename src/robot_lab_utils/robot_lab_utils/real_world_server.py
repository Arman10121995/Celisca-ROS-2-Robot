"""Local web server for the globe area picker (ROADMAP R6.8).

Runs as its own process (the GUI starts it; no Tk threads), bound to
127.0.0.1 only. It serves ``web/globe.html`` and runs one import at a time
as a child process (``python3 -m robot_lab_utils.real_world_import --spec``),
streaming its progress to the page and to a status file the GUI polls::

    python3 -m robot_lab_utils.real_world_server --port 0   # prints {"url": ...}
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading

PAGE = Path(__file__).resolve().parent / 'web' / 'globe.html'


class ImportJob:
    def __init__(self, status_path):
        self.lock = threading.Lock()
        self.state, self.error, self.result, self.pending, self.process = 'idle', '', None, [], None
        self.status_path = Path(status_path) if status_path else None

    def start(self, request):
        with self.lock:
            if self.state == 'running':
                raise ValueError('an import is already running')
            if not isinstance(request, dict) or not isinstance(request.get('id'), str):
                raise ValueError('a world name is required')
            with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as handle:
                json.dump(request, handle)
                spec = handle.name
            self.state, self.error, self.result, self.pending = 'running', '', None, []
            self.process = subprocess.Popen([sys.executable, '-m', 'robot_lab_utils.real_world_import', '--spec', spec],
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            threading.Thread(target=self._follow, args=(spec, request['id']), daemon=True).start()
            self._save(request['id'])

    def _follow(self, spec, name):
        for line in self.process.stdout:
            try:
                message = json.loads(line)
            except ValueError:
                continue                                  # library chatter; the JSON lines carry the state
            with self.lock:
                if 'progress' in message:
                    self.pending.append(message['progress'])
                elif 'manifest' in message:
                    self.result, self.state = message['manifest'], 'done'
                elif 'error' in message:
                    self.error, self.state = message['error'], 'failed'
        self.process.wait()
        Path(spec).unlink(missing_ok=True)
        with self.lock:
            if self.state == 'running':
                self.state, self.error = 'failed', 'importer exited with %s' % self.process.returncode
            self._save(name)

    def _save(self, name):
        if self.status_path:
            self.status_path.parent.mkdir(parents=True, exist_ok=True)
            self.status_path.write_text(json.dumps(dict(id=name, state=self.state, error=self.error,
                                                        result=self.result)))

    def poll(self):
        with self.lock:
            messages, self.pending = self.pending, []
            return dict(state=self.state, error=self.error, result=self.result, messages=messages)


def make_handler(job):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def _send(self, code, body, content_type='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._send(200, PAGE.read_bytes(), 'text/html; charset=utf-8')
            elif self.path == '/status':
                self._send(200, job.poll())
            else:
                self._send(404, dict(error='not found'))

        def do_POST(self):
            if self.path != '/import':
                self._send(404, dict(error='not found'))
                return
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length < 65536:
                    raise ValueError('bad request size')
                job.start(json.loads(self.rfile.read(length)))
                self._send(200, dict(ok=True))
            except (ValueError, TypeError) as exc:
                self._send(200, dict(ok=False, error=str(exc)))
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--status', type=Path, help='status JSON file the GUI polls')
    args = parser.parse_args(argv)
    job = ImportJob(args.status)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(job))
    print(json.dumps(dict(url='http://127.0.0.1:%d/' % server.server_address[1], pid=os.getpid())), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if job.process and job.process.poll() is None:
            job.process.terminate()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
