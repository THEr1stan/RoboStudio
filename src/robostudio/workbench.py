"""Loopback-only workbench. Explicit asset allowlist; no remote execution APIs."""

import json
import socket
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer
from importlib.resources import files
from pathlib import Path
from typing import cast
from urllib.parse import urlsplit
from urllib.request import urlopen

from . import __version__
from .backends.mujoco_backend import engine
from .core import ErrorCode, RoboStudioError
from .demo import run_demo
from .registry.manifests import record, sequence
from .serialization import parse_json

ASSETS = {
    "/favicon.svg": ("ui/shared/favicon.svg", "image/svg+xml"),
    "/": ("ui/studio/studio.html", "text/html"),
    "/index.html": ("ui/studio/studio.html", "text/html"),
    "/workcell": ("ui/legacy/workcell/cell.html", "text/html"),
    "/studio.js": ("ui/studio/studio.js", "text/javascript"),
    "/studio.css": ("ui/studio/studio.css", "text/css"),
    "/studio-renderer.js": ("ui/studio/studio-renderer.js", "text/javascript"),
    "/studio-lab.js": ("ui/studio/studio-lab.js", "text/javascript"),
    "/studio-scenes.js": ("ui/studio/studio-scenes.js", "text/javascript"),
    "/advanced": ("ui/legacy/advanced/index.html", "text/html"),
    "/cell.js": ("ui/legacy/workcell/cell.js", "text/javascript"),
    "/cell.css": ("ui/legacy/workcell/cell.css", "text/css"),
    "/styles.css": ("ui/legacy/advanced/styles.css", "text/css"),
    "/app.js": ("ui/legacy/advanced/app.js", "text/javascript"),
    "/kinematics.js": ("ui/legacy/advanced/kinematics.js", "text/javascript"),
    "/renderer.js": ("ui/legacy/advanced/renderer.js", "text/javascript"),
}


def make_server(port: int = 8765, output_dir: Path = Path("artifacts")) -> HTTPServer:
    from .simulation.api import StudioAPI

    studio = StudioAPI(output_dir)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: object) -> None:
            pass

        def _send(self, status: int, body: bytes, content_type: str = "application/json") -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type + "; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; "
                "style-src 'self'; img-src 'self' data:; connect-src 'self'; "
                "frame-ancestors 'none'; base-uri 'none'; form-action 'none'",
            )
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass  # A tab can close while a completed physics response is being sent.

        def _json(self, status: int, value: object) -> None:
            self._send(
                status, json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
            )

        def _discard_small_body(self) -> None:
            # On Windows, closing with an unread request can reset the response socket.
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if 0 < size <= 16384:
                    self.connection.settimeout(3)
                    self.rfile.read(size)
            except (ValueError, OSError):
                pass

        def _local(self) -> bool:
            port = cast(HTTPServer, self.server).server_port
            host = self.headers.get("Host", "")
            if host not in (f"127.0.0.1:{port}", f"localhost:{port}"):
                if self.command == "POST":
                    self._discard_small_body()
                self._json(403, {"error": {"code": "LOCAL_ONLY", "message": "local host required"}})
                return False
            return True

        def do_GET(self) -> None:
            if not self._local():
                return
            path = urlsplit(self.path).path
            if path == "/api/status":
                try:
                    physics_version = str(engine().__version__)
                    physics_error = None
                except RoboStudioError as error:
                    physics_version, physics_error = None, error.message
                self._json(
                    200,
                    {
                        "application": "robostudio",
                        "version": __version__,
                        "backend": "mujoco" if physics_version else "fake-v1",
                        "preview": "geometric-only",
                        "physics_available": physics_version is not None,
                        "physics_version": physics_version,
                        "physics_error": physics_error,
                        "ik_available": physics_version is not None,
                        "jacobian_available": physics_version is not None,
                    },
                )
            elif path.startswith("/api/studio/"):
                try:
                    self._json(200, studio.get(path.removeprefix("/api/studio/")))
                except RoboStudioError as error:
                    self._json(400, {"error": error.as_dict()})
            elif path == "/api/cell/catalog":
                from .workcell import cell_catalog

                self._json(200, cell_catalog())
            elif path == "/api/cell/records":
                from .workcell import records

                self._json(200, records(output_dir))
            elif path.startswith("/api/cell/records/"):
                from .workcell import load_cell_record

                try:
                    self._json(200, load_cell_record(output_dir, path.rsplit("/", 1)[-1]))
                except RoboStudioError as error:
                    self._json(400, {"error": error.as_dict()})
            elif path == "/api/catalog":
                from .tracking import catalog

                self._json(200, catalog())
            elif path == "/api/runs":
                from .tracking import list_records

                self._json(200, list_records(output_dir))
            elif path.startswith("/api/runs/"):
                from .tracking import load_record

                try:
                    self._json(
                        200,
                        load_record(
                            output_dir,
                            path.removeprefix("/api/runs/"),
                            full=urlsplit(self.path).query == "full=1",
                        ),
                    )
                except RoboStudioError as error:
                    self._json(400, {"error": error.as_dict()})
            elif path in ASSETS:
                filename, mime = ASSETS[path]
                self._send(200, files("robostudio").joinpath(filename).read_bytes(), mime)
            else:
                self._json(404, {"error": {"code": "NOT_FOUND", "message": "unknown route"}})

        def do_POST(self) -> None:
            if not self._local():
                return
            origin = self.headers.get("Origin")
            if origin != "http://" + self.headers.get("Host", ""):
                self._discard_small_body()
                self._json(
                    403, {"error": {"code": "LOCAL_ONLY", "message": "same origin required"}}
                )
                return
            if self.path not in (
                "/api/demo",
                "/api/arm/run",
                "/api/arm/analyze",
                "/api/arm/ik",
                "/api/arm/track",
                "/api/cell/preview",
                "/api/cell/ik",
                "/api/cell/run",
                "/api/cell/compare",
                "/api/cell/replay",
                "/api/cell/spec",
                "/api/cell/candidate",
                "/api/studio/create",
                "/api/studio/create_world",
                "/api/studio/scene_validate",
                "/api/studio/reset",
                "/api/studio/step",
                "/api/studio/snapshot",
                "/api/studio/command",
                "/api/studio/save",
                "/api/studio/close",
                "/api/studio/experiment",
                "/api/studio/replay",
                "/api/studio/lab",
                "/api/studio/algorithm_records",
                "/api/studio/inspect",
            ):
                self._json(404, {"error": {"code": "NOT_FOUND"}})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                limit = 8192 if self.path in ("/api/cell/spec", "/api/cell/candidate") else 1024
                if self.path.startswith("/api/studio/"):
                    limit = 16384
                if not 0 < size <= limit or self.headers.get_content_type() != "application/json":
                    self._discard_small_body()
                    raise RoboStudioError(ErrorCode.INVALID_VALUE, "expected small JSON request")
                request_data = parse_json(self.rfile.read(size).decode("utf-8"))
                if self.path.startswith("/api/studio/"):
                    self._json(200, studio.post(self.path.rsplit("/", 1)[-1], request_data))
                    return
                if self.path.startswith("/api/cell/"):
                    from .workcell import handle_request

                    self._json(
                        200, handle_request(self.path.rsplit("/", 1)[-1], request_data, output_dir)
                    )
                    return
                if self.path == "/api/arm/ik":
                    from .tracking import solve_target

                    self._json(200, solve_target(request_data))
                    return
                if self.path == "/api/arm/track":
                    from .tracking import run_tracking

                    self._json(200, run_tracking(request_data, output_dir))
                    return
                if self.path == "/api/arm/analyze":
                    from .jacobian import analyze_pose

                    analysis = record(request_data, "analysis", {"q_rad", "qvel_rad_s"})
                    q = cast(tuple[float, ...], tuple(sequence(analysis["q_rad"], "q_rad")))
                    qvel = cast(
                        tuple[float, ...], tuple(sequence(analysis["qvel_rad_s"], "qvel_rad_s"))
                    )
                    self._json(200, analyze_pose(q, qvel))
                    return
                data = record(request_data, "demo", {"seed"})
                if self.path == "/api/arm/run":
                    from .arm_experiment import run_arm

                    arm_result = run_arm(seed=cast(int, data["seed"]), output_dir=output_dir)
                    self._json(
                        200 if arm_result["result"]["status"] == "SUCCEEDED" else 503, arm_result
                    )
                    return
                result = run_demo(seed=cast(int, data["seed"]), output_dir=output_dir)
                self._json(200 if result.error_code is None else 500, result.as_dict())
            except (ValueError, UnicodeError) as error:
                if not isinstance(error, RoboStudioError):
                    error = RoboStudioError(ErrorCode.INVALID_VALUE, str(error))
                self._json(400, {"error": error.as_dict()})

    server = HTTPServer(("127.0.0.1", port), Handler, bind_and_activate=False)
    server.allow_reuse_address = False
    try:
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        server.server_bind()
        server.server_activate()
    except OSError:
        server.server_close()
        raise
    return server


def serve(port: int, output_dir: Path, open_browser: bool) -> None:
    try:
        server = make_server(port, output_dir)
    except OSError as error:
        # Reopening the launcher should not create a competing Windows listener.
        try:
            with urlopen(f"http://127.0.0.1:{port}/api/status", timeout=1) as response:
                existing = json.loads(response.read(8192))
            if isinstance(existing, dict) and existing.get("application") == "robostudio":
                url = f"http://127.0.0.1:{port}/"
                print(f"RoboStudio already running ({existing.get('version')}): {url}", flush=True)
                if open_browser:
                    webbrowser.open(url)
                return
        except (OSError, ValueError):
            pass
        raise RoboStudioError(ErrorCode.IO_ERROR, str(error), "workbench port") from error
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"RoboStudio local workbench: {url}", flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
