"""Owned desktop backend: loopback listener, JSON readiness and stdin lifetime lease."""

import argparse
import json
import os
import sys
import threading
from pathlib import Path

from .backends.mujoco_backend import engine
from .core import RoboStudioError
from .workbench import make_server


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18765)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("port must be 0..65535")
    try:
        args.output.mkdir(parents=True, exist_ok=True)
        # Load native math libraries before the lifetime thread blocks on stdin.
        # Their Windows CRT initialization may otherwise wait on that input handle.
        engine()
        server = make_server(args.port, args.output)
    except (OSError, RoboStudioError) as error:
        print(f"DESKTOP_START_FAILED: {error}", file=sys.stderr, flush=True)
        return 1

    def watch_parent() -> None:
        # No executable commands travel over this channel. EOF is the ownership lease.
        while os.read(sys.stdin.fileno(), 1):
            pass
        server.shutdown()

    threading.Thread(target=watch_parent, name="desktop-parent", daemon=True).start()
    print(
        json.dumps(
            {
                "url": f"http://127.0.0.1:{server.server_port}",
                "port": server.server_port,
                "pid": os.getpid(),
            }
        ),
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.1)
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
