"""The desktop child advertises only its own listener and exits when its parent pipe closes."""

import json
import os
import socket
import subprocess
import sys
from pathlib import Path
from urllib.request import urlopen


def start_host(tmp_path, port=0):
    env = {**os.environ, "PYTHONPATH": str(Path("src").resolve())}
    return subprocess.Popen(
        [
            sys.executable,
            "-X",
            "utf8",
            "-m",
            "robostudio.desktop_host",
            "--port",
            str(port),
            "--output",
            str(tmp_path / "中文 data"),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        env=env,
    )


def test_host_is_ready_and_does_not_outlive_parent(tmp_path):
    process = start_host(tmp_path)
    try:
        ready = json.loads(process.stdout.readline())
        # Windows venv python.exe can be a launcher with a different child PID.
        assert isinstance(ready["pid"], int) and ready["pid"] > 0
        assert ready["url"].startswith("http://127.0.0.1:")
        with urlopen(ready["url"] + "/api/status", timeout=10) as response:
            assert json.load(response)["application"] == "robostudio"
        process.stdin.close()
        assert process.wait(timeout=10) == 0
        with socket.socket() as probe:
            assert probe.connect_ex(("127.0.0.1", ready["port"])) != 0
        assert process.stderr.read() == ""
    finally:
        if process.poll() is None:
            process.kill()
        process.wait(timeout=5)


def test_host_never_attaches_to_an_unowned_listener(tmp_path):
    with socket.socket() as occupied:
        occupied.bind(("127.0.0.1", 0))
        occupied.listen()
        process = start_host(tmp_path, occupied.getsockname()[1])
        out, err = process.communicate(timeout=10)
        assert process.returncode != 0
        assert not out
        assert "DESKTOP_START_FAILED" in err


def test_host_rejects_invalid_port_before_launch(tmp_path):
    process = start_host(tmp_path, -1)
    out, err = process.communicate(timeout=10)
    assert process.returncode != 0
    assert not out
    assert "port" in err
