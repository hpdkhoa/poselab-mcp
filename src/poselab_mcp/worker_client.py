"""Starts the headless Blender worker on first use and talks to it over a local socket with a per-session token."""
import glob
import json
import os
import secrets
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORKER = HERE / "blender" / "worker.py"


def find_blender():
    """POSELAB_BLENDER, else blender on the PATH, else the usual install folders."""
    env = os.environ.get("POSELAB_BLENDER")
    if env:
        return env
    on_path = shutil.which("blender")
    if on_path:
        return on_path
    guesses = []
    if sys.platform == "win32":
        guesses += sorted(glob.glob(r"C:\Program Files\Blender Foundation\Blender *\blender.exe"), reverse=True)
    elif sys.platform == "darwin":
        guesses.append("/Applications/Blender.app/Contents/MacOS/Blender")
    else:
        guesses += ["/usr/bin/blender", "/snap/bin/blender"]
    for g in guesses:
        if os.path.exists(g):
            return g
    raise RuntimeError("Blender not found: set POSELAB_BLENDER to blender's executable")


def out_dir():
    """Where Pose Lab writes (renders, the worker's log): POSELAB_OUT, else ~/.poselab."""
    d = Path(os.environ.get("POSELAB_OUT") or Path.home() / ".poselab")
    d.mkdir(parents=True, exist_ok=True)
    return d


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Worker:
    def __init__(self):
        self.proc = None
        self.file = None
        self.token = secrets.token_hex(16)

    def ensure(self):
        if self.file and self.proc and self.proc.poll() is None:
            return
        port = _free_port()
        out = out_dir()
        log = open(out / "worker.log", "w")
        env = dict(os.environ, POSELAB_TOKEN=self.token)
        rigs = os.environ.get("POSELAB_RIGS", "")
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.proc = subprocess.Popen([find_blender(), "-b", "--factory-startup", "-P", str(WORKER), "--", str(port), rigs, str(out)],
                                     stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, env=env, creationflags=flags)
        start = time.time()
        while time.time() - start < 120:
            if self.proc.poll() is not None:
                raise RuntimeError("Blender stopped while starting; see %s" % (out / "worker.log"))
            try:
                s = socket.create_connection(("127.0.0.1", port), timeout=5)
                s.settimeout(900)
                self.file = s.makefile("rwb")
                return
            except OSError:
                time.sleep(0.5)
        raise RuntimeError("Blender did not answer within 120 s; see %s" % (out / "worker.log"))

    def call(self, cmd, **args):
        """Runs a Lab command; returns its result or raises with the worker's error."""
        self.ensure()
        try:
            self.file.write((json.dumps({"token": self.token, "cmd": cmd, "args": args}) + "\n").encode())
            self.file.flush()
            line = self.file.readline()
        except OSError:
            self.file = None
            raise
        if not line:
            self.file = None
            raise RuntimeError("the Blender worker closed; see %s" % (out_dir() / "worker.log"))
        reply = json.loads(line)
        if not reply.get("ok"):
            raise RuntimeError(reply.get("error", "worker error"))
        return reply["result"]

    def stop(self):
        if self.file:
            try:
                self.file.write((json.dumps({"token": self.token, "cmd": "quit"}) + "\n").encode())
                self.file.flush()
            except OSError:
                pass
        if self.proc:
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
