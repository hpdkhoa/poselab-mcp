# Pose Lab's Blender side: headless Blender keeps one rig loaded and answers JSON commands on a local socket, one line
# in, one line out: {"token": ..., "cmd": "<Lab method>", "args": {...}} -> {"ok": true, "result": ...}.
#   blender -b --factory-startup -P worker.py -- <port> <rigs file or ""> <output folder>
# The session token comes in the POSELAB_TOKEN environment variable; a request without it is refused. Only the Lab's
# public commands run. The server (poselab_mcp.worker_client) starts it; {"cmd": "quit"} stops it.
import hmac
import json
import os
import socket
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lab  # noqa: E402

args = sys.argv[sys.argv.index("--") + 1:]
port, rigs_file, out_dir = int(args[0]), args[1] or None, args[2]
token = os.environ.get("POSELAB_TOKEN", "")
if not token:
    sys.exit("POSELAB_TOKEN is not set")
L = lab.Lab(rigs_file, out_dir)
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.bind(("127.0.0.1", port))
srv.listen(1)
print("POSELAB WORKER: listening on 127.0.0.1:%d" % port, flush=True)
running = True
while running:
    conn, _ = srv.accept()
    f = conn.makefile("rwb")
    for line in f:
        try:
            req = json.loads(line)
        except ValueError:
            continue
        if not hmac.compare_digest(str(req.get("token", "")), token):
            f.write(b'{"ok": false, "error": "bad token"}\n')
            f.flush()
            continue
        cmd = req.get("cmd", "")
        if cmd == "quit":
            f.write(b'{"ok": true, "result": "bye"}\n')
            f.flush()
            running = False
            break
        try:
            if cmd not in lab.PUBLIC:
                raise ValueError("no command %r" % cmd)
            out = {"ok": True, "result": getattr(L, cmd)(**req.get("args", {}))}
        except Exception as e:
            out = {"ok": False, "error": "%s: %s" % (type(e).__name__, e), "trace": traceback.format_exc()[-1500:]}
        f.write((json.dumps(out) + "\n").encode())
        f.flush()
    conn.close()
srv.close()
