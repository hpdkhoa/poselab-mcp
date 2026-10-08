"""A small MCP client over stdio: starts the Pose Lab server and calls its tools."""
import json
import subprocess
import sys


class PoseLab:
    def __init__(self):
        self.p = subprocess.Popen([sys.executable, "-m", "poselab_mcp"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.n = 0
        self._rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "poselab-bench", "version": "1"}})
        self._send({"jsonrpc": "2.0", "method": "notifications/initialized"})
        self.tools = self._rpc("tools/list")["tools"]

    def _send(self, msg):
        self.p.stdin.write((json.dumps(msg) + "\n").encode())
        self.p.stdin.flush()

    def _rpc(self, method, params=None):
        self.n += 1
        self._send({"jsonrpc": "2.0", "id": self.n, "method": method, "params": params or {}})
        while True:
            line = self.p.stdout.readline()
            if not line:
                raise RuntimeError("the Pose Lab server stopped")
            reply = json.loads(line)
            if reply.get("id") == self.n:
                if "error" in reply:
                    raise RuntimeError(reply["error"])
                return reply["result"]

    def call(self, _tool, **args):
        """The tool's raw MCP result: {content: [...], isError}."""
        return self._rpc("tools/call", {"name": _tool, "arguments": args})

    def data(self, _tool, **args):
        """The tool's result as data (for setup and grading); raises on a tool error."""
        r = self.call(_tool, **args)
        texts = [c["text"] for c in r["content"] if c["type"] == "text"]
        if r.get("isError"):
            raise RuntimeError("%s failed: %s" % (_tool, texts[0][:400] if texts else ""))
        d = r.get("structuredContent")
        if isinstance(d, dict) and set(d) == {"result"}:
            d = d["result"]
        if d is None and texts:
            try:
                d = json.loads(texts[0])
            except ValueError:
                d = texts[0]
        return d

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=60)
        except Exception:
            self.p.kill()
