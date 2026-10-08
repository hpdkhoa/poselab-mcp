"""Pose Lab's end-to-end test: starts the server as an MCP client would (stdio), loads the sample rig, measures the
port on the rifle's right side from the eye, asks the solver whether turning alone can show it, then whether turning
and moving can, and saves a contact sheet.

    python examples/selftest.py            (needs Blender; see the README)
"""
import json
import subprocess
import sys
import time


class Client:
    def __init__(self):
        self.p = subprocess.Popen([sys.executable, "-m", "poselab_mcp"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.n = 0

    def send(self, msg):
        self.p.stdin.write((json.dumps(msg) + "\n").encode())
        self.p.stdin.flush()

    def rpc(self, method, params=None):
        self.n += 1
        self.send({"jsonrpc": "2.0", "id": self.n, "method": method, "params": params or {}})
        while True:
            reply = json.loads(self.p.stdout.readline())
            if reply.get("id") == self.n:
                return reply

    def tool(self, _tool, **args):
        t = time.time()
        r = self.rpc("tools/call", {"name": _tool, "arguments": args})["result"]
        texts = [c["text"] for c in r["content"] if c["type"] == "text"]
        images = [c for c in r["content"] if c["type"] == "image"]
        print("--- %s (%.1f s)%s%s" % (_tool, time.time() - t, " ERROR" if r.get("isError") else "", " + image" if images else ""))
        print((texts[0] if texts else "")[:1500])
        if r.get("isError"):
            raise SystemExit(1)
        data = r.get("structuredContent")
        if isinstance(data, dict) and set(data) == {"result"}:
            data = data["result"]
        if data is None and texts:
            try:
                data = json.loads(texts[0])
            except ValueError:
                data = texts[0]
        return data


def main():
    c = Client()
    init = c.rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "selftest", "version": "0"}})
    print("server:", init["result"]["serverInfo"])
    c.send({"jsonrpc": "2.0", "method": "notifications/initialized"})
    print("tools:", [t["name"] for t in c.rpc("tools/list")["result"]["tools"]])
    c.tool("list_rigs")
    c.tool("arm_ranges")                       # the research behind anatomy's limits; needs no rig
    c.tool("load_rig", rig="sample")
    c.tool("clearance", parts="both")          # the baseline: what the hands touch on the idle grip
    c.tool("anatomy")                          # the idle grip against the human arm's limits
    c.tool("grip")                             # each hand on the rifle with its palm, the back of the hand clear
    c.tool("move_part", part="carrier", cm=4.0)
    c.tool("faces_eye", point="port")
    c.tool("visible", point="port")
    c.tool("screen", point="port")
    c.tool("snapshot", action="save", name="start")
    grip = ["index*", "middle*", "ring*", "pinky*", "thumb*", "palm*"]   # the hands keep their grips
    goals = [{"type": "faces_eye", "point": "port", "min": 0.5},
             {"type": "clearance", "parts": "both", "ignore": grip, "max_cm": 0.0},
             {"type": "visible", "point": "port", "min": 0.6},
             {"type": "on_screen", "point": "port"}]
    # turning alone: the eye looks along the barrel, so no turn of the rifle about the shoulder shows the port
    c.tool("solve", dofs={"roll": [-90, 90], "swing": [-40, 40]}, goals=goals, samples=60)
    c.tool("snapshot", action="load", name="start")
    # turning and moving: lowered and brought across in front of the face
    c.tool("solve", dofs={"roll": [-30, 90], "swing": [-40, 40], "right": [-25, 10], "up": [-30, 5]}, goals=goals, samples=200, maximize=0)
    c.tool("render", views=["eye", "right", "top", "front"])
    c.p.stdin.close()
    c.p.wait(timeout=60)


if __name__ == "__main__":
    main()
