"""The motion layer's end-to-end test, as an MCP client: on the sample rig it records a clip with two common faults,
scans it, fixes it and saves it.

The faults: the rifle rolls 75 degrees and back, keyed only at the ends, so between keys the hands blend by rotation
and drift off the rifle; and one frame jumps 15 cm (a pop).

    python examples/motion_test.py            (needs Blender; see the README)
"""
from selftest import Client

GRIP = ["index*", "middle*", "ring*", "pinky*", "thumb*", "palm*"]   # a hand round its grip touches it
CHECKS = [
    {"type": "hold", "side": "l", "max_cm": 0.5},
    {"type": "hold", "side": "r", "max_cm": 0.5},
    {"type": "pop", "bones": ["hand_l", "hand_r"], "max_cm_per_s": 250},
    {"type": "clearance", "parts": "both", "ignore": GRIP, "max_cm": 0.0},
]


def main():
    c = Client()
    c.rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "motion_test", "version": "0"}})
    c.send({"jsonrpc": "2.0", "method": "notifications/initialized"})
    c.tool("load_rig", rig="sample")
    c.tool("record_clip", action="start", clip="roll")
    c.tool("record_clip", action="key", seconds=0.0)
    c.tool("move_gun", roll=75, keep_hands=["l", "r"])
    c.tool("record_clip", action="key", seconds=0.6)
    c.tool("record_clip", action="key", seconds=0.8)
    c.tool("move_gun", up=15, keep_hands=["l", "r"])
    c.tool("record_clip", action="key", seconds=0.8333)
    c.tool("move_gun", up=-15, keep_hands=["l", "r"])
    c.tool("record_clip", action="key", seconds=0.8667)
    c.tool("pose_idle")
    c.tool("record_clip", action="key", seconds=1.4)
    c.tool("record_clip", action="stop")
    before = c.tool("scan_clip", clip="roll", checks=CHECKS)
    fixed = c.tool("fix_clip", clip="roll", checks=CHECKS)
    c.tool("save_clip", clip="roll_fixed", formats=["pose.json", "fbx"])
    c.tool("pose_clip", clip="roll_fixed", seconds=0.6)
    c.tool("render", views=["eye", "right", "front", "top"])
    print("\nSUMMARY")
    for x in before["checks"]:
        print("  before: %-9s passed %-5s worst %s at %s s, failing %s" % (x["check"]["type"], x["passed"], x["worst"], x["worst_at_s"], x["failing_s"]))
    for x in fixed["after"]:
        print("  after:  %-9s passed %-5s worst %s" % (x["check"], x["passed"], x["worst"]))
    print("  changed frames:", fixed["changed_frames"], " all passed after:", fixed["all_passed_after"])
    print("  out of reach:", fixed["out_of_reach"])
    c.p.stdin.close()
    c.p.wait(timeout=60)


if __name__ == "__main__":
    main()
