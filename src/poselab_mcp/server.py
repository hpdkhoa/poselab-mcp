"""Pose Lab: an MCP server of measured spatial answers for posing first-person arms and a rifle in Blender.

A model gets numbers instead of pictures to guess from: positions in named frames, how deep anything clips, how squarely
a surface faces the eye, what the eye can see, and a solver that searches rifle moves against goals and says which
goals no move can meet. One headless Blender worker keeps the rig loaded between calls.
"""
import atexit
from typing import Annotated, Literal

from mcp.server.mcpserver import Image, MCPServer
from pydantic import Field

from . import __version__
from .sheet import sheet
from .worker_client import Worker

mcp = MCPServer("poselab", title="Pose Lab", version=__version__, instructions=(
    "Pose Lab measures a first-person rig (arms holding a rifle) in Blender. Call list_rigs, then load_rig. Positions "
    "use named frames: gun (the gun bone, cm: +X the gun's left, +Y along the barrel, +Z up), arms (the arms' space, "
    "cm) and view (from the eye, cm: +X right, +Y forward, +Z up). Turns: roll + turns the gun's right side up, swing + "
    "takes the muzzle left, pitch + the muzzle up. Measure before you pose: clearance, faces_eye, visible, screen; use "
    "solve to search moves against goals; a goal never met in any sample is a fact of the geometry. A hand round its "
    "grip touches the rifle on the idle pose already: check clearance at pose_idle for that baseline. For motion: "
    "record_clip or load_clip a clip, scan_clip it against checks, fix_clip what fails, then save_clip the result."))
worker = Worker()
atexit.register(worker.stop)

Point = Annotated[str | list[float], Field(description="a name (a bone such as hand_l, a fingertip such as index_tip_l, a rig point such as port) or [x, y, z] in the frame")]
Frame = Annotated[Literal["gun", "arms", "view"], Field(description="gun: the gun bone, cm (+X gun's left, +Y barrel, +Z up); arms: the arms' space, cm; view: from the eye, cm (+X right, +Y forward, +Z up)")]
Side = Literal["l", "r"]
Hands = Annotated[list[Side], Field(description="hands that keep their hold on the rifle as it moves (arm IK)")]
Parts = Literal["both", "left", "right", "l", "r"]


@mcp.tool()
def list_rigs() -> dict:
    """The rigs to load: the built-in sample and those in the rigs file (POSELAB_RIGS)."""
    return worker.call("list_rigs")


@mcp.tool()
def load_rig(rig: str) -> dict:
    """Loads a rig: the arms on their idle pose, the rifle on the gun bone, its clips. Returns the frames, sign rules,
    named points, clips and moving parts. Call first."""
    return worker.call("load_rig", rig=rig)


@mcp.tool()
def describe() -> dict:
    """What is loaded and the conventions every tool uses."""
    return worker.call("describe")


@mcp.tool()
def pose_idle() -> dict:
    """The arms back on the idle pose, every rifle part home."""
    return worker.call("pose_idle")


@mcp.tool()
def pose_clip(clip: str, seconds: float) -> dict:
    """The arms as a clip has them at a time in seconds. The rifle follows the gun bone; its parts stay where move_part
    left them."""
    return worker.call("pose_clip", clip=clip, seconds=seconds)


@mcp.tool()
def move_part(part: str, cm: float) -> dict:
    """Draws a moving part of the rifle (such as carrier) back along the barrel, cm from home."""
    return worker.call("move_part", part=part, cm=cm)


@mcp.tool()
def move_gun(roll: float = 0.0, swing: float = 0.0, pitch: float = 0.0, right: float = 0.0, forward: float = 0.0,
             up: float = 0.0, pivot: Annotated[str | list[float], Field(description="a rig point (stock, bore) or [x, y, z] in the gun frame")] = "stock",
             keep_hands: Hands = ["l", "r"]) -> dict:
    """Moves the rifle from where it stands: roll + turns its right side up (about the bore); swing + takes the muzzle
    left, pitch + the muzzle up (about the pivot); right, forward, up move it in the view, cm. The hands in keep_hands
    keep their hold. Returns how far each hand ended from its hold and the barrel's angle off the view line."""
    return worker.call("move_gun", roll=roll, swing=swing, pitch=pitch, right=right, forward=forward, up=up, pivot=pivot, keep_hands=keep_hands)


@mcp.tool()
def reach(side: Side, target: Point, frame: Frame = "gun",
          pole: Annotated[list[float] | None, Field(description="where the elbow points, arms frame cm")] = None) -> dict:
    """Puts a wrist on a point by arm IK; the hand keeps its turn. Returns how far the wrist ended from the point.
    Trying poles while the wrist stays put moves only the elbow: a way to clear a forearm."""
    return worker.call("reach", side=side, target=target, frame=frame, pole=pole)


@mcp.tool()
def snapshot(action: Literal["save", "load"], name: str) -> dict:
    """Saves or loads a named pose (the arms and the rifle's parts)."""
    return worker.call("snapshot", action=action, name=name)


@mcp.tool()
def where(names: list[str], frame: Frame = "gun") -> dict:
    """Where bones, fingertips (<finger>_tip_<l|r>) and rig points are, in a frame."""
    return worker.call("where", names=names, frame=frame)


@mcp.tool()
def distance(a: Point, b: Point, frame: Frame = "gun") -> dict:
    """The distance between two points, cm, and b minus a in the frame."""
    return worker.call("distance", a=a, b=b, frame=frame)


@mcp.tool()
def clearance(parts: Parts = "both",
              ignore: Annotated[list[str], Field(description="segment names, prefixes or wildcards to leave out (thumb, index3_r, *_l)")] = [],
              frame: Frame = "gun", top: int = 5) -> dict:
    """How deep the rifle sits inside the arms (capsules: forearm 3 cm, palm 1.8 cm, finger segments 0.75 cm).
    worst_cm 0 means nothing touches. The deepest contacts by segment, with the rifle point in the frame."""
    return worker.call("clearance", parts=parts, ignore=ignore, frame=frame, top=top)


@mcp.tool()
def faces_eye(point: Point = "port", normal: Annotated[list[float] | None, Field(description="the surface's normal, gun frame; a rig normal by the point's name if left out")] = None) -> dict:
    """How squarely a surface faces the eye: facing 1 square on, 0 edge on, below 0 turned away."""
    return worker.call("faces_eye", point=point, normal=normal)


@mcp.tool()
def visible(point: Point = "port", radius_cm: float = 0.6) -> dict:
    """The share of a small disc round a point that the eye sees with nothing (arms, rifle) in the way."""
    return worker.call("visible", point=point, radius_cm=radius_cm)


@mcp.tool()
def screen(point: Point) -> dict:
    """Where a point falls on a 90 degree, 16:9 screen from the eye: x and y from -1 to 1, and whether it is on screen."""
    return worker.call("screen", point=point)


@mcp.tool()
def solve(dofs: Annotated[dict[str, list[float]], Field(description="roll, swing, pitch, right, forward, up -> [min, max]")],
          goals: Annotated[list[dict], Field(description="each {type: faces_eye|visible (point, min), on_screen (point), clearance (parts, ignore, max_cm), barrel (max_deg), distance (a, b, max_cm)}")],
          keep_hands: Hands = ["l", "r"], pivot: str = "stock", samples: int = 120,
          maximize: Annotated[int | None, Field(description="a goal's index to push higher once all are met")] = None) -> dict:
    """Searches rifle moves from the current pose for one meeting every goal, and leaves the scene there. Reports each
    goal at the best move and how often it was met across samples: a goal never met is a fact of the geometry."""
    return worker.call("solve", dofs=dofs, goals=goals, keep_hands=keep_hands, pivot=pivot, samples=samples, maximize=maximize)


Checks = Annotated[list[dict], Field(description=(
    "each {type, ...}: clearance (parts, ignore, max_cm), faces_eye / visible (point, min), on_screen (point), "
    "barrel (max_deg), contact (a, b, max_cm), hold (side l|r, max_cm, ref_s: the hand's drift on the rifle from its "
    "grip at ref_s), pop (bones, 'gun' for the gun bone, max_cm_per_s). Any check takes during: [from_s, to_s]"))]


@mcp.tool()
def record_clip(action: Literal["start", "key", "stop"], clip: str | None = None, seconds: float = 0.0, fps: float = 30.0,
                ease: bool = True) -> dict:
    """Builds a clip from poses you set: start (a name), key (the current pose at a time in seconds), stop (bakes the
    frames, each bone eased from key to key). Between keys the bones blend by rotation, so hands can drift off the
    rifle; scan_clip's hold check finds that and fix_clip mends it."""
    return worker.call("record_clip", action=action, clip=clip, seconds=seconds, fps=fps, ease=ease)


@mcp.tool()
def load_clip(clip: str, path: str, bone_map: Annotated[dict[str, str] | None, Field(description="the file's bone names -> this rig's")] = None) -> dict:
    """Adds a clip from a file: .pose.json bone data, an FBX animation, or a BVH (for example a text-to-motion
    model's output). A relative path is taken from the rigs file's folder. Reports how well its skeleton fits."""
    return worker.call("load_clip", clip=clip, path=path, bone_map=bone_map)


@mcp.tool()
def scan_clip(clip: str, checks: Checks) -> dict:
    """Plays a clip frame by frame and runs each check. Reports for each check whether it passed, the worst value
    and when, and the time spans where it fails."""
    return worker.call("scan_clip", clip=clip, checks=checks)


@mcp.tool()
def fix_clip(clip: str, checks: Checks, out: str | None = None, max_swing_deg: float = 90.0, spread_frames: int = 4) -> dict:
    """Mends a clip against the checks and stores a new clip (out, default <clip>_fixed): pops are blended again from
    the good frames round them; a hold puts the hand back on its grip; a contact moves the wrist until the point
    touches its mark; clearance swings each elbow about its shoulder-wrist line (the wrist kept) by the least angle
    that clears, spread over neighbouring frames. Reports the scan before and after."""
    return worker.call("fix_clip", clip=clip, checks=checks, out=out, max_swing_deg=max_swing_deg, spread_frames=spread_frames)


@mcp.tool()
def save_clip(clip: str, formats: list[Literal["pose.json", "fbx"]] = ["pose.json"],
              root_name: Annotated[str | None, Field(description="the armature's name in the FBX (Unreal reads it as the root bone)")] = None) -> dict:
    """Writes a clip to the output folder's clips/: pose.json bone data, and fbx for a game engine."""
    return worker.call("save_clip", clip=clip, formats=formats, root_name=root_name)


@mcp.tool(structured_output=False)
def render(views: list[Literal["eye", "right", "left", "top", "front"]] = ["eye", "right", "left", "top"]) -> list:
    """Renders the pose as one labelled contact sheet: eye is the player's camera (90 degrees, 5 cm near plane); right,
    left, top and front look at the rifle from outside. Every tile is a Blender view, labelled so."""
    images = worker.call("render", views=views)["images"]
    path, png = sheet(images)
    return ["contact sheet: %s (every tile a Blender view)" % path, Image(data=png, format="png")]


def main():
    mcp.run()


if __name__ == "__main__":
    main()
