# Pose Lab

<!-- mcp-name: io.github.hpdkhoa/poselab-mcp -->

**Measured spatial answers for posing first-person arms and a rifle in Blender, over MCP.**

Models are weak at judging 3D space from pictures: which side of a rifle faces the eye, whether a finger sits inside
the receiver, whether any turn of the gun can ever show its ejection port. Pose Lab gives a model numbers instead:
positions in named frames, how deep anything clips, how squarely a surface faces the eye, what the eye can see, and a
solver that searches rifle moves against goals and reports which goals no move can meet.

It came out of hand-making chamber checks for a first-person shooter. One question ("can turning the rifle show the
port to the eye?") took several full Blender runs by hand. With Pose Lab it is one `solve` call: turning alone meets the
goal in 0 of 60 samples, which is a fact of the geometry; turning and moving the rifle meets every goal in about 20 s.

## What it gives a model

| Tool | Answers |
|---|---|
| `list_rigs`, `load_rig`, `describe` | the rigs, the frames, the sign rules, named points, clips, moving parts |
| `pose_idle`, `pose_clip`, `move_part`, `snapshot` | put the rig in a pose: a clip at a time, the carrier drawn back, saved poses |
| `move_gun` | roll, swing, pitch and move the rifle; the hands keep their hold by arm IK |
| `reach` | a wrist onto a point by arm IK (try elbow poles to clear a forearm) |
| `where`, `distance` | positions in a named frame |
| `clearance` | how deep the rifle sits inside a forearm, palm or finger, and where |
| `faces_eye`, `visible`, `screen` | how squarely a surface faces the eye, how much of it the eye sees, where it falls on screen |
| `solve` | searches rifle moves against goals; reports each goal and how often any sample met it |
| `render` | a contact sheet: the player's eye and outside views, each tile labelled as a Blender view |

### Frames and signs

Every position goes in and comes out in a named frame, so no one has to guess axes:

* `gun`: the gun bone as it stands, Unreal-style axes, cm: +X the gun's left, +Y along the barrel, +Z up (the default)
* `arms`: the arms' space, Unreal-style axes, cm
* `view`: from the eye, cm: +X right, +Y forward, +Z up

Turns use the player's words: `roll` + turns the gun's right side up, `swing` + takes the muzzle left, `pitch` + the
muzzle up. Moves (`right`, `forward`, `up`) are in the view.

A hand round its grip touches the rifle on the idle pose already (a finger on the trigger, fingers round the
handguard). Call `clearance` at `pose_idle` for that baseline, and leave those segments out with `ignore` wildcards.

## Install

You need [Blender](https://www.blender.org/download/) 4.2 or newer and Python 3.10 or newer.

```
uvx poselab-mcp
```

or `pip install poselab-mcp` and run `poselab-mcp`.

Add it to Claude Code:

```
claude mcp add poselab -- uvx poselab-mcp
```

or to any MCP client's configuration:

```json
{
  "mcpServers": {
    "poselab": {
      "command": "uvx",
      "args": ["poselab-mcp"],
      "env": { "POSELAB_BLENDER": "C:/Program Files/Blender Foundation/Blender 4.5/blender.exe" }
    }
  }
}
```

### Settings

| Variable | Meaning |
|---|---|
| `POSELAB_BLENDER` | Blender's executable, if it is not on the PATH or in the usual install folder |
| `POSELAB_RIGS` | a `rigs.json` describing your own rigs (see `examples/rigs.example.json`) |
| `POSELAB_OUT` | the only folder Pose Lab writes to (renders, the worker's log); default `~/.poselab` |

## Rigs

**The built-in sample** (`load_rig {"rig": "sample"}`) is made in Blender from code: two arms with Unreal mannequin
bone names holding a simple rifle with a charging handle that slides back. It needs no files.

**Your own rigs** come from FBX files: the arms mesh, an idle pose, the rifle, and clips. Describe them in a
`rigs.json` (copy `examples/rigs.example.json`) and point `POSELAB_RIGS` at it. Clips can be FBX animations on the same
skeleton, or `<clip>.pose.json` bone data: `{"fps": 30, "frames": [{"bone": [[x, y, z], [w, x, y, z]], ...}, ...]}`,
local location and rotation per bone on the idle armature. (Blender misreads an FBX animation it exported itself when
it imports it again; bone data avoids that. `describe` reports each FBX clip's skeleton fit.)

## Safety

* Pose Lab only reads your rig files. It writes nothing but renders and its log, and only inside `POSELAB_OUT`.
* The Blender worker listens on 127.0.0.1 only, on a free port, and answers only requests that carry the session's
  random token. It runs only Pose Lab's own commands.

## How it is built

* `poselab_mcp/server.py`: the MCP server (the official Python SDK, stdio).
* `poselab_mcp/worker_client.py`: starts one headless Blender on the first call and keeps the rig loaded.
* `poselab_mcp/blender/lab.py`: inside Blender: the rig, the frames, the measures, the IK, the solver, the renders.
* `poselab_mcp/blender/sample.py`: the built-in sample rig.

## Test

```
python examples/selftest.py
```

It starts the server as an MCP client does, loads the sample rig, and replays the question above: turning alone
never shows the port; turning and moving does. The contact sheet lands in `~/.poselab/renders/sheet.png`.

## License

MIT
