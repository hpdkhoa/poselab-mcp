# Publishing Pose Lab

The order matters. The MCP Registry accepts the listing only after the PyPI package is live. It also checks that the
README on PyPI carries `mcp-name: io.github.hpdkhoa/poselab-mcp`. README.md holds that line as an HTML comment.

## 1. GitHub

The GitHub repo already holds a first commit (its `.gitignore` and `LICENSE`). Build on it:

    git init -b main
    git remote add origin https://github.com/hpdkhoa/poselab-mcp.git
    git fetch origin
    git reset origin/main
    git add .
    git commit -F <message file>
    git push -u origin main

`git reset origin/main` points the new branch at GitHub's commit and keeps every file here as it is.

## 2. PyPI

Make a PyPI account and an API token (pypi.org, Account settings, API tokens). Then, with uv:

    uv build
    uv publish --token <your PyPI token>

or with pip tools:

    python -m pip install build twine
    python -m build
    python -m twine upload dist/*

Check it: `uvx poselab-mcp` starts the server (it waits for a client on stdin; Ctrl+C to stop).

## 3. The MCP Registry

Install the publisher (one of):

    npm install -g mcp-publisher
    # or download mcp-publisher from https://github.com/modelcontextprotocol/registry/releases

Then, in this folder (server.json is here):

    mcp-publisher login github
    mcp-publisher publish

`login github` proves you own the io.github.hpdkhoa namespace. Search for it afterwards:
https://registry.modelcontextprotocol.io/v0/servers?search=poselab

## Each release

Run the checks first, with POSELAB_BLENDER set: `python examples/selftest.py`, `python examples/motion_test.py`,
`python examples/edge_cases.py` (no raw Python errors), and `python benchmarks/run.py --oracle` (5 of 5).

Raise the version in three places: `pyproject.toml`, `src/poselab_mcp/__init__.py`, and `server.json` (both
`version` fields). Build and upload to PyPI first, then `mcp-publisher publish`.
