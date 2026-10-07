# Publishing Pose Lab

The order matters. The MCP Registry accepts the listing only after the PyPI package is live. It also checks that the
README on PyPI carries `mcp-name: io.github.hpdkhoa/poselab-mcp`. README.md holds that line as an HTML comment.

## 1. GitHub

    git init
    git add .
    git commit -m "Pose Lab 0.1.0"
    git branch -M main
    git remote add origin https://github.com/hpdkhoa/poselab-mcp.git
    git push -u origin main

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

Raise the version in three places: `pyproject.toml`, `src/poselab_mcp/__init__.py`, and `server.json` (both
`version` fields). Build and upload to PyPI first, then `mcp-publisher publish`.
