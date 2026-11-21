#!/usr/bin/env python3
# description: MCP (stdio) front for Cartographer — parse a mission's output and learn and check the exact syntax of movement commands, without moving anything.
# invocation: {"command": "python3", "args": ["<workspace>/skills/cartographer-mcp/cartographer-mcp.py"]}   (stdio JSON-RPC, one message per line)
# tags: [mcp, cartographer, robots, vector, navigation]
"""cartographer-mcp — STDIO frontend. Transport only.

The usual minimal shape for a local MCP server:
newline-delimited JSON-RPC on stdio, one dispatcher, no HTTP listener, standard
library only. The tools are in cartographer_service.py.

This server reads mission files and checks commands. It queues nothing, runs no
drive helper and contacts no robot.

Environment
  CARTOGRAPHER_ROOT          mission directory (default <workspace>/cartographer)
  CARTOGRAPHER_MOVEMENT_PKG  directory holding vector_slam_maze (default <workspace>/skills/vector-slam)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cartographer_service as svc                              # noqa: E402

MAX_LINE_BYTES = 4 << 20


def log(msg):
    """One audit line on stderr; stdout carries the protocol only."""
    assert isinstance(msg, str)
    sys.stderr.write("%s: %s\n" % (svc.SERVER_NAME, msg))
    sys.stderr.flush()


def handle_line(line, cfg):
    """The response for one input line, or None when there is nothing to send."""
    assert isinstance(line, str) and isinstance(cfg, dict)
    if len(line) > MAX_LINE_BYTES:
        log("skipped an oversized line")
        return None
    try:
        msg = json.loads(line)
    except ValueError:
        log("skipped a malformed line")
        return None
    try:
        return svc.dispatch(msg, cfg)
    except Exception as exc:  # noqa: BLE001 - a transport must never die on one call
        log("dispatch failed: %s" % type(exc).__name__)
        mid = msg.get("id") if isinstance(msg, dict) else None
        if mid is None:
            return None
        return {"jsonrpc": "2.0", "id": mid, "error": {"code": -32603, "message": "internal error"}}


def main():
    """Serve until stdin closes."""
    cfg = svc.config()
    planner = "planner ok" if cfg["movement"] is not None else "PLANNER UNAVAILABLE (%s)" % cfg["movement_error"]
    log("%s %s stdio ready (root=%s; %s)" % (svc.SERVER_NAME, svc.SERVER_VERSION, cfg["root"], planner))
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        response = handle_line(line, cfg)
        if response is not None:
            sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
            sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
