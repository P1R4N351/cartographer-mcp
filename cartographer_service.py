#!/usr/bin/env python3
"""cartographer_service — the tools of cartographer-mcp. Transport-independent.

Authored-by: CPCS. Six tools for the intelligence that decides where a robot goes:
read what Cartographer wrote, and learn and check the exact syntax of a command.

THIS SERVER MOVES NOTHING. It does not queue a command, run the drive helper or
contact a robot. `cartographer_check_command` plans offline with the driver's own
planner (vector_slam_maze.movement_directives.plan_directive, strict) and returns
the record and the argv a caller would use; sending them is a separate act by
whoever holds that authority.
"""
import importlib
import json
import math
import os
import sys

import cartographer_grammar as grammar
import cartographer_read as reader

SERVER_NAME = "cartographer-mcp"
SERVER_VERSION = "0.1.0"
DEFAULT_PROTOCOL = "2024-11-05"
DRY_RUN_KINDS = ("move", "rotate", "turn", "translate", "head", "lift", "directives")
_HERE = os.path.dirname(os.path.abspath(__file__))
_WORKSPACE = os.path.dirname(os.path.dirname(_HERE))

QUEUE_RECORD_SHAPE = {
    "id": "command id, assigned by Cartographer when it queues the command",
    "ts": "when it was queued",
    "kind": "the command kind",
    "args": "the argument object for that kind",
    "deadline_s": "positive whole seconds Cartographer waits for a result before acknowledging with reason deadline",
    "expected_delta": "{x, y, theta}: the pose change the command should produce, metres and radians, in the robot's own frame",
    "sent": "false until the recipe dispatches it",
    "sent_at": "when it was dispatched",
    "ack": "null until the result is read; then {reason, at}",
    "result": "null until the result is read; then the driver's output",
}
CONVENTIONS = (
    "Distances in args are millimetres unless the key ends in _m. Angles are degrees unless the key ends in _rad.",
    "Positive rotation is to the robot's left (counter-clockwise seen from above).",
    "The robot's own frame: x forward, y left.",
    "One command at a time: a mission accepts a new command only after the previous one is acknowledged.",
    "A mission's pose is in metres and radians in the mission's map frame.",
)

TOOLS = [
    {"name": "cartographer_grammar",
     "description": "The exact syntax of Cartographer movement commands: every kind, where it is legal, its argument "
                    "keys with units and alternatives, the current limits read from the driver, the shape of a queued "
                    "command, what each stop reason means, and known hazards. Call this before composing a command. "
                    "Pass `kind` for one kind only.",
     "inputSchema": {"type": "object", "properties": {"kind": {"type": "string"}}}},
    {"name": "cartographer_check_command",
     "description": "Check one command without moving anything. Refuses unknown keys, a missing distance or angle, "
                    "ambiguous alternatives and values outside the limits, then plans it with the driver's own "
                    "planner in strict mode. Returns errors, warnings, the plan, and the queue record and dry-run "
                    "argv for it. A command that is not valid here should not be sent.",
     "inputSchema": {"type": "object", "required": ["kind"],
                     "properties": {"kind": {"type": "string"}, "args": {"type": "object"}}}},
    {"name": "cartographer_missions",
     "description": "List Cartographer missions on this host, newest first, with robot, space, status and phase.",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "cartographer_mission",
     "description": "One mission, parsed: status and phase, the robot's pose, budgets with what remains, whether a new "
                    "command can be sent now and why not, the largest frontiers with distance and bearing from the "
                    "robot, and the most recent commands and events with their results interpreted.",
     "inputSchema": {"type": "object", "required": ["mission_id"],
                     "properties": {"mission_id": {"type": "string"},
                                    "events": {"type": "integer", "minimum": 1, "maximum": reader.MAX_TAIL_ITEMS},
                                    "commands": {"type": "integer", "minimum": 1, "maximum": reader.MAX_TAIL_ITEMS}}}},
    {"name": "cartographer_parse_result",
     "description": "Say what one driver output means: a motion result, a batch result, a dry-run plan, a status "
                    "line or a queue record. Gives the category (completed, blocked, hazard, refused, rejected, "
                    "busy, planned, unknown), a plain meaning, what the robot did and the pose change reported.",
     "inputSchema": {"type": "object", "required": ["result"],
                     "properties": {"result": {"type": ["object", "string"]}}}},
    {"name": "cartographer_map",
     "description": "A mission's occupancy grid as text: cell counts and a small character picture "
                    "(# occupied, . free, ? unknown, R robot).",
     "inputSchema": {"type": "object", "required": ["mission_id"],
                     "properties": {"mission_id": {"type": "string"},
                                    "cols": {"type": "integer", "minimum": 8, "maximum": reader.MAX_COLS}}}},
]


def config():
    """Where the missions and the movement package are. Environment wins; defaults follow the workspace layout."""
    assert os.path.isdir(_HERE)
    cfg = {"root": os.environ.get("CARTOGRAPHER_ROOT") or os.path.join(_WORKSPACE, "cartographer"),
           "movement_pkg": os.environ.get("CARTOGRAPHER_MOVEMENT_PKG") or os.path.join(_WORKSPACE, "skills", "vector-slam")}
    cfg["movement"], cfg["movement_error"] = load_movement(cfg["movement_pkg"])
    assert (cfg["movement"] is None) != (cfg["movement_error"] is None)
    return cfg


def load_movement(pkg_dir):
    """(module, error): the driver's movement module, imported from `pkg_dir`. Never raises."""
    assert isinstance(pkg_dir, str) and pkg_dir
    if not os.path.isfile(os.path.join(pkg_dir, "vector_slam_maze", "movement_directives.py")):
        return (None, "no vector_slam_maze/movement_directives.py under %s" % pkg_dir)
    if pkg_dir not in sys.path:
        sys.path.insert(0, pkg_dir)
    try:
        module = importlib.import_module("vector_slam_maze.movement_directives")
    except Exception as exc:  # noqa: BLE001 - any import failure means "planner unavailable", said plainly
        return (None, "could not import the movement module (%s)" % type(exc).__name__)
    if not all(callable(getattr(module, name, None)) for name in ("plan_directive", "plan_directive_batch")):
        return (None, "the movement module has no planner")
    return (module, None)


def _text(obj, is_error=False):
    """MCP tool result envelope: one text block carrying JSON."""
    assert isinstance(obj, dict)
    return {"content": [{"type": "text", "text": json.dumps(obj, ensure_ascii=False)}], "isError": bool(is_error)}


def _err(msg):
    """A tool error."""
    assert isinstance(msg, str) and msg
    return _text({"error": msg}, is_error=True)


def tool_grammar(args, cfg):
    """The command grammar, whole or for one kind."""
    assert isinstance(args, dict) and isinstance(cfg, dict)
    bounds = grammar.resolve_bounds(cfg["movement"])
    kind = args.get("kind")
    if kind is not None and kind not in grammar.KINDS:
        return _err("unknown kind %r; known kinds: %s" % (kind, ", ".join(sorted(grammar.KINDS))))
    kinds = [kind] if kind else sorted(grammar.KINDS)
    out = {"kinds": [grammar.describe(k, bounds) for k in kinds],
           "limits_source": cfg["movement_error"] and ("unavailable: %s" % cfg["movement_error"])
           or ("read from %s" % getattr(cfg["movement"], "__file__", "the movement module")),
           "conventions": list(CONVENTIONS)}
    if kind is None:
        out.update({
            "surfaces": {s: {"what": grammar.SURFACE_NOTES[s], "kinds": list(grammar.SURFACE_KINDS[s])} for s in grammar.SURFACE_KINDS},
            "limits": bounds,
            "never_in_args": list(grammar.FORBIDDEN_KEYS),
            "queue_record": QUEUE_RECORD_SHAPE,
            "stop_reasons": {k: {"category": v[0], "meaning": v[1], "robot": v[2]} for k, v in sorted(reader.STOP_REASONS.items())},
            "ack_reasons": reader.ACK_REASONS,
            "known_hazards": list(grammar.KNOWN_HAZARDS),
            "this_server": "reads and checks only; it queues nothing and contacts no robot",
        })
    return _text(out)


def _plan(kind, args, movement):
    """(plan, error) from the driver's own planner in strict mode; (None, None) when the kind has no planner."""
    assert isinstance(kind, str) and isinstance(args, dict)
    if kind not in grammar.SURFACE_KINDS["directive"] and kind != "directives":
        return (None, None)
    if movement is None:
        return (None, "the driver's planner is unavailable, so the limits could not be checked")
    try:
        if kind == "directives":
            plan = movement.plan_directive_batch(args["directives"], strict=True)
        else:
            plan = movement.plan_directive(kind, args, strict=True)
    except Exception as exc:  # noqa: BLE001 - a planner crash is a failed check, never a pass
        return (None, "the driver's planner failed on this command (%s)" % type(exc).__name__)
    if not isinstance(plan, dict) or plan.get("ok") is not True:
        reason = plan.get("stopped_reason") if isinstance(plan, dict) else None
        meaning = reader.interpret_stop(reason)["meaning"]
        return (plan if isinstance(plan, dict) else None, "the driver's planner refused it: %s. %s" % (reason, meaning))
    return (plan, None)


def _expected_delta(kind, plan):
    """{x, y, theta} in metres and radians, in the robot's frame, from a single-command plan."""
    assert isinstance(kind, str)
    norm = plan.get("normalized") if isinstance(plan, dict) else None
    if not isinstance(norm, dict):
        return {}
    if kind == "move":
        return {"x": round(norm["dist_mm"] / 1000.0, 4), "y": 0.0, "theta": 0.0}
    if kind in ("rotate", "turn"):
        return {"x": 0.0, "y": 0.0, "theta": round(math.radians(norm["dtheta_deg"]), 5)}
    if kind == "translate":
        theta = 0.0 if norm.get("preserve_heading") else math.radians(norm["bearing_deg"])
        return {"x": round(norm["dx_mm"] / 1000.0, 4), "y": round(norm["dy_mm"] / 1000.0, 4), "theta": round(theta, 5)}
    return {"x": 0.0, "y": 0.0, "theta": 0.0}


def _queue_record(kind, args, plan):
    """The record Cartographer would hold for this command, or why it cannot hold one."""
    assert isinstance(kind, str) and isinstance(args, dict)
    missing = [s for s in ("queue", "vector_recipe") if kind not in grammar.SURFACE_KINDS[s]]
    if missing:
        hint = " Use rotate." if kind == "turn" else ""
        return {"record": None, "why": "%s is not legal on: %s.%s" % (kind, ", ".join(missing), hint)}
    seconds = plan.get("estimated_duration_s") if isinstance(plan, dict) else None
    if kind == "directives" and isinstance(plan, dict):
        seconds = sum(float(p.get("estimated_duration_s", 0.0)) for p in plan.get("plans", [])[:64])
    deadline = max(10, int(math.ceil(float(seconds) * 2.0)) + 5) if isinstance(seconds, (int, float)) else 15
    record = {"kind": kind, "args": args, "deadline_s": deadline,
              "expected_delta": {} if kind == "directives" else _expected_delta(kind, plan)}
    return {"record": record, "why": None,
            "note": "id, ts, sent, sent_at, ack and result are filled in by Cartographer. deadline_s here is twice "
                    "the planner's estimate plus five seconds, at least ten."}


def tool_check_command(args, cfg):
    """Check one command against the grammar and the driver's planner. Moves nothing."""
    assert isinstance(args, dict) and isinstance(cfg, dict)
    kind, cmd_args = args.get("kind"), args.get("args", {})
    if not isinstance(kind, str):
        return _err("kind is required and must be a string")
    errors, warnings = grammar.check_args(kind, cmd_args)
    plan = None
    if not errors:
        plan, plan_err = _plan(kind, cmd_args, cfg["movement"])
        if plan_err:
            errors.append(plan_err)
    known = kind in grammar.KINDS
    out = {"valid": not errors, "kind": kind, "errors": errors, "warnings": warnings,
           "legal_on": grammar.surfaces_for(kind) if known else [],
           "moves_robot": grammar.KINDS[kind]["moves_robot"] if known else None,
           "plan": plan if not errors else None,
           "checked_against": "grammar and the driver's planner (strict)" if plan is not None
           else "grammar only" + ("" if errors else "; this kind has no offline planner"),
           "moved_anything": False}
    if not errors:
        out["queue"] = _queue_record(kind, cmd_args, plan)
        if kind in DRY_RUN_KINDS:
            out["dry_run_argv"] = ["python3", "skills/_vector_drive.py", "--kind", kind, "--args",
                                   json.dumps(cmd_args, sort_keys=True), "--strict", "--dry-run"]
            out["argv_note"] = ("An argv list: pass it as separate arguments, never joined into a shell string. "
                                "A live run replaces --dry-run with --via-vector, the operator's consent flag.")
    elif known:
        out["grammar"] = grammar.describe(kind, grammar.resolve_bounds(cfg["movement"]))
    return _text(out)


def tool_missions(_args, cfg):
    """List missions."""
    assert isinstance(cfg, dict)
    return _text(reader.list_missions(cfg["root"]))


def tool_mission(args, cfg):
    """One mission, parsed."""
    assert isinstance(args, dict) and isinstance(cfg, dict)
    events, commands = args.get("events", 10), args.get("commands", 10)
    if not all(isinstance(v, int) and not isinstance(v, bool) for v in (events, commands)):
        return _err("events and commands must be integers")
    out = reader.read_mission(cfg["root"], args.get("mission_id"), events=events, commands=commands)
    return _text(out, is_error="error" in out)


def tool_parse_result(args, _cfg):
    """Interpret one driver output."""
    assert isinstance(args, dict)
    result = args.get("result")
    if not isinstance(result, (dict, str)):
        return _err("result must be a JSON object or its text")
    out = reader.parse_result(result)
    return _text(out, is_error="error" in out)


def tool_map(args, cfg):
    """A mission's occupancy grid as text."""
    assert isinstance(args, dict) and isinstance(cfg, dict)
    cols = args.get("cols", 48)
    if not isinstance(cols, int) or isinstance(cols, bool):
        return _err("cols must be an integer")
    out = reader.map_summary(cfg["root"], args.get("mission_id"), cols=cols)
    return _text(out, is_error="error" in out)


HANDLERS = {"cartographer_grammar": tool_grammar, "cartographer_check_command": tool_check_command,
            "cartographer_missions": tool_missions, "cartographer_mission": tool_mission,
            "cartographer_parse_result": tool_parse_result, "cartographer_map": tool_map}


def call_tool(name, arguments, cfg):
    """Run one tool by name."""
    assert isinstance(cfg, dict)
    handler = HANDLERS.get(name)
    if handler is None:
        return _err("unknown tool: %s" % name)
    if arguments is None:
        arguments = {}
    if not isinstance(arguments, dict):
        return _err("arguments must be an object")
    return handler(arguments, cfg)


def dispatch(msg, cfg):
    """One MCP dispatcher. Returns a response dict, or None for a notification."""
    assert isinstance(cfg, dict)
    if not isinstance(msg, dict):
        return None
    method, msg_id = msg.get("method"), msg.get("id")
    params = msg.get("params") if isinstance(msg.get("params"), dict) else {}
    if method == "initialize":
        return {"jsonrpc": "2.0", "id": msg_id, "result": {
            "protocolVersion": params.get("protocolVersion") or DEFAULT_PROTOCOL,
            "capabilities": {"tools": {}},
            "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION}}}
    if method in ("notifications/initialized", "initialized"):
        return None
    if method == "ping":
        return {"jsonrpc": "2.0", "id": msg_id, "result": {}}
    if method == "tools/list":
        return {"jsonrpc": "2.0", "id": msg_id, "result": {"tools": TOOLS}}
    if method == "tools/call":
        return {"jsonrpc": "2.0", "id": msg_id, "result": call_tool(params.get("name"), params.get("arguments"), cfg)}
    if msg_id is None:
        return None
    return {"jsonrpc": "2.0", "id": msg_id, "error": {"code": -32601, "message": "method not found: %s" % method}}
