#!/usr/bin/env python3
"""cartographer_grammar — the command grammar a Cartographer mission accepts, as data.

One place that says, for every command kind: where it is legal,
which argument keys exist, their units, which are alternatives, and what limits
apply. Limits are never written here as numbers: each names a constant of
vector_slam_maze.movement_directives and is resolved from that module when asked,
so this file cannot drift from the code that enforces them.

check_args() refuses, before the driver is asked, the mistakes a caller writing
commands as text makes: a missing distance or angle, an unknown key, two keys for
one quantity, and `strict` or `dry_run` inside the args. vector-slam-maze v0.4.0
let the first three through (a move with no distance drove 100 mm); v0.4.1
refuses them itself. The checks stay here so the caller is told which key is
wrong and what is allowed, whichever driver version is installed.
"""
import math

MAX_ARG_KEYS = 16
MAX_STEPS_SCAN = 64
FORBIDDEN_KEYS = ("strict", "dry_run")
# Keys the driver ignores for a kind but that Cartographer's own frontier planner writes, always as zero
# (measured 2026-11-14 on a mission from tests/smoke/cartographer-vector-frontier.sh: every queued move
# carried "dtheta_deg": 0.0). Zero is tolerated with a warning; any other value would be silently dropped
# by the driver, so it is refused.
IGNORED_UNLESS_ZERO = {"move": ("dtheta_deg",)}

# Where each kind is legal. Pinned to the source files by tests/test_grammar.py.
SURFACE_KINDS = {
    "queue": ("probe", "move", "rotate", "translate", "head", "lift", "directives", "dock", "undock", "scan",
              "read-map", "read-pose", "read-battery", "halt", "sleep", "solve-maze"),
    "vector_recipe": ("move", "rotate", "translate", "head", "lift", "directives", "undock", "dock", "scan",
                      "read-map", "solve-maze", "halt"),
    "drive_helper": ("probe", "read-map", "move", "rotate", "turn", "translate", "head", "lift", "directives",
                     "undock", "dock", "scan", "solve-maze", "halt"),
    "directive": ("move", "rotate", "turn", "translate", "head", "lift", "undock", "dock", "halt"),
}
SURFACE_NOTES = {
    "queue": "a line in the mission's cmd-queue.jsonl (skills/_cartographer_lib.sh, _CART_VALID_CMDS)",
    "vector_recipe": "what the Vector recipe will actually send (skills/draft-cartographer-vector-proprio.sh, cmd_send)",
    "drive_helper": "skills/_vector_drive.py --kind",
    "directive": "a step inside a `directives` batch (vector_slam_maze.movement_directives)",
}


def _f(name, typ, unit, note, bound=None):
    """One argument field."""
    assert name and typ in ("number", "boolean", "string", "integer", "list")
    assert isinstance(note, str) and note
    return {"name": name, "type": typ, "unit": unit, "note": note, "bound": bound}


_SPEED_MM = _f("speed_mmps", "number", "mm/s", "Optional. Clamped to the limits; reverse has a lower ceiling.",
               ["MIN_LINEAR_SPEED_MMPS", "MAX_FORWARD_SPEED_MMPS", "MAX_REVERSE_SPEED_MMPS"])
_STANDOFF = _f("standoff_mm", "number", "mm", "Optional. Distance kept from an object the front range sensor sees. "
               "It can be raised above the default. The driver refuses a value below it in strict mode.",
               ["DEFAULT_FORWARD_STANDOFF_MM"])

KINDS = {
    "move": {
        "summary": "Drive straight along the current heading. Positive is forward, negative is reverse.",
        "moves_robot": True,
        "args": [
            _f("dx_mm", "number", "mm", "Signed distance.", ["MAX_FORWARD_DIRECTIVE_MM", "MAX_REVERSE_DIRECTIVE_MM"]),
            _f("dx_m", "number", "m", "Signed distance in metres; alternative to dx_mm."),
            _SPEED_MM,
            _f("speed_mps", "number", "m/s", "Alternative to speed_mmps."),
            _STANDOFF,
        ],
        "exactly_one_of": [["dx_mm", "dx_m"]],
        "at_most_one_of": [["speed_mmps", "speed_mps"]],
        "examples": [{"dx_mm": 200.0}, {"dx_m": -0.05}, {"dx_mm": 300.0, "speed_mmps": 60.0}],
    },
    "rotate": {
        "summary": "Turn in place. Positive turns left (counter-clockwise seen from above).",
        "moves_robot": True,
        "args": [
            _f("angle_deg", "number", "deg", "Signed angle.", ["MAX_ROTATE_DIRECTIVE_DEG"]),
            _f("dtheta_deg", "number", "deg", "Same meaning as angle_deg."),
            _f("dtheta_rad", "number", "rad", "Signed angle in radians."),
            _f("speed_dps", "number", "deg/s", "Optional.", ["MIN_TURN_SPEED_DPS", "MAX_TURN_SPEED_DPS"]),
        ],
        "exactly_one_of": [["angle_deg", "dtheta_deg", "dtheta_rad"]],
        "at_most_one_of": [],
        "examples": [{"angle_deg": 90.0}, {"dtheta_rad": -1.5708}],
    },
    "translate": {
        "summary": "Move to a point given in the robot's own frame (x forward, y left): turn to face it, then "
                   "drive. A point straight behind (dx negative, |dy| within the tolerance) is a plain reverse.",
        "moves_robot": True,
        "args": [
            _f("dx_mm", "number", "mm", "Forward component.", ["MAX_FORWARD_DIRECTIVE_MM", "MAX_REVERSE_DIRECTIVE_MM"]),
            _f("dy_mm", "number", "mm", "Leftward component.", ["PURE_REVERSE_LATERAL_TOL_MM"]),
            _f("dx_m", "number", "m", "Forward component in metres."),
            _f("dy_m", "number", "m", "Leftward component in metres."),
            _f("distance_mm", "number", "mm", "Polar form: distance to the point."),
            _f("bearing_deg", "number", "deg", "Polar form: direction of the point, positive left."),
            _SPEED_MM,
            _STANDOFF,
            _f("preserve_heading", "boolean", None, "Optional. Turn back to the starting heading afterwards."),
        ],
        "exactly_one_of": [],
        "at_most_one_of": [["dx_mm", "dx_m"], ["dy_mm", "dy_m"]],
        "forms": {"cartesian": ["dx_mm", "dy_mm", "dx_m", "dy_m"], "polar": ["distance_mm", "bearing_deg"]},
        "examples": [{"dx_mm": 150.0, "dy_mm": 150.0}, {"distance_mm": 200.0, "bearing_deg": -45.0},
                     {"dx_mm": -40.0, "dy_mm": 0.0}],
    },
    "head": {
        "summary": "Tilt the head.",
        "moves_robot": False,
        "args": [_f("head_deg", "number", "deg", "Head angle.", ["MIN_HEAD_DEG", "MAX_HEAD_DEG"]),
                 _f("angle_deg", "number", "deg", "Same meaning as head_deg.")],
        "exactly_one_of": [["head_deg", "angle_deg"]],
        "at_most_one_of": [],
        "examples": [{"head_deg": 0.0}],
    },
    "lift": {
        "summary": "Raise or lower the lift.",
        "moves_robot": False,
        "args": [_f("height_mm", "number", "mm", "Lift height.", ["LIFT_MIN_MM", "LIFT_MAX_MM"]),
                 _f("lift_norm", "number", "0..1", "Lift height as a fraction of its travel."),
                 _f("height_norm", "number", "0..1", "Same meaning as lift_norm.")],
        "exactly_one_of": [["height_mm", "lift_norm", "height_norm"]],
        "at_most_one_of": [],
        "examples": [{"lift_norm": 1.0}, {"height_mm": 32.0}],
    },
    "directives": {
        "summary": "An ordered batch of directive steps, stopped at the first step that does not complete.",
        "moves_robot": True,
        "args": [_f("directives", "list", None, "Steps, each {\"verb\": <directive kind>, \"args\": {...}}.",
                    ["MAX_DIRECTIVE_STEPS", "MAX_BATCH_DISTANCE_MM", "MAX_BATCH_TURN_DEG", "MAX_BATCH_DURATION_S"])],
        "exactly_one_of": [["directives"]],
        "at_most_one_of": [],
        "examples": [{"directives": [{"verb": "rotate", "args": {"angle_deg": 90.0}},
                                     {"verb": "move", "args": {"dx_mm": 180.0}}]}],
    },
    "scan": {
        "summary": "Rotate through a full circle in eight 45 degree steps, reading the map at each.",
        "moves_robot": True,
        "args": [_f("undock_if_needed", "boolean", None, "Optional. Drive off the charger first.")],
        "exactly_one_of": [], "at_most_one_of": [], "examples": [{}],
    },
    "solve-maze": {
        "summary": "Run the maze driver. As of vector-slam-maze v0.4.0 that driver does not use the bounded "
                   "movement primitives, so the stand-off and range-sensor fixes do not apply to it.",
        "moves_robot": True,
        "args": [_f("mode", "string", None, "Optional. Planner mode; the driver's default is flood-fill."),
                 _f("max_steps", "integer", "steps", "Optional. The driver's default is 8."),
                 _f("marker_map", "string", None, "Optional. Path to a marker map on the robot host."),
                 _f("peek_unknown", "boolean", None, "Optional."),
                 _f("rehydrate_before_solve", "boolean", None, "Optional."),
                 _f("two_pass", "boolean", None, "Optional.")],
        "exactly_one_of": [], "at_most_one_of": [], "examples": [{"mode": "flood-fill", "max_steps": 8}],
    },
}
for _kind, _summary, _moves in (
        ("undock", "Drive off the charger.", True),
        ("dock", "Return to the charger.", True),
        ("halt", "Stop all motors. The universal abort; legal at any time.", False),
        ("probe", "Read the robot's status without moving it.", False),
        ("read-map", "Take a snapshot of the robot's map without moving it.", False),
        ("read-pose", "Listed for the queue; the Vector recipe does not send it.", False),
        ("read-battery", "Listed for the queue; the Vector recipe does not send it.", False),
        ("sleep", "Listed for the queue; the Vector recipe does not send it.", False),
        ("turn", "Same as rotate, but only the drive helper and a directive step accept the word.", True)):
    KINDS[_kind] = {"summary": _summary, "moves_robot": _moves, "args": [], "exactly_one_of": [],
                    "at_most_one_of": [], "examples": [{}]}
KINDS["turn"].update({k: KINDS["rotate"][k] for k in ("args", "exactly_one_of", "at_most_one_of", "examples")})

BOUND_NAMES = (
    "MAX_FORWARD_DIRECTIVE_MM", "MAX_REVERSE_DIRECTIVE_MM", "MAX_FORWARD_CHUNK_MM", "MAX_REVERSE_CHUNK_MM",
    "DEFAULT_FORWARD_STANDOFF_MM", "MIN_LINEAR_SPEED_MMPS", "MAX_FORWARD_SPEED_MMPS", "MAX_REVERSE_SPEED_MMPS",
    "DEFAULT_FORWARD_SPEED_MMPS", "DEFAULT_REVERSE_SPEED_MMPS", "MAX_ROTATE_DIRECTIVE_DEG", "MIN_TURN_SPEED_DPS",
    "MAX_TURN_SPEED_DPS", "DEFAULT_TURN_SPEED_DPS", "MIN_HEAD_DEG", "MAX_HEAD_DEG", "LIFT_MIN_MM", "LIFT_MAX_MM",
    "PURE_REVERSE_LATERAL_TOL_MM", "MAX_DIRECTIVE_STEPS", "MAX_BATCH_DISTANCE_MM", "MAX_BATCH_TURN_DEG",
    "MAX_BATCH_DURATION_S",
)

KNOWN_HAZARDS = (
    "A result with stopped_reason pose_unavailable after motion began does not say how far the robot went.",
    "bounded turns do not check that the robot's body clears nearby walls as it rotates.",
    "The front range sensor is a single beam on the centreline. A narrow object off the centreline is not seen.",
    "solve-maze runs a driver that does not yet use the bounded primitives (tracked as [0e8a96e]).",
    "Vector's own cliff reflex is off while the household holds SDK control (OVERRIDE_BEHAVIORS_PRIORITY: "
    "'will drive off table'), and the drive checks the cliff sensor only after each chunk "
    "(see bounds MAX_FORWARD_CHUNK_MM). A forward move toward unmapped space or a known drop can go over "
    "the edge. Robots have done so repeatedly (2026-11-14).",
    "At DEFAULT control priority Vector does not grant SDK control while its cliff flag is set and it is "
    "parked at an edge. The connection then opens without control, and a behaviour call blocks with the "
    "robot motionless (seen 2026-11-15: 314 s, pose unchanged). A command queued in that state will not "
    "move the robot; it has to be moved away from the edge by hand.",
)


def surfaces_for(kind):
    """Names of the surfaces on which `kind` is legal, in a fixed order."""
    assert isinstance(kind, str)
    assert set(SURFACE_KINDS) == set(SURFACE_NOTES)
    return [s for s in ("queue", "vector_recipe", "drive_helper", "directive") if kind in SURFACE_KINDS[s]]


def resolve_bounds(module):
    """{constant name: value} read from the movement module; {} when it could not be loaded."""
    assert len(BOUND_NAMES) == len(set(BOUND_NAMES))
    if module is None:
        return {}
    out = {}
    for name in BOUND_NAMES:
        value = getattr(module, name, None)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            out[name] = value
    assert len(out) <= len(BOUND_NAMES)
    return out


def _is_number(value):
    """A finite int or float; a bool is not a number here."""
    assert value is None or isinstance(value, object)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    return math.isfinite(float(value))


def _type_ok(field, value):
    """Does `value` have the field's declared type?"""
    assert field["type"] in ("number", "boolean", "string", "integer", "list")
    typ = field["type"]
    if typ == "number":
        return _is_number(value)
    if typ == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if typ == "boolean":
        return isinstance(value, bool)
    if typ == "string":
        return isinstance(value, str) and 0 < len(value) <= 256
    return isinstance(value, list)


def _group_errors(spec, args):
    """Errors about alternatives: exactly-one and at-most-one groups, and the two translate forms."""
    assert isinstance(spec, dict) and isinstance(args, dict)
    errors = []
    for group in spec["exactly_one_of"]:
        present = [k for k in group if k in args]
        if len(present) != 1:
            errors.append("give exactly one of %s (found %s)" % (", ".join(group), ", ".join(present) or "none"))
    for group in spec["at_most_one_of"]:
        present = [k for k in group if k in args]
        if len(present) > 1:
            errors.append("give at most one of %s (found %s)" % (", ".join(group), ", ".join(present)))
    forms = spec.get("forms")
    if forms:
        cart = [k for k in forms["cartesian"] if k in args]
        polar = [k for k in forms["polar"] if k in args]
        if cart and polar:
            errors.append("mixes the cartesian form (%s) with the polar form (%s)" % (", ".join(cart), ", ".join(polar)))
        elif not cart and len(polar) != 2:
            errors.append("give a point: dx and dy, or both distance_mm and bearing_deg")
    return errors


def check_fields(kind, args):
    """(errors, warnings) for one non-batch command. Pure: reads no file and contacts no robot."""
    assert isinstance(kind, str)
    if kind not in KINDS:
        return (["unknown kind %r; known kinds: %s" % (kind, ", ".join(sorted(KINDS)))], [])
    if not isinstance(args, dict):
        return (["args must be a JSON object"], [])
    if len(args) > MAX_ARG_KEYS:
        return (["too many argument keys (%d)" % len(args)], [])
    spec, errors, warnings = KINDS[kind], [], []
    fields = {f["name"]: f for f in spec["args"]}
    for key in sorted(args):
        if key in FORBIDDEN_KEYS:
            errors.append("%r is set by the caller of the driver, never inside args" % key)
        elif key in IGNORED_UNLESS_ZERO.get(kind, ()):
            if _is_number(args[key]) and args[key] == 0:
                warnings.append("%r is ignored by the driver for %s; leave it out" % (key, kind))
            else:
                errors.append("%r is ignored by the driver for %s, so a value other than 0 would be dropped "
                              "silently; use rotate or translate to turn" % (key, kind))
        elif key not in fields:
            errors.append("unknown key %r for %s; allowed: %s" % (key, kind, ", ".join(sorted(fields)) or "none"))
        elif not _type_ok(fields[key], args[key]):
            errors.append("%r must be a %s" % (key, fields[key]["type"]))
    errors.extend(_group_errors(spec, args))
    if "standoff_mm" in args and _is_number(args["standoff_mm"]) and args["standoff_mm"] < 0:
        errors.append("'standoff_mm' must not be negative")
    return (errors, warnings)


def check_args(kind, args):
    """(errors, warnings) for any command, including a `directives` batch."""
    assert isinstance(kind, str)
    errors, warnings = check_fields(kind, args)
    if kind != "directives" or errors:
        return (errors, warnings)
    steps = args["directives"]
    if not steps:
        return (["directives must hold at least one step"], warnings)
    for index, step in enumerate(steps[:MAX_STEPS_SCAN]):
        if not isinstance(step, dict) or set(step) - {"verb", "args"} or not isinstance(step.get("verb"), str):
            errors.append("step %d must be {\"verb\": ..., \"args\": {...}} and nothing else" % index)
            continue
        if step["verb"] not in SURFACE_KINDS["directive"]:
            errors.append("step %d: %r is not a directive kind (%s)" % (index, step["verb"], ", ".join(SURFACE_KINDS["directive"])))
            continue
        sub_errors, sub_warnings = check_fields(step["verb"], step.get("args", {}))
        errors.extend("step %d: %s" % (index, e) for e in sub_errors)
        warnings.extend("step %d: %s" % (index, w) for w in sub_warnings)
    if len(steps) > MAX_STEPS_SCAN:
        errors.append("too many steps to check (%d)" % len(steps))
    return (errors, warnings)


def describe(kind, bounds):
    """The grammar entry for one kind, with each named limit resolved to its current value."""
    assert kind in KINDS and isinstance(bounds, dict)
    spec = KINDS[kind]
    args = []
    for field in spec["args"]:
        entry = {k: v for k, v in field.items() if k != "bound" and v is not None}
        if field["bound"]:
            entry["limits"] = {name: bounds.get(name, "unavailable") for name in field["bound"]}
        args.append(entry)
    out = {"kind": kind, "summary": spec["summary"], "moves_robot": spec["moves_robot"], "legal_on": surfaces_for(kind),
           "args": args, "exactly_one_of": spec["exactly_one_of"], "at_most_one_of": spec["at_most_one_of"],
           "examples": spec["examples"]}
    if spec.get("forms"):
        out["forms"] = spec["forms"]
    assert out["kind"] == kind
    return out
