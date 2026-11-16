---
name: cartographer-mcp
description: MCP (stdio) interface for the intelligence that decides where a robot goes. Parses what a Cartographer mission wrote and gives the exact syntax of movement commands, with an offline check. It moves nothing.
---

# cartographer-mcp

Authored-by: CPCS. Charter it serves: `CARTOGRAPHER.md`.

Cartographer turns a robot's sensing into text and takes commands back as text.
The model that decides what to do next is chosen for judgement, not for
vision, so it needs two things stated exactly: what the mission's files say,
and how a command must be written. This server gives it both.

It reads mission files and checks commands. **It queues nothing, runs no drive
helper and contacts no robot.** Sending a command stays a separate act by
whoever holds that authority.

## Tools

| Tool | What it gives |
|---|---|
| `cartographer_grammar` | Every command kind: where it is legal, its argument keys with units and alternatives, the current limits, the shape of a queued command, what each stop reason means, and known hazards. Pass `kind` for one kind. |
| `cartographer_check_command` | Checks one `{kind, args}` without moving anything. Returns errors, warnings, the driver's own plan, the queue record and a dry-run argv. |
| `cartographer_missions` | Missions on this host, newest first. |
| `cartographer_mission` | One mission parsed: status, pose, budgets with what remains, whether a command can be sent now and why not, the largest frontiers with distance and bearing from the robot, recent commands and events with results interpreted. |
| `cartographer_parse_result` | What one driver output means: category, plain meaning, what the robot did, pose change reported. |
| `cartographer_map` | The occupancy grid as cell counts and a small text picture. |

## Run

```json
{"command": "python3", "args": ["<workspace>/skills/cartographer-mcp/cartographer-mcp.py"]}
```

Newline-delimited JSON-RPC on stdio, standard library only, no listener.

| Variable | Default | Meaning |
|---|---|---|
| `CARTOGRAPHER_ROOT` | `<workspace>/cartographer` | Directory holding mission directories. |
| `CARTOGRAPHER_MOVEMENT_PKG` | `<workspace>/skills/vector-slam` | Directory holding `vector_slam_maze`. |

The server's first stderr line says whether the planner loaded. Without it,
`cartographer_check_command` reports a movement command as not valid; it never
passes a command it could not check.

## Where the syntax comes from

Nothing about limits is written in this skill as a number. Each limit names a
constant of `vector_slam_maze.movement_directives` and is read from that module
when asked. The lists of legal kinds are pinned by tests to the four files that
decide them:

| Surface | Decided by |
|---|---|
| queue | `_CART_VALID_CMDS` in `skills/_cartographer_lib.sh` |
| vector_recipe | `cmd_send` in `skills/draft-cartographer-vector-proprio.sh` |
| drive_helper | `--kind` choices in `skills/_vector_drive.py` |
| directive | `VALID_DIRECTIVE_VERBS` in `movement_directives.py` |

A change in any of those files fails a test here, as does a stop reason the
driver can emit that this server does not explain.

## Checked before the driver is asked

`cartographer_check_command` refuses the mistakes a caller writing commands as
text makes: a missing distance or angle, an unknown key, two keys for one
quantity, and `strict` or `dry_run` inside the args. It then plans with the
driver's planner in strict mode, so an out-of-range value is refused, not
clamped, and a stand-off below the driver's default is refused.

`vector-slam-maze` v0.4.0 let the first three through: a `move` with no
distance key drove a default 100 mm, an unknown key was ignored, and a
`strict` key in the args overrode the caller. v0.4.1 refuses them itself. The
checks stay here so the caller is told which key is wrong and what is allowed,
whichever driver version is installed.

One tolerated key: Cartographer's own frontier planner writes `"dtheta_deg": 0`
into every `move` it queues, and the driver ignores it. Zero is accepted with a
warning; any other value is refused, because the turn would be dropped
silently.

## Known limits

- `cartographer_check_command` plans; it does not know what is in front of the
  robot. A valid command can still stop at an obstacle.
- `scan`, `solve-maze`, `probe` and `read-map` have no offline planner, so they
  are checked against the grammar only, and the result says so.
- `read-pose`, `read-battery` and `sleep` are legal in the queue, but the Vector
  recipe does not send them and their arguments are not defined anywhere this
  server can read.
- The text picture marks the robot only when `grid.yaml` gives a resolution and
  an origin and the state gives a pose.
- Only the tail of `cmd-queue.jsonl` and `events.jsonl` is read (512 KiB).
- The "no way to act" test is a tripwire over imports and file modes, not a
  proof.

## Tests

```bash
python3 -m unittest discover -s skills/cartographer-mcp/tests -v
python3 skills/cartographer-mcp/tests/mutate.py
python3 skills/p10-scan.py --fail-on warn --fail-on-unscannable skills/cartographer-mcp
```

`tests/fixtures/real-mission/` is a mission written by Cartographer itself
(`tests/smoke/cartographer-vector-frontier.sh`, mock driver, no robot). Its
files are unedited except that the robot's serial number is blanked in
`state.json`. The cell counts this server computes from its grid are checked
against the counts Cartographer recorded in its own state.

`mutate.py` breaks the server one line at a time in a scratch copy and reports
whether the suite noticed, with the tests that failed. A fault of the harness
is reported as `SETUP-ERROR`, never as a verdict about the server.
