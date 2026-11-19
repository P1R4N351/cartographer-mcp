#!/usr/bin/env python3
"""Tests for cartographer-mcp. Standard library only; no robot, no network.

Authored-by: CPCS. Run:  python3 -m unittest discover -s skills/cartographer-mcp/tests -v
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
SKILL = HERE.parent
WORKSPACE = SKILL.parent.parent
sys.path.insert(0, str(SKILL))

import cartographer_grammar as grammar      # noqa: E402
import cartographer_read as reader          # noqa: E402
import cartographer_service as svc          # noqa: E402

MOVEMENT_PKG = os.environ.get("CARTOGRAPHER_MOVEMENT_PKG") or str(WORKSPACE / "skills" / "vector-slam")
MOVEMENT, MOVEMENT_ERR = svc.load_movement(MOVEMENT_PKG)
CATEGORIES = {"completed", "blocked", "hazard", "refused", "rejected", "busy", "planned", "unknown"}


def cfg_for(root, movement=MOVEMENT):
    """A service configuration over a temporary mission root."""
    return {"root": str(root), "movement_pkg": MOVEMENT_PKG, "movement": movement,
            "movement_error": None if movement is not None else "test: planner withheld"}


def call(name, arguments, cfg):
    """Call one tool through the dispatcher and return (payload, isError)."""
    resp = svc.dispatch({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": name, "arguments": arguments}}, cfg)
    result = resp["result"]
    return json.loads(result["content"][0]["text"]), result["isError"]


def write_pgm(path, rows):
    """Write a binary PGM from a list of equal-length rows of pixel values."""
    with open(path, "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (len(rows[0]), len(rows)))
        for row in rows:
            fh.write(bytes(row))


def make_mission(root, name="2026-11-14-0500-vector-demo-studio", theta=0.0, awaiting=True):
    """A mission directory with every file the reader knows."""
    mdir = Path(root) / name
    mdir.mkdir(parents=True)
    state = {"schema_version": 1, "mission_id": name, "robot": "vector-demo", "space_name": "studio", "strategy": "frontier",
             "status": "exploring", "phase": "explore", "updated_at": "2026-11-14T05:00:00Z",
             "pose": {"x_m": 0.0, "y_m": 0.0, "theta_rad": theta},
             "budgets": {"distance_m": {"cap": 40.0, "used": 3.8}, "seconds": {"cap": 1800, "used": 394},
                         "collisions": {"cap": 3, "used": 0}},
             "map_stats": {"known_cells": 12}}
    if awaiting:
        state["awaiting"] = {"cmd_id": "cmd-0003"}
    (mdir / "state.json").write_text(json.dumps(state))
    goal = {"ok": True, "stopped_reason": "goal", "prox_mm": 400.0, "actual_delta": {"x": 0.2, "y": 0.0, "theta": 0.0},
            "clamped": False, "chunks_executed": 2, "motion_result": {"ok": True}}
    stop = {"ok": False, "stopped_reason": "proximity_standoff", "prox_mm": 80.0,
            "actual_delta": {"x": 0.14, "y": 0.0, "theta": 0.0}, "clamped": False, "chunks_executed": 1, "motion_result": {"ok": False}}
    queue = [
        {"id": "cmd-0001", "kind": "move", "args": {"dx_mm": 200.0}, "deadline_s": 11, "expected_delta": {"x": 0.2, "y": 0, "theta": 0},
         "sent": True, "ack": {"reason": "completed", "at": "t1"}, "result": goal},
        {"id": "cmd-0002", "kind": "move", "args": {"dx_mm": 180.0}, "deadline_s": 11, "expected_delta": {"x": 0.18, "y": 0, "theta": 0},
         "sent": True, "ack": {"reason": "halted", "at": "t2"}, "result": stop},
        {"id": "cmd-0003", "kind": "rotate", "args": {"angle_deg": 90.0}, "deadline_s": 10, "expected_delta": {},
         "sent": True, "ack": None, "result": None},
    ]
    (mdir / "cmd-queue.jsonl").write_text("\n".join(json.dumps(q) for q in queue) + "\nthis line is not json\n")
    (mdir / "events.jsonl").write_text(json.dumps({"ts": "t0", "phase": "plan", "msg": "frontier step\n"}) + "\n")
    (mdir / "frontiers.json").write_text(json.dumps([{"centroid": [0.0, 1.0], "size": 9, "cells": []},
                                                     {"centroid": [2.0, 0.0], "size": 3, "cells": []}]))
    write_pgm(mdir / "grid.pgm", [[0, 0, 254, 254], [205, 205, 254, 254], [205, 205, 205, 205], [254, 254, 254, 254]])
    (mdir / "grid.yaml").write_text("image: grid.pgm\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n")
    return mdir


class SourcePins(unittest.TestCase):
    """The grammar's tables against the files that actually decide. A change in those files fails here."""

    def read(self, rel):
        path = WORKSPACE / rel
        if not path.is_file() and rel.startswith("skills/vector-slam/vector_slam_maze/") and MOVEMENT is not None:
            cand = Path(MOVEMENT.__file__).resolve().parent / rel.rsplit("/", 1)[1]
            if cand.is_file():
                path = cand
        if not path.is_file():
            msg = "SETUP-ERROR: %s is missing, so the grammar cannot be checked against it" % rel
            if os.environ.get("CARTOGRAPHER_REQUIRE_SOURCE_PINS") == "1":
                self.fail(msg)
            self.skipTest("private workspace source pin absent in standalone checkout: %s" % rel)
        return path.read_text()

    def test_queue_kinds_match_the_cartographer_library(self):
        match = re.search(r'_CART_VALID_CMDS="([^"]+)"', self.read("skills/_cartographer_lib.sh"))
        self.assertIsNotNone(match)
        self.assertEqual(sorted(match.group(1).split()), sorted(grammar.SURFACE_KINDS["queue"]))

    def test_recipe_kinds_match_what_the_vector_recipe_sends(self):
        text = self.read("skills/draft-cartographer-vector-proprio.sh")
        body = text[text.index("cmd_send() {"):text.index("_poll_consume_result() {")]
        sent = re.search(r"\n\s+([a-z|-]+)\)\n\s+_spawn_vector_helper_async", body)
        self.assertIsNotNone(sent)
        self.assertIn("halt) cmd_halt", body)
        self.assertEqual(sorted(sent.group(1).split("|") + ["halt"]), sorted(grammar.SURFACE_KINDS["vector_recipe"]))

    def test_drive_helper_kinds_match_its_argument_parser(self):
        match = re.search(r"choices=\[([^\]]+)\]", self.read("skills/_vector_drive.py"))
        self.assertIsNotNone(match)
        self.assertEqual(sorted(re.findall(r'"([a-z-]+)"', match.group(1))), sorted(grammar.SURFACE_KINDS["drive_helper"]))
        dry = re.search(r"DRY_RUN_KINDS = \(([^)]+)\)", self.read("skills/_vector_drive.py"))
        self.assertEqual(sorted(re.findall(r'"([a-z-]+)"', dry.group(1))), sorted(svc.DRY_RUN_KINDS))

    def test_directive_kinds_and_limits_come_from_the_movement_module(self):
        if MOVEMENT is None:
            if os.environ.get("CARTOGRAPHER_REQUIRE_SOURCE_PINS") == "1":
                self.fail("SETUP-ERROR: movement module not loadable: %s" % MOVEMENT_ERR)
            self.skipTest("movement module absent in standalone checkout: %s" % MOVEMENT_ERR)
        self.assertEqual(sorted(MOVEMENT.VALID_DIRECTIVE_VERBS), sorted(grammar.SURFACE_KINDS["directive"]))
        bounds = grammar.resolve_bounds(MOVEMENT)
        self.assertEqual(sorted(bounds), sorted(grammar.BOUND_NAMES))

    def test_every_stop_reason_the_driver_can_emit_is_explained(self):
        named = re.compile(r'detail="([a-z_]+)"|else "([a-z_]+)"|"stopped_reason": f?"([a-z_]+)|return "([a-z_]+)"\n|f"([a-z_]+):\{')
        in_tuple = re.compile(r'"([a-z]+(?:_[a-z]+)+)"(?![\]:]| in )')    # not a key, and not a membership test
        nested_call = re.compile(r"[A-Za-z_.]+\([^()]*\)")
        found = set()
        for rel in ("movement_directives.py", "sdk_session.py"):
            text = self.read("skills/vector-slam/vector_slam_maze/" + rel)
            section = text if rel == "movement_directives.py" else text[text.index("def _evaluate_hazard"):text.index("def poll_latest_nav_map")]
            found.update(g for m in named.finditer(section) for g in m.groups() if g)
            for line in section.splitlines():
                if line.lstrip().startswith("return ("):
                    for _pass in range(6):      # drop nested calls: their string arguments are names, not reasons
                        line = nested_call.sub("", line)
                    found.update(in_tuple.findall(line))
        found -= {"ok", "unknown_verb", "invalid_step"}
        self.assertGreaterEqual(len(found), 30, "the scan found too few reasons to be trusted: %s" % sorted(found))
        self.assertEqual(sorted(found - set(reader.STOP_REASONS)), [])
        for prefixed in ("unknown_keys:distance", "invalid_boolean:strict", "invalid_numeric:dx_mm", "unknown_verb:fly", "invalid_step:3"):
            self.assertEqual(reader.interpret_stop(prefixed)["category"], "rejected", prefixed)


class GrammarChecks(unittest.TestCase):
    def test_every_example_is_valid_and_plans(self):
        for kind, spec in sorted(grammar.KINDS.items()):
            for example in spec["examples"]:
                out, is_err = call("cartographer_check_command", {"kind": kind, "args": example}, cfg_for("/nonexistent"))
                self.assertFalse(is_err)
                self.assertTrue(out["valid"], "%s %s: %s" % (kind, example, out["errors"]))
                self.assertFalse(out["moved_anything"])

    def test_every_alternative_key_is_one_the_planner_honours(self):
        values = {"dx_mm": 120.0, "dx_m": 0.12, "angle_deg": 30.0, "dtheta_deg": 30.0, "dtheta_rad": 0.5, "head_deg": 10.0,
                  "height_mm": 50.0, "lift_norm": 0.5, "height_norm": 0.5}
        expect = {"dx_mm": ("dist_mm", 120.0), "dx_m": ("dist_mm", 120.0), "angle_deg": ("dtheta_deg", 30.0),
                  "dtheta_deg": ("dtheta_deg", 30.0), "dtheta_rad": ("dtheta_deg", math.degrees(0.5)), "head_deg": ("head_deg", 10.0),
                  "height_mm": ("lift_norm", 0.3), "lift_norm": ("lift_norm", 0.5), "height_norm": ("lift_norm", 0.5)}
        seen = 0
        for kind in ("move", "rotate", "head", "lift"):
            for key in grammar.KINDS[kind]["exactly_one_of"][0]:
                if kind == "head" and key == "angle_deg":
                    expected = ("head_deg", 30.0)
                else:
                    expected = expect[key]
                out, _ = call("cartographer_check_command", {"kind": kind, "args": {key: values[key]}}, cfg_for("/nonexistent"))
                self.assertTrue(out["valid"], (kind, key, out["errors"]))
                self.assertAlmostEqual(out["plan"]["normalized"][expected[0]], expected[1], places=3, msg="%s %s" % (kind, key))
                seen += 1
        self.assertEqual(seen, 10)

    def test_translate_forms(self):
        out, _ = call("cartographer_check_command", {"kind": "translate", "args": {"dx_mm": 0.0, "dy_mm": 200.0}}, cfg_for("/x"))
        self.assertTrue(out["valid"])
        self.assertAlmostEqual(out["plan"]["normalized"]["bearing_deg"], 90.0, places=3)
        self.assertAlmostEqual(out["queue"]["record"]["expected_delta"]["theta"], math.radians(90.0), places=4)
        polar, _ = call("cartographer_check_command", {"kind": "translate", "args": {"distance_mm": 200.0, "bearing_deg": 90.0}}, cfg_for("/x"))
        self.assertAlmostEqual(polar["plan"]["normalized"]["dy_mm"], 200.0, places=3)
        for bad in ({"dx_mm": 100.0, "distance_mm": 100.0, "bearing_deg": 0.0}, {"distance_mm": 100.0}, {}, {"dx_mm": 1.0, "dx_m": 1.0, "dy_mm": 0.0}):
            errors, _w = grammar.check_args("translate", bad)
            self.assertTrue(errors, bad)

    def test_what_the_driver_lets_through_is_refused_here(self):
        cases = [({}, "exactly one of dx_mm, dx_m"), ({"distance": 400.0}, "unknown key 'distance'"),
                 ({"dx_mm": 100.0, "strict": False}, "'strict' is set by the caller"),
                 ({"dx_mm": 100.0, "dry_run": True}, "'dry_run' is set by the caller"),
                 ({"dx_mm": True}, "'dx_mm' must be a number"), ({"dx_mm": float("nan")}, "'dx_mm' must be a number"),
                 ({"dx_mm": float("inf")}, "'dx_mm' must be a number"), ({"dx_mm": "200"}, "'dx_mm' must be a number"),
                 ({"dx_mm": 100.0, "dx_m": 0.1}, "exactly one of dx_mm, dx_m"),
                 ({"dx_mm": 100.0, "speed_mmps": 50.0, "speed_mps": 0.05}, "at most one of speed_mmps, speed_mps"),
                 ({"dx_mm": 100.0, "standoff_mm": -1.0}, "must not be negative")]
        for args, needle in cases:
            out, _ = call("cartographer_check_command", {"kind": "move", "args": args}, cfg_for("/x"))
            self.assertFalse(out["valid"], args)
            self.assertIsNone(out["plan"])
            self.assertNotIn("queue", out)
            self.assertTrue(any(needle in e for e in out["errors"]), "%s -> %s" % (args, out["errors"]))

    def test_out_of_bounds_is_refused_by_the_planner_in_strict_mode(self):
        for kind, args in (("move", {"dx_mm": 5000.0}), ("move", {"dx_mm": -151.0}), ("rotate", {"angle_deg": 361.0}),
                           ("head", {"head_deg": 46.0}), ("lift", {"lift_norm": 1.5}), ("move", {"dx_mm": 100.0, "speed_mmps": 500.0})):
            out, _ = call("cartographer_check_command", {"kind": kind, "args": args}, cfg_for("/x"))
            self.assertFalse(out["valid"], (kind, args))
            self.assertTrue(any("bounds_exceeded" in e for e in out["errors"]), out["errors"])
        edge, _ = call("cartographer_check_command", {"kind": "move", "args": {"dx_mm": 600.0}}, cfg_for("/x"))
        self.assertTrue(edge["valid"])

    def test_the_stand_off_can_be_raised_and_not_lowered(self):
        default = MOVEMENT.DEFAULT_FORWARD_STANDOFF_MM
        for value in (0.0, default - 1.0):
            out, _ = call("cartographer_check_command", {"kind": "move", "args": {"dx_mm": 100.0, "standoff_mm": value}}, cfg_for("/x"))
            self.assertFalse(out["valid"], value)
            self.assertTrue(any("bounds_exceeded" in e for e in out["errors"]), out["errors"])
        for value in (default, default + 40.0):
            out, _ = call("cartographer_check_command", {"kind": "move", "args": {"dx_mm": 100.0, "standoff_mm": value}}, cfg_for("/x"))
            self.assertTrue(out["valid"], out["errors"])
            self.assertEqual(out["plan"]["normalized"]["standoff_mm"], value)
            self.assertEqual(out["warnings"], [])

    def test_batches(self):
        good = {"directives": [{"verb": "rotate", "args": {"angle_deg": 90.0}}, {"verb": "move", "args": {"dx_mm": 180.0}}]}
        out, _ = call("cartographer_check_command", {"kind": "directives", "args": good}, cfg_for("/x"))
        self.assertTrue(out["valid"], out["errors"])
        self.assertEqual(out["plan"]["step_count"], 2)
        self.assertEqual(out["queue"]["record"]["expected_delta"], {})
        bad_steps = [[], [{"verb": "move"}], [{"verb": "scan", "args": {}}], [{"verb": "move", "args": {"mm": 5.0}}],
                     [{"verb": "move", "args": {"dx_mm": 5.0}, "strict": False}], ["move"],
                     [{"verb": "move", "args": {"dx_mm": 500.0}}] * 3, [{"verb": "halt", "args": {}}] * 17,
                     [{"verb": "move", "args": {"dx_mm": 5000.0}}]]
        for steps in bad_steps:
            out, _ = call("cartographer_check_command", {"kind": "directives", "args": {"directives": steps}}, cfg_for("/x"))
            self.assertFalse(out["valid"], steps)

    def test_no_planner_is_never_a_pass_for_a_planned_kind(self):
        blind = cfg_for("/x", movement=None)
        out, _ = call("cartographer_check_command", {"kind": "move", "args": {"dx_mm": 100.0}}, blind)
        self.assertFalse(out["valid"])
        self.assertTrue(any("planner is unavailable" in e for e in out["errors"]))
        probe, _ = call("cartographer_check_command", {"kind": "probe", "args": {}}, blind)
        self.assertTrue(probe["valid"])
        self.assertIn("no offline planner", probe["checked_against"])
        gram, _ = call("cartographer_grammar", {}, blind)
        self.assertTrue(gram["limits_source"].startswith("unavailable"))
        self.assertEqual(gram["limits"], {})

    def test_where_a_kind_is_legal(self):
        turn, _ = call("cartographer_check_command", {"kind": "turn", "args": {"angle_deg": 45.0}}, cfg_for("/x"))
        self.assertTrue(turn["valid"])
        self.assertIsNone(turn["queue"]["record"])
        self.assertIn("Use rotate", turn["queue"]["why"])
        self.assertIn("dry_run_argv", turn)
        battery, _ = call("cartographer_check_command", {"kind": "read-battery", "args": {}}, cfg_for("/x"))
        self.assertIsNone(battery["queue"]["record"])
        self.assertNotIn("dry_run_argv", battery)
        unknown, _ = call("cartographer_check_command", {"kind": "teleport", "args": {}}, cfg_for("/x"))
        self.assertFalse(unknown["valid"])
        self.assertEqual(unknown["legal_on"], [])

    def test_dry_run_argv_is_a_list_that_carries_strict_and_dry_run(self):
        out, _ = call("cartographer_check_command", {"kind": "move", "args": {"dx_mm": 100.0}}, cfg_for("/x"))
        argv = out["dry_run_argv"]
        self.assertEqual(argv[:4], ["python3", "skills/_vector_drive.py", "--kind", "move"])
        self.assertEqual(json.loads(argv[argv.index("--args") + 1]), {"dx_mm": 100.0})
        self.assertIn("--strict", argv)
        self.assertIn("--dry-run", argv)
        self.assertNotIn("--via-vector", argv)

    def test_grammar_tool(self):
        whole, _ = call("cartographer_grammar", {}, cfg_for("/x"))
        refusal = [h for h in whole["known_hazards"] if "does not grant SDK control" in h]
        self.assertEqual(len(refusal), 1)
        self.assertIn("moved away from the edge by hand", refusal[0])
        self.assertEqual(len(whole["known_hazards"]), len(grammar.KNOWN_HAZARDS))
        self.assertEqual(sorted(k["kind"] for k in whole["kinds"]), sorted(grammar.KINDS))
        self.assertEqual(whole["limits"]["MAX_FORWARD_DIRECTIVE_MM"], MOVEMENT.MAX_FORWARD_DIRECTIVE_MM)
        self.assertEqual(set(v["category"] for v in whole["stop_reasons"].values()) - CATEGORIES, set())
        one, _ = call("cartographer_grammar", {"kind": "move"}, cfg_for("/x"))
        self.assertEqual([k["kind"] for k in one["kinds"]], ["move"])
        self.assertNotIn("stop_reasons", one)
        _bad, is_err = call("cartographer_grammar", {"kind": "teleport"}, cfg_for("/x"))
        self.assertTrue(is_err)


class ReaderChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "cartographer"
        self.root.mkdir()
        self.mission = make_mission(self.root)
        self.cfg = cfg_for(self.root)

    def tearDown(self):
        self.tmp.cleanup()

    def test_mission_is_parsed(self):
        out, is_err = call("cartographer_mission", {"mission_id": self.mission.name}, self.cfg)
        self.assertFalse(is_err)
        self.assertEqual(out["robot"], "vector-demo")
        self.assertFalse(out["can_send_command"])
        self.assertEqual(out["why_not"], ["command cmd-0003 has been sent and its result has not been read yet"])
        self.assertAlmostEqual(out["budgets"]["distance_m"]["remaining"], 36.2)
        cmds = out["recent_commands"]["newest_last"]
        self.assertEqual([c["stage"] for c in cmds], ["acknowledged", "acknowledged", "sent, awaiting the robot"])
        self.assertEqual(cmds[1]["result"]["category"], "blocked")
        self.assertEqual(cmds[1]["result"]["moved"]["forward_m"], 0.14)
        self.assertEqual(out["recent_commands"]["unparsed_lines"], 1)
        self.assertEqual(out["recent_events"]["newest_last"][0]["phase"], "plan")

    def test_can_send_reasons(self):
        free = make_mission(self.root, name="m-free", awaiting=False)
        out, _ = call("cartographer_mission", {"mission_id": free.name}, self.cfg)
        self.assertTrue(out["can_send_command"])
        self.assertEqual(out["why_not"], [])
        state = json.loads((free / "state.json").read_text())
        state.update({"paused": True, "status": "aborted"})
        state["budgets"]["collisions"]["used"] = 3
        (free / "state.json").write_text(json.dumps(state))
        out, _ = call("cartographer_mission", {"mission_id": free.name}, self.cfg)
        self.assertFalse(out["can_send_command"])
        self.assertEqual(len(out["why_not"]), 3)

    def test_frontier_bearing_is_relative_to_the_robot_heading(self):
        out, _ = call("cartographer_mission", {"mission_id": self.mission.name}, self.cfg)
        first = out["frontiers"]["largest"][0]
        self.assertEqual((first["centroid_m"], first["size_cells"]), ([0.0, 1.0], 9))
        self.assertAlmostEqual(first["distance_m"], 1.0)
        self.assertAlmostEqual(first["bearing_deg"], 90.0)
        turned = make_mission(self.root, name="m-turned", theta=math.pi / 2)
        out, _ = call("cartographer_mission", {"mission_id": turned.name}, self.cfg)
        self.assertAlmostEqual(out["frontiers"]["largest"][0]["bearing_deg"], 0.0)
        self.assertAlmostEqual(out["frontiers"]["largest"][1]["bearing_deg"], -90.0)

    def test_mission_id_cannot_leave_the_root(self):
        outside = Path(self.tmp.name) / "outside"
        make_mission(Path(self.tmp.name), name="outside")
        self.assertTrue((outside / "state.json").is_file())
        os.symlink(outside, self.root / "link-out")
        (self.root / "not-a-mission").mkdir()
        for bad in ("../outside", "link-out", "missing", "not-a-mission", "", "a/b", ".", None, 7):
            out, is_err = call("cartographer_mission", {"mission_id": bad}, self.cfg)
            self.assertTrue(is_err, bad)
            self.assertIn("error", out)
            out, is_err = call("cartographer_map", {"mission_id": bad}, self.cfg)
            self.assertTrue(is_err, bad)

    def test_missions_list(self):
        (self.root / "AGENTS.md").write_text("not a mission")
        (self.root / "broken").mkdir()
        (self.root / "broken" / "state.json").write_text("{not json")
        out, _ = call("cartographer_missions", {}, self.cfg)
        self.assertEqual([m["mission_id"] for m in out["missions"]], ["broken", self.mission.name])
        self.assertEqual(out["unreadable"], 1)
        gone, _ = call("cartographer_missions", {}, cfg_for(Path(self.tmp.name) / "nope"))
        self.assertFalse(gone["root_exists"])

    def test_oversized_and_malformed_state_are_errors(self):
        (self.mission / "state.json").write_text("[1, 2]")
        _out, is_err = call("cartographer_mission", {"mission_id": self.mission.name}, self.cfg)
        self.assertTrue(is_err)
        (self.mission / "state.json").write_text(json.dumps({"pad": "x" * (reader.MAX_JSON_BYTES + 1)}))
        out, is_err = call("cartographer_mission", {"mission_id": self.mission.name}, self.cfg)
        self.assertTrue(is_err)
        self.assertIn("too large", out["error"])

    def test_queue_tail_reads_the_newest_records(self):
        lines = [json.dumps({"id": "cmd-%04d" % i, "kind": "move", "args": {"dx_mm": 10.0}, "sent": False, "ack": None}) for i in range(400)]
        (self.mission / "cmd-queue.jsonl").write_text("\n".join(lines) + "\n")
        out, _ = call("cartographer_mission", {"mission_id": self.mission.name, "commands": 3}, self.cfg)
        self.assertEqual([c["id"] for c in out["recent_commands"]["newest_last"]], ["cmd-0397", "cmd-0398", "cmd-0399"])
        self.assertEqual(out["recent_commands"]["newest_last"][0]["stage"], "queued")

    def test_map_as_text(self):
        out, is_err = call("cartographer_map", {"mission_id": self.mission.name, "cols": 8}, self.cfg)
        self.assertFalse(is_err)
        self.assertEqual(out["cells"], {"occupied": 2, "free": 8, "unknown": 6})
        self.assertEqual((out["width_cells"], out["height_cells"], out["resolution_m"]), (4, 4, 0.5))
        self.assertEqual(sum(out["cells"].values()), 16)
        self.assertTrue(out["robot_marked"])
        self.assertEqual("".join(out["picture"]).count("R"), 1)
        (self.mission / "grid.pgm").write_bytes(b"P5\n4 4\n255\n\x00\x00")
        short, _ = call("cartographer_map", {"mission_id": self.mission.name}, self.cfg)
        self.assertFalse(short["available"])
        (self.mission / "grid.pgm").unlink()
        none, _ = call("cartographer_map", {"mission_id": self.mission.name}, self.cfg)
        self.assertEqual(none, {"available": False, "why": "grid.pgm is missing"})

    def test_map_picture_rows_follow_the_image(self):
        bands = [[0] * 8, [254] * 8, [205] * 8, [254] * 8]
        write_pgm(self.mission / "grid.pgm", [row for band in bands for row in (band, band)])
        (self.mission / "grid.yaml").unlink()
        out, _ = call("cartographer_map", {"mission_id": self.mission.name, "cols": 8}, self.cfg)
        self.assertEqual(out["picture"], ["########", "........", "????????", "........"])
        self.assertIn("1 x 2 cells", out["picture_scale"])
        self.assertFalse(out["robot_marked"])


class RealMissionChecks(unittest.TestCase):
    """A mission written by Cartographer itself (tests/smoke/cartographer-vector-frontier.sh, mock driver,
    2026-11-14). Its files are unedited except that the robot's serial number is blanked in state.json."""

    ROOT = HERE / "fixtures" / "real-mission"
    NAME = "2026-11-14-0027-demo-bench"

    def test_mission_reads_as_cartographer_wrote_it(self):
        out, is_err = call("cartographer_mission", {"mission_id": self.NAME}, cfg_for(self.ROOT))
        self.assertFalse(is_err, out)
        self.assertEqual((out["robot"], out["status"], out["can_send_command"], out["why_not"]),
                         ("DEMO", "done", False, ["the mission is done"]))
        cmds = out["recent_commands"]["newest_last"]
        self.assertEqual([(c["id"], c["stage"], c["ack"]["reason"], c["result"]["category"]) for c in cmds],
                         [("cmd-0001", "acknowledged", "completed", "completed"), ("cmd-0002", "acknowledged", "completed", "completed")])
        self.assertEqual((out["recent_commands"]["unparsed_lines"], out["recent_events"]["unparsed_lines"]), (0, 0))
        self.assertEqual(out["frontiers"]["count"], 0)

    def test_cell_counts_agree_with_cartographers_own(self):
        state = json.loads((self.ROOT / self.NAME / "state.json").read_text())
        out, _ = call("cartographer_map", {"mission_id": self.NAME}, cfg_for(self.ROOT))
        self.assertEqual((out["cells"]["occupied"], out["cells"]["free"]),
                         (state["map_stats"]["occ_cells"], state["map_stats"]["free_cells"]))
        self.assertEqual(out["cells"]["occupied"] + out["cells"]["free"], state["map_stats"]["known_cells"])
        self.assertTrue(out["robot_marked"])

    def test_commands_cartographer_queued_are_valid_here(self):
        lines = (self.ROOT / self.NAME / "cmd-queue.jsonl").read_text().splitlines()
        self.assertEqual(len(lines), 2)
        for line in lines:
            rec = json.loads(line)
            out, _ = call("cartographer_check_command", {"kind": rec["kind"], "args": rec["args"]}, cfg_for("/x"))
            self.assertTrue(out["valid"], out["errors"])
            self.assertEqual(out["warnings"], ["'dtheta_deg' is ignored by the driver for move; leave it out"])
            self.assertAlmostEqual(out["queue"]["record"]["expected_delta"]["x"], rec["expected_delta"]["x"], places=4)

    def test_a_turn_smuggled_into_a_move_is_refused(self):
        for value in (90.0, -1, True, "0"):
            out, _ = call("cartographer_check_command", {"kind": "move", "args": {"dx_m": 0.1, "dtheta_deg": value}}, cfg_for("/x"))
            self.assertFalse(out["valid"], value)
            self.assertTrue(any("dropped silently" in e for e in out["errors"]), out["errors"])
        other, _ = call("cartographer_check_command", {"kind": "head", "args": {"head_deg": 0.0, "dtheta_deg": 0}}, cfg_for("/x"))
        self.assertFalse(other["valid"])


class ResultChecks(unittest.TestCase):
    def test_shapes(self):
        cases = [
            ({"ok": True, "dry_run": True, "verb": "move", "clamped": False}, "plan", "planned"),
            ({"ok": False, "dry_run": True, "stopped_reason": "bounds_exceeded"}, "plan", "rejected"),
            ({"ok": True, "stopped_reason": "goal", "actual_delta": {"x": 0.1, "y": 0.0, "theta": 0.0}, "motion_result": {}}, "motion_result", "completed"),
            ({"ok": False, "stopped_reason": "cliff", "actual_delta": {"x": 0.02, "y": 0.0, "theta": 0.0}, "motion_result": {}}, "motion_result", "hazard"),
            ({"ok": False, "stopped_reason": "proximity_standoff", "steps_completed": 1, "results": [{}, {}]}, "batch_result", "blocked"),
            ({"ok": False, "error": "drive_lock_timeout"}, "status", "busy"),
            ({"ok": False, "stopped_reason": "unknown_verb:teleport", "dry_run": True}, "plan", "rejected"),
            ({"ok": False, "stopped_reason": "sunspots"}, "status", "unknown"),
            ({"hello": "world"}, "unrecognised", "unknown"),
        ]
        for obj, shape, category in cases:
            out, _ = call("cartographer_parse_result", {"result": obj}, cfg_for("/x"))
            self.assertEqual((out["shape"], out["category"]), (shape, category), obj)
        text, _ = call("cartographer_parse_result", {"result": json.dumps(cases[2][0])}, cfg_for("/x"))
        self.assertEqual(text["moved"]["forward_m"], 0.1)
        for bad in ("{not json", 7, None, []):
            _out, is_err = call("cartographer_parse_result", {"result": bad}, cfg_for("/x"))
            self.assertTrue(is_err, bad)

    def test_a_plan_is_never_reported_as_motion(self):
        out, _ = call("cartographer_parse_result", {"result": {"ok": True, "dry_run": True, "verb": "move"}}, cfg_for("/x"))
        self.assertEqual(out["robot"], "was not contacted")
        self.assertNotIn("moved", out)

    def test_every_reason_has_a_known_category(self):
        for reason in reader.STOP_REASONS:
            self.assertIn(reader.interpret_stop(reason)["category"], CATEGORIES, reason)
        self.assertEqual(reader.interpret_stop(None)["category"], "unknown")

    def test_turned_angle_is_reported_in_degrees(self):
        out = reader.parse_result({"ok": True, "stopped_reason": "goal", "actual_delta": {"x": 0.0, "y": 0.0, "theta": math.pi / 2}})
        self.assertAlmostEqual(out["moved"]["turned_deg"], 90.0)


class TransportChecks(unittest.TestCase):
    def test_dispatcher(self):
        cfg = cfg_for("/x")
        init = svc.dispatch({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-03-26"}}, cfg)
        self.assertEqual(init["result"]["protocolVersion"], "2025-03-26")
        self.assertEqual(init["result"]["serverInfo"]["name"], "cartographer-mcp")
        self.assertIsNone(svc.dispatch({"jsonrpc": "2.0", "method": "notifications/initialized"}, cfg))
        tools = svc.dispatch({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}, cfg)["result"]["tools"]
        self.assertEqual(sorted(t["name"] for t in tools), sorted(svc.HANDLERS))
        self.assertTrue(all(t["inputSchema"]["type"] == "object" and t["description"] for t in tools))
        self.assertEqual(svc.dispatch({"jsonrpc": "2.0", "id": 3, "method": "nope"}, cfg)["error"]["code"], -32601)
        self.assertIsNone(svc.dispatch({"jsonrpc": "2.0", "method": "nope"}, cfg))
        self.assertIsNone(svc.dispatch(["not", "an", "object"], cfg))
        _out, is_err = call("no_such_tool", {}, cfg)
        self.assertTrue(is_err)
        bad = svc.dispatch({"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "cartographer_grammar", "arguments": [1]}}, cfg)
        self.assertTrue(bad["result"]["isError"])

    def test_stdio_end_to_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "cartographer"
            root.mkdir()
            mission = make_mission(root)
            lines = [
                {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2024-11-05"}},
                {"jsonrpc": "2.0", "method": "notifications/initialized"},
                {"jsonrpc": "2.0", "id": 2, "method": "tools/list"},
                {"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "cartographer_mission", "arguments": {"mission_id": mission.name}}},
                {"jsonrpc": "2.0", "id": 4, "method": "tools/call", "params": {"name": "cartographer_check_command", "arguments": {"kind": "move", "args": {}}}},
            ]
            stdin = "\n".join(json.dumps(m) for m in lines[:2]) + "\nthis is not json\n\n" + "\n".join(json.dumps(m) for m in lines[2:]) + "\n"
            env = dict(os.environ, CARTOGRAPHER_ROOT=str(root), CARTOGRAPHER_MOVEMENT_PKG=MOVEMENT_PKG)
            proc = subprocess.run([sys.executable, str(SKILL / "cartographer-mcp.py")], input=stdin, capture_output=True, text=True, timeout=60, env=env)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        replies = [json.loads(line) for line in proc.stdout.splitlines()]
        self.assertEqual([r["id"] for r in replies], [1, 2, 3, 4])
        self.assertEqual(json.loads(replies[2]["result"]["content"][0]["text"])["robot"], "vector-demo")
        self.assertFalse(json.loads(replies[3]["result"]["content"][0]["text"])["valid"])
        self.assertIn("planner ok", proc.stderr)
        self.assertIn("skipped a malformed line", proc.stderr)

    def test_the_server_has_no_way_to_act(self):
        """A tripwire, not a proof: the modules import nothing that runs a program or opens a socket, and write no file."""
        for name in ("cartographer_grammar.py", "cartographer_read.py", "cartographer_service.py", "cartographer-mcp.py"):
            text = (SKILL / name).read_text()
            imports = set(re.findall(r"^(?:import|from) ([a-z_.]+)", text, re.M))
            self.assertEqual(imports & {"subprocess", "socket", "shutil", "urllib", "http", "requests", "ctypes"}, set(), name)
            self.assertIsNone(re.search(r"os\.(system|popen|exec|spawn|remove|unlink|rename|replace|mkdir|makedirs)", text), name)
            modes = re.findall(r"open\([^)]*?,\s*\"([a-z+]+)\"", text)
            self.assertTrue(all(mode == "rb" for mode in modes), "%s opens a file in %s" % (name, modes))


if __name__ == "__main__":
    unittest.main()
