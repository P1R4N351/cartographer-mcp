#!/usr/bin/env python3
"""Break cartographer-mcp on purpose and check its tests notice.

Authored-by: CPCS. Run:  python3 skills/cartographer-mcp/tests/mutate.py
Each arm changes one line in a scratch copy, runs the suite, and restores the file.
An arm is CAUGHT only when the suite fails; the tests that failed are printed so a
catch for the wrong reason can be seen. A fault of this harness (a needle that does
not occur exactly once, a file left unchanged, a control that does not pass) is
reported as SETUP-ERROR and never as a verdict about the server. Exit 0 when the
control passes, every arm is caught and the neutral arm survives; 1 otherwise;
3 on a setup error.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SKILL = Path(__file__).resolve().parent.parent
WORKSPACE = SKILL.parent.parent
LINKS = ("vector-slam", "_cartographer_lib.sh", "draft-cartographer-vector-proprio.sh", "_vector_drive.py")
G, R, S = "cartographer_grammar.py", "cartographer_read.py", "cartographer_service.py"

ARMS = [
    ("the control-refusal hazard dropped", G, '    "At DEFAULT control priority Vector does not grant SDK control while its cliff flag is set and it is "',
     '    "At DEFAULT control priority Vector grants SDK control at an edge and it is "'),
    ("unknown argument key accepted", G, "        elif key not in fields:", "        elif False:"),
    ("strict or dry_run allowed inside args", G, "        if key in FORBIDDEN_KEYS:", "        if False:"),
    ("missing distance or angle accepted", G, "        if len(present) != 1:", "        if len(present) > 1:"),
    ("an ignored key accepted with any value", G, "            if _is_number(args[key]) and args[key] == 0:", "            if True:"),
    ("a boolean accepted as a number", G, "    if isinstance(value, bool) or not isinstance(value, (int, float)):",
     "    if not isinstance(value, (int, float)):"),
    ("planner run without strict", S, "            plan = movement.plan_directive(kind, args, strict=True)",
     "            plan = movement.plan_directive(kind, args, strict=False)"),
    ("batch planner run without strict", S, '            plan = movement.plan_directive_batch(args["directives"], strict=True)',
     '            plan = movement.plan_directive_batch(args["directives"], strict=False)'),
    ("no planner treated as a pass", S, '        return (None, "the driver\'s planner is unavailable, so the limits could not be checked")',
     "        return (None, None)"),
    ("dry-run argv loses --strict", S, 'json.dumps(cmd_args, sort_keys=True), "--strict", "--dry-run"]',
     'json.dumps(cmd_args, sort_keys=True), "--dry-run"]'),
    ("mission id may leave the root", R, "    if os.path.dirname(path) != real_root or not os.path.isdir(path):",
     "    if not os.path.isdir(path):"),
    ("unknown stop reason guessed as completed", R,
     '    return {"category": "unknown", "meaning": "%r is not a stop reason this server knows." % reason, "robot": "unknown"}',
     '    return {"category": "completed", "meaning": "%r is not a stop reason this server knows." % reason, "robot": "unknown"}'),
    ("a command still awaited is ignored", R, '    if isinstance(awaiting, dict) and awaiting.get("cmd_id"):', "    if False:"),
    ("frontier bearing ignores the heading", R, '            bearing = math.degrees(math.atan2(dy, dx) - pose["theta_rad"])',
     "            bearing = math.degrees(math.atan2(dy, dx))"),
    ("a dry-run plan reported like motion", R, '        out["robot"] = "was not contacted"\n', ""),
    ("unknown cells counted as occupied", R, "                block_occ = sum(1 for v in row if v <= 50)",
     "                block_occ = sum(1 for v in row if v <= 220)"),
    ("queue tail returns the oldest records", R, "    for line in [ln for ln in lines if ln.strip()][-limit:]:",
     "    for line in [ln for ln in lines if ln.strip()][:limit]:"),
    ("file size cap removed", R, "        if size > cap:", "        if False:"),
]
NEUTRAL = ("neutral wording change", G, '    """One argument field."""', '    """One argument field. """')


def build_copy(work):
    """A scratch workspace holding a copy of the skill and links to the files its tests pin against."""
    assert work and os.path.isdir(work)
    skills = Path(work) / "skills"
    skills.mkdir()
    shutil.copytree(SKILL, skills / "cartographer-mcp", ignore=shutil.ignore_patterns("__pycache__"))
    for name in LINKS:
        source = WORKSPACE / "skills" / name
        if not source.exists():
            return "missing %s" % source
        os.symlink(source, skills / name)
    return None


def run_suite(work):
    """(passed, ran, failing test names) for the suite in the scratch copy."""
    assert work and os.path.isdir(work)
    env = {k: v for k, v in os.environ.items() if not k.startswith("CARTOGRAPHER_")}
    proc = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "skills/cartographer-mcp/tests"],
                          cwd=work, capture_output=True, text=True, timeout=300, env=env)
    ran = re.search(r"^Ran (\d+) tests", proc.stderr, re.M)
    failing = sorted(set(re.findall(r"^(?:FAIL|ERROR): (\w+)", proc.stderr, re.M)))
    return (proc.returncode == 0, int(ran.group(1)) if ran else 0, failing)


def run_arm(work, arm):
    """'CAUGHT', 'SURVIVED' or 'SETUP-ERROR: ...' for one arm, with the failing tests."""
    assert len(arm) == 4
    _label, name, old, new = arm
    target = Path(work) / "skills" / "cartographer-mcp" / name
    pristine = target.read_text()
    if old == new or pristine.count(old) != 1:
        return ("SETUP-ERROR: needle occurs %d times" % pristine.count(old), [])
    mutated = pristine.replace(old, new)
    if mutated == pristine:
        return ("SETUP-ERROR: file unchanged", [])
    target.write_text(mutated)
    try:
        passed, ran, failing = run_suite(work)
    finally:
        target.write_text(pristine)
    if ran == 0:
        return ("SETUP-ERROR: the suite did not run", failing)
    return ("SURVIVED" if passed else "CAUGHT", failing)


def main():
    """Run the control, every arm and the neutral arm."""
    work = tempfile.mkdtemp(prefix="cartographer-mcp-mutate-")
    try:
        err = build_copy(work)
        if err:
            print("SETUP-ERROR: %s" % err)
            return 3
        passed, total, failing = run_suite(work)
        print("control: %s, %d tests%s" % ("pass" if passed else "FAIL", total, "" if passed else " %s" % failing))
        if not passed or total == 0:
            print("SETUP-ERROR: the unmodified copy does not pass, so no arm means anything")
            return 3
        caught = setup = 0
        for arm in ARMS:
            verdict, failing = run_arm(work, arm)
            caught += verdict == "CAUGHT"
            setup += verdict.startswith("SETUP-ERROR")
            print("%-42s %s  %s" % (arm[0], verdict, ", ".join(failing)))
        neutral, _failing = run_arm(work, NEUTRAL)
        print("%-42s %s (must survive)" % (NEUTRAL[0], neutral))
        again, total_again, _f = run_suite(work)
        print("restored: %s, %d tests" % ("pass" if again else "FAIL", total_again))
        print("caught %d of %d arms" % (caught, len(ARMS)))
        if setup or neutral != "SURVIVED" or not again or total_again != total:
            return 3
        return 0 if caught == len(ARMS) else 1
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
