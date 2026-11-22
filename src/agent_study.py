"""SkillMem Run 3 (Run 2 + 2500-tick budget + real error messages): does a robot's skill memory help, hurt, or confuse an LLM that writes robot programs?

Changes from the pilot (agent_pilot.py, whose memory/run helpers this file reuses):
  1. The sandbox rules (ASPIRE's program validator) are in the system prompt for EVERY condition. In the
     pilot only condition D happened to retrieve them, which confounded the comparison.
  2. Up to 3 repair rounds (v2..v4) instead of 1. Each version runs on the 3 dev seeds; we stop as soon
     as a version passes all 3. v1 (one-shot) and the final version are both scored on 5 held-out seeds.
  3. Each repair also shows Claude the last camera frame (agentview) of the first failed dev seed.
  4. MCP connectors are disabled (--strict-mcp-config): fewer tokens, and the agent can't see other tools.

Usage: python agent_study.py --tasks libero_goal_swap:8,libero_goal_swap:1 --conds A,B,C,D --out DIR
Resumable through DIR/state.json.
"""
import argparse
import base64
import io
import json
import os
import pathlib
import re
import shutil
import subprocess
import time

from agent_pilot import (DEV_SEEDS, HELDOUT_SEEDS, REPO, build_memory, feedback, run_program, task_info,
                         task_query)

DEADLINE = float(os.environ.get("SKILLMEM_DEADLINE", "inf"))  # unix time: stop cleanly before the GPU quota ends
RETRY_WAITS = [30, 120] + [600] * 8  # transient errors and 5-hour-limit resets
MAX_REPAIRS = 5  # Run 3: 3 -> 5 repair rounds (v2..v6)

SYSTEM = """You write robot control programs for ASPIRE's code-as-policy harness: a Franka Panda arm in LIBERO-Pro (MuJoCo).
The program is checked by a sandbox validator before it runs. Violations reject the whole program:
- No name or attribute may start with an underscore (no `_helper`, no `obj.__dict__`).
- Not allowed: class, with, global, nonlocal, del, async/await, `from x import y`. The only import allowed is `import math` (no alias).
- Only these builtins exist: Exception ValueError TypeError abs all any bool dict enumerate float int len list max min print range reversed round set sorted str sum tuple zip. (No getattr, hasattr, isinstance, type, open, map, filter.)
- Contract constructors available by name: ArmCommand MotionStrategy Pose RobotAction SynchronizedTrajectory Trajectory. Everything else comes from the documented API; use only fields and functions the API reference documents.
Task rules:
- Perceive the scene at runtime (localize objects with the segmentation tools). Never hard-code object coordinates: object positions differ on every seed.
- The episode stops at 2500 control ticks (20 Hz). Every control tick spent waiting counts, including the ticks a failed move burns. After the episode ends, every further call fails with "executing action in terminated episode".
- The arm moves about 0.15 rad/s per joint, so large rotations cost many ticks. Keep the gripper pointing down and avoid big wrist turns unless the task needs them.
- A move_to_pose "timeout" with a large joint_error_rad (above about 0.05) means the arm is BLOCKED, not slow: the target is inside the table or an object, or out of reach. Retrying the same target wastes ticks. Raise it, approach from above, or change it.
- Moves are straight joint interpolation with NO obstacle avoidance. cuRobo is not installed, so never request it.
  The scene has tall obstacles (wine rack, cabinet, bottle). Never move sideways low: first go straight up to
  z of about 0.30 m (0.28-0.35, robot_base), travel at that height in short hops, then descend vertically over the target.
  The hand housing is much wider than the fingers: if a descent is blocked by a neighbour, choose another contact point or yaw the gripper 90 degrees.
  If a hop is blocked, back up, then go higher or around the obstacle.
- `math` is not predefined: write `import math` if you use it.
- IMPORTANT: move_to_pose fails with status "timeout" whenever the final joint error is above 0.01 rad, whatever `tolerance` you pass.
  An error below 0.05 rad means the arm has practically ARRIVED: continue the plan, don't retry or abort. Test it like this:
  `near = r.ok or (r.error is not None and r.error.details.get("joint_error_rad", 9) < 0.05)`
- Sanity-check every localization: small objects are sometimes confused with the robot's own gripper or with other objects.
  A stove knob or a plate is near the table (z < 0.15) and a knob is next to the stove. Try other queries or cameras if not.
- Check every motion result (with the near test above) and adapt on a real failure. Don't run the rest of the plan blind.
- Set a top-level `result` dict recording what each stage achieved (e.g. {"localized": True, "grasped": False, ...}).
Reply with exactly one ```python code block containing the complete program and nothing else."""


def png_b64(path, max_w=640):
    from PIL import Image
    img = Image.open(path).convert("RGB")
    if img.width > max_w:
        img = img.resize((max_w, round(img.height * max_w / img.width)))
    buf = io.BytesIO(); img.save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode()


def claude(prompt, model, image=None):
    content = []
    if image:
        content.append({"type": "image", "source": {"type": "base64", "media_type": "image/png", "data": png_b64(image)}})
    content.append({"type": "text", "text": prompt})
    msg = {"type": "user", "message": {"role": "user", "content": content}}
    cmd = ["claude", "-p", "--model", model, "--tools", "", "--strict-mcp-config", "--system-prompt", SYSTEM,
           "--input-format", "stream-json", "--output-format", "stream-json", "--verbose", "--no-session-persistence"]
    t0 = time.time()
    r = subprocess.run(cmd, input=json.dumps(msg) + "\n", capture_output=True, text=True, timeout=600)
    finals = [json.loads(l) for l in r.stdout.splitlines() if l.startswith("{") and '"type":"result"' in l.replace(" ", "")]
    if not finals:
        raise RuntimeError(f"claude gave no result (rc={r.returncode}): {r.stdout[-400:]} {r.stderr[-400:]}")
    j = finals[-1]
    if j.get("is_error"):
        raise RuntimeError(f"claude error: {str(j.get('result'))[:400]}")
    blocks = re.findall(r"```python\n(.*?)```", j.get("result", ""), flags=re.S)
    u = j.get("usage", {})
    usage = {"input": u.get("input_tokens", 0), "cache_read": u.get("cache_read_input_tokens", 0),
             "cache_write": u.get("cache_creation_input_tokens", 0), "output": u.get("output_tokens", 0),
             "cost_usd_equiv": j.get("total_cost_usd"), "seconds": round(time.time() - t0, 1), "image": bool(image)}
    prog = blocks[-1] if blocks else ""
    if re.search(r"\bmath\.", prog) and not re.search(r"^import math\b", prog, flags=re.M):
        prog = "import math\n" + prog
    return prog, usage


def claude_retry(prompt, model, image):
    for wait in RETRY_WAITS + [None]:
        try:
            prog, usage = claude(prompt, model, image)
            if prog.strip():
                return prog, usage
            err = RuntimeError("reply had no python block")
        except Exception as e:
            err = e
        if wait is None or time.time() + wait > DEADLINE:
            raise err
        print(f"[retry] claude: {str(err)[:200]}; waiting {wait}s", flush=True)
        time.sleep(wait)


def last_frame(run_root):
    """agentview.png of the last keyframe ASPIRE recorded for this run (the scene after the final motion)."""
    frames = sorted(pathlib.Path(run_root).rglob("media/keyframes/*/agentview.png"))
    return str(frames[-1]) if frames else None


def keep_frame(src, out, name):
    """Copy the frame into out/frames/ (checkpoints skip the big keyframe folders)."""
    if not src or not pathlib.Path(src).exists():
        return None
    dst = out / "frames" / f"{name}.png"; dst.parent.mkdir(exist_ok=True)
    shutil.copy(src, dst)
    return str(dst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tasks", required=True)
    ap.add_argument("--conds", default="A,B,C,D")
    ap.add_argument("--out", required=True)
    ap.add_argument("--model", default="sonnet")
    ap.add_argument("--max-calls", type=int, default=120)
    args = ap.parse_args()
    out = pathlib.Path(args.out); out.mkdir(parents=True, exist_ok=True)
    sp = out / "state.json"
    state = json.load(open(sp)) if sp.exists() else {}
    def save():  # atomic, so a checkpoint never packs a half-written state
        tmp = out / "state.json.tmp"
        tmp.write_text(json.dumps(state, indent=1))
        os.replace(tmp, sp)
    api = open(f"{REPO}/docs/api-reference.md").read()
    for v in state.values():  # move frames recorded before this fix into out/frames/
        for k, rs in (v.items() if isinstance(v, dict) else []):
            if isinstance(rs, list):
                for r in rs:
                    if isinstance(r, dict) and r.get("frame") and "/frames/" not in r["frame"]:
                        r["frame"] = keep_frame(r["frame"], out, f"old_{abs(hash(r['frame']))}")
    save()
    calls = 0
    versions = [f"v{i}" for i in range(1, MAX_REPAIRS + 2)]

    def evaluate(st, ver, phase, seeds, tag, suite, tid, videos):
        k = f"{ver}_{phase}"
        st.setdefault(k, [])
        done = {r["seed"] for r in st[k]}
        prog_path = out / "programs" / f"{tag}_{ver}.py"; prog_path.parent.mkdir(exist_ok=True)
        prog_path.write_text(st[f"{ver}_program"] or "result = {'empty_program': True}\n")
        for s in seeds:
            if s in done:
                continue
            if time.time() > DEADLINE:
                save(); print("[stop] time budget used up; rerun to resume", flush=True); raise SystemExit(0)
            root = out / "runs" / tag / ver / f"{phase}_s{s}"
            r = run_program(prog_path, suite, tid, s, root, videos)
            r["frame"] = keep_frame(last_frame(root), out, f"{tag}_{ver}_{phase}_s{s}")
            for kind in ("keyframes", "overlays"):  # gigabytes; the last frame was copied above
                for d in list(root.rglob(f"media/{kind}")):
                    shutil.rmtree(d, ignore_errors=True)
            st[k].append(r); save()
            print(f"[run] {tag} {ver} {phase} seed {s}: success={r['success']} ({r.get('termination')}, {r.get('wall_s')}s)", flush=True)
        st[f"{k}_success"] = sum(r["success"] for r in st[k]); save()

    for task_ref in args.tasks.split(","):
        suite, tid = task_ref.split(":")
        if f"info:{task_ref}" not in state:
            state[f"info:{task_ref}"] = task_info(task_ref); save()
        info = state[f"info:{task_ref}"]
        query = task_query(info["language"], info["bddl"])
        for cond in args.conds.split(","):
            key, tag = f"{task_ref}|{cond}", f"{suite}_{tid}_{cond}"
            st = state.setdefault(key, {})
            if st.get("finished"):
                continue
            mem, mem_index = build_memory(cond, query)
            st["memory_chars"], st["memory_sections"] = len(mem), mem_index
            context = (f"# API reference\n{api}\n\n" + (f"# Skill library (lessons from earlier robot tasks)\n{mem}\n\n" if mem else "")
                       + f"# Task\nsuite={suite} task_id={tid}\nlanguage: {info['language']}\nBDDL scene definition:\n{info['bddl']}\n")
            prev = None
            for ver in versions:
                if f"{ver}_program" not in st:
                    if calls >= args.max_calls:
                        print(f"[stop] reached --max-calls {args.max_calls}; rerun to resume", flush=True); save(); return
                    image = None
                    if prev is None:
                        prompt = context + "\nWrite the program for this task."
                    else:
                        failed = [r for r in st[f"{prev}_dev"] if not r["success"]]
                        image = failed[0].get("frame") if failed else None
                        if image and not pathlib.Path(image).exists():
                            image = None
                        prompt = (context + f"\n# Your previous program ({prev})\n```python\n{st[f'{prev}_program']}\n```\n"
                                  f"\n# What happened on the development seeds\n{feedback(st[f'{prev}_dev'])}\n"
                                  + ("\nThe attached image is the scene camera (agentview) after the last motion of the first failed seed.\n" if image else "")
                                  + "\nFix the program. Return the complete corrected program.")
                    try:
                        prog, usage = claude_retry(prompt, args.model, image)
                    except Exception as e:
                        print(f"[stop] {key} {ver}: {e}", flush=True); save(); return  # e.g. usage limit: rerun later
                    calls += 1
                    st[f"{ver}_program"], st[f"{ver}_usage"] = prog, usage; save()
                    print(f"[gen] {key} {ver}: {len(prog)} chars, usage {usage}", flush=True)
                evaluate(st, ver, "dev", DEV_SEEDS, tag, suite, tid, videos=False)
                prev = ver
                if st[f"{ver}_dev_success"] == len(DEV_SEEDS):
                    break
            tried = [v for v in versions if f"{v}_dev_success" in st]
            prev = max(tried, key=lambda v: (st[f"{v}_dev_success"], int(v[1:])))
            st["final_version"] = prev
            evaluate(st, "v1", "heldout", HELDOUT_SEEDS, tag, suite, tid, videos=True)
            if prev != "v1":
                evaluate(st, prev, "heldout", HELDOUT_SEEDS, tag, suite, tid, videos=True)
            st["finished"] = True; save()
            print(f"[done] {key}: dev passes by version {[st.get(v + '_dev_success') for v in versions if v + '_dev_success' in st]}, "
                  f"v1 heldout {st['v1_heldout_success']}/5, final {prev} heldout {st[prev + '_heldout_success']}/5", flush=True)
    print("STUDY COMPLETE", flush=True)


if __name__ == "__main__":
    main()
