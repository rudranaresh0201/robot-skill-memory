"""SkillMem pilot: does a robot's skill memory help, hurt, or confuse an LLM that writes robot programs?

For each task and each memory condition, Claude writes ONE program (single call, no tools). The program
runs on 3 development seeds; Claude gets ONE repair call with the failures; both versions are scored on
5 held-out seeds. Everything runs through ASPIRE's harness (cap-harness) on LIBERO-Pro.

Memory conditions (the only thing that changes between conditions):
  A  none      API reference only
  B  relevant  + ASPIRE's tabletop skill library (grasp / localize / transport / manipulation)
  C  bloated   + B + ASPIRE's BEHAVIOR (mobile humanoid) skills + its evosearch pipeline guide
  D  managed   the same pool as C, but a BM25 retriever keeps only the sections most relevant to the
               task, up to the size of B

Usage: python agent_pilot.py --tasks libero_goal_swap:8 --conds A,B,C,D --out /kaggle/working/run1
Resumable: every finished step is recorded in <out>/state.json and skipped on rerun.
"""
import argparse
import glob
import json
import math
import os
import pathlib
import re
import subprocess
import sys
import time
from collections import Counter

REPO = os.environ.get("ASPIRE_REPO", "/root/aspire")
CAP = f"{REPO}/.venv-libero/bin/cap-harness"
SK = f"{REPO}/.claude/skills"
RELEVANT = [f"{SK}/iterative-debugging/skills/{n}.md" for n in ("grasp", "localize", "transport", "manipulation")]
DISTRACT = sorted(glob.glob(f"{SK}/iterative-debugging/behavior/skills/*.md")) + [f"{SK}/evosearch/skills/evosearch-iteration.md"]
DEV_SEEDS, HELDOUT_SEEDS = [51, 52, 53], [6, 7, 8, 9, 10]
MAX_STEPS = int(os.environ.get("SKILLMEM_MAX_STEPS", "2500"))  # Run 3: 1000 -> 2500 (Run 2 episodes ran out of ticks)

EP_TIMEOUT = int(os.environ.get("SKILLMEM_EP_TIMEOUT", "700"))  # slowest Run 3 episode: 270 s
SYSTEM = """You write robot control programs for ASPIRE's code-as-policy harness: a Franka Panda arm in LIBERO-Pro (MuJoCo).
Rules for every program (violations make the run invalid):
- Use only the documented generated-program API. `import math` is the only import allowed; no NumPy, files, network, or simulator internals.
- Perceive the scene at runtime (e.g. localize objects with the SAM3 tools). Never hard-code object coordinates: every seed has a different layout.
- The episode stops at 1000 control ticks (20 Hz). Budget your motions.
- Set a top-level `result` dict recording what each stage achieved (e.g. {"localized": True, "grasped": False, ...}).
Reply with exactly one ```python code block containing the complete program and nothing else."""


# ---------------------------------------------------------------- memory
def strip_frontmatter(text):
    return re.sub(r"\A---\n.*?\n---\n", "", text, flags=re.S)


def sections(paths):
    """Split markdown files into '## ' sections: (source, title, text)."""
    out = []
    for p in paths:
        text = strip_frontmatter(open(p).read())
        parts = re.split(r"(?m)^(?=## )", text)
        for part in parts:
            if part.strip():
                title = part.strip().splitlines()[0][:80]
                out.append((os.path.relpath(p, SK), title, part.strip()))
    return out


def tokenize(s):
    return re.findall(r"[a-z]+", s.lower())


STOP = set("the a an of to in on and or for with is are be it this that from by as at into onto your you".split())


def task_query(language, bddl):
    """Retrieval query: the task sentence + object/fixture names from the BDDL (not its boilerplate)."""
    names = []
    for block in re.findall(r"\(:(?:objects|fixtures)(.*?)\)", bddl, flags=re.S):
        names += re.findall(r"([a-z_]+?)_\d+\s*-", block)
    return language + " " + " ".join(n.replace("_", " ") for n in names)


def bm25_rank(query, docs, k1=1.5, b=0.75):
    toks = [[w for w in tokenize(d) if w not in STOP] for d in docs]
    avg = sum(map(len, toks)) / max(1, len(toks))
    df = Counter(w for t in toks for w in set(t))
    n = len(docs)
    q = set(tokenize(query)) - STOP
    scores = []
    for t in toks:
        tf = Counter(t)
        s = 0.0
        for w in q:
            if w in tf:
                idf = math.log(1 + (n - df[w] + 0.5) / (df[w] + 0.5))
                s += idf * tf[w] * (k1 + 1) / (tf[w] + k1 * (1 - b + b * len(t) / avg))
        scores.append(s)
    return sorted(range(n), key=lambda i: -scores[i]), scores


def render(secs):
    return "\n\n".join(f"<!-- {src} -->\n{text}" for src, _, text in secs)


def build_memory(cond, query):
    rel, dis = sections(RELEVANT), sections(DISTRACT)
    if cond == "A":
        return "", []
    if cond == "B":
        return render(rel), [f"{s}|{t}" for s, t, _ in rel]
    pool = rel + dis
    if cond == "C":
        pool = sorted(pool, key=lambda x: x[0])  # deterministic, relevant and distractor files interleaved by path
        return render(pool), [f"{s}|{t}" for s, t, _ in pool]
    if cond == "D":
        budget = len(render(rel))
        # titles count three times: they say what a section is for
        order, scores = bm25_rank(query, [f"{t} {t} {t} {x}" for _, t, x in pool])
        picked, size = [], 0
        for i in order:
            if scores[i] <= 0:
                break  # keep only sections the task actually matches
            if picked and size + len(pool[i][2]) > budget:
                continue
            picked.append(pool[i]); size += len(pool[i][2])
        return render(picked), [f"{s}|{t}|{scores[pool.index(x)]:.1f}" for x in picked for s, t, _ in [x]]
    raise ValueError(cond)


# ---------------------------------------------------------------- task info
def task_info(task_ref):
    code = ("import json,sys\nfrom cap_harness.validation import load_libero_registry\n"
            f"m = load_libero_registry().resolve({task_ref!r})\n"
            "print(json.dumps({'language': m.language, 'task_name': m.task_name, 'bddl': open(m.bddl_path).read()}))")
    out = subprocess.run([f"{REPO}/.venv-libero/bin/python", "-c", code], capture_output=True, text=True, cwd=REPO)
    line = [l for l in out.stdout.splitlines() if l.startswith("{")]
    if not line:
        raise RuntimeError(f"task_info failed for {task_ref}: {out.stderr[-800:]}")
    return json.loads(line[-1])


# ---------------------------------------------------------------- claude
def claude(prompt, model):
    cmd = ["claude", "-p", "--model", model, "--tools", "", "--system-prompt", SYSTEM,
           "--output-format", "json", "--no-session-persistence"]
    t0 = time.time()
    r = subprocess.run(cmd, input=prompt, capture_output=True, text=True, timeout=1200)
    try:
        j = json.loads(r.stdout)
    except json.JSONDecodeError:
        raise RuntimeError(f"claude returned no JSON (rc={r.returncode}): {r.stdout[-500:]} {r.stderr[-500:]}")
    if j.get("is_error"):
        raise RuntimeError(f"claude error: {str(j.get('result'))[:500]}")
    text = j.get("result", "")
    blocks = re.findall(r"```python\n(.*?)```", text, flags=re.S)
    u = j.get("usage", {})
    usage = {"input": u.get("input_tokens", 0), "cache_read": u.get("cache_read_input_tokens", 0),
             "cache_write": u.get("cache_creation_input_tokens", 0), "output": u.get("output_tokens", 0),
             "cost_usd_equiv": j.get("total_cost_usd"), "seconds": round(time.time() - t0, 1)}
    return (blocks[-1] if blocks else ""), usage


# ---------------------------------------------------------------- run + feedback
def run_program(prog_path, suite, tid, seed, out_root, videos):
    out_root = pathlib.Path(out_root)
    cmd = [CAP, "run", "--benchmark", "libero-pro", "--suite", suite, "--task-id", str(tid), "--seed", str(seed),
           "--init-mode", "seeded", "--max-steps", str(MAX_STEPS), "--program", str(prog_path),
           "--output-root", str(out_root), "--flat-run-dir"] + ([] if videos else ["--no-videos"])
    log = out_root.with_suffix(".log")
    out_root.parent.mkdir(parents=True, exist_ok=True)
    with open(log, "w") as f:
        try:
            rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=REPO, timeout=EP_TIMEOUT).returncode
        except subprocess.TimeoutExpired:
            return {"seed": seed, "success": False, "error": f"program killed after {EP_TIMEOUT}s (endless loop?)", "rc": -9}
    outs = sorted(out_root.rglob("outcome.json"))
    if not outs:
        return {"seed": seed, "success": False, "error": "no outcome", "log_tail": log.read_text(errors='replace')[-600:], "rc": rc}
    pr_path = outs[-1].parent / "source/program-result.json"
    try:
        o = json.load(open(outs[-1]))
        pr = json.load(open(pr_path)) if pr_path.exists() else {}
    except ValueError as e:
        return {"seed": seed, "success": False, "error": f"unreadable outcome: {e}", "rc": rc}
    # Run 3: program-result.json has no error text, so read it from the per-call traces (trace/calls/*/output.json)
    msgs = []
    for f in sorted((outs[-1].parent / "trace/calls").glob("*/output.json")):
        try:
            d = json.load(open(f))
        except Exception:
            continue
        e = d.get("error") if isinstance(d, dict) else None
        if isinstance(e, dict) and e.get("message"):
            det = e.get("details") or {}
            extra = f" (joint_error_rad={det['joint_error_rad']:.3f}, steps_used={d.get('steps_executed')})" if isinstance(det.get("joint_error_rad"), (int, float)) else ""
            msgs.append(e["message"][:140] + extra)
    failed = [{"tool": c.get("tool"), "code": c.get("failure_code") or c.get("error_type"),
               "msg": (c.get("error_message") or "")[:160]} for c in pr.get("calls", []) if not c.get("ok")]
    seen = []  # each failure is traced at two layers: drop exact consecutive repeats
    for m in msgs:
        if not seen or seen[-1] != m:
            seen.append(m)
    return {"seed": seed, "success": bool(o.get("task_success")), "termination": o.get("termination_reason"),
            "steps": o.get("steps_executed"), "wall_s": round(o.get("wall_duration_s") or 0),
            "program_error": (str(pr.get("error"))[:600] if pr.get("error") else None),
            "result": json.dumps(pr.get("result"))[:700], "failed_calls": failed[:8], "errors": seen if len(seen) <= 8 else seen[:4] + seen[-4:], "rc": rc}


def feedback(runs):
    lines = []
    for r in runs:
        lines.append(f"seed {r['seed']}: success={r['success']} termination={r.get('termination')} steps={r.get('steps')} of {MAX_STEPS}")
        if r.get("program_error"):
            lines.append(f"  program error: {r['program_error']}")
        if r.get("error"):
            lines.append(f"  harness error: {r['error']} {r.get('log_tail', '')}")
        lines.append(f"  result: {r.get('result')}")
        for c in r.get("failed_calls", []):
            lines.append(f"  failed call: {c['tool']} -> {c['code']} {c['msg']}")
        for m in r.get("errors", []):
            lines.append(f"  error: {m}")
    return "\n".join(lines)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tasks", required=True)  # comma list of suite:id
    ap.add_argument("--conds", default="A,B,C,D")
    ap.add_argument("--out", required=True)
    ap.add_argument("--model", default="sonnet")
    ap.add_argument("--max-calls", type=int, default=40)  # hard cap on Claude calls this session
    args = ap.parse_args()
    out = pathlib.Path(args.out); out.mkdir(parents=True, exist_ok=True)
    sp = out / "state.json"
    state = json.load(open(sp)) if sp.exists() else {}
    save = lambda: json.dump(state, open(sp, "w"), indent=1)
    api = open(f"{REPO}/docs/api-reference.md").read()
    calls = 0

    for task_ref in args.tasks.split(","):
        suite, tid = task_ref.split(":")
        info = state.setdefault(f"info:{task_ref}", task_info(task_ref)); save()
        query = task_query(info["language"], info["bddl"])
        for cond in args.conds.split(","):
            key = f"{task_ref}|{cond}"
            st = state.setdefault(key, {}); tag = f"{suite}_{tid}_{cond}"
            mem, mem_index = build_memory(cond, query)
            st["memory_chars"], st["memory_sections"] = len(mem), mem_index
            context = (f"# API reference\n{api}\n\n" + (f"# Skill library (lessons from earlier robot tasks)\n{mem}\n\n" if mem else "")
                       + f"# Task\nsuite={suite} task_id={tid}\nlanguage: {info['language']}\nBDDL scene definition:\n{info['bddl']}\n")
            for ver in ("v1", "v2"):
                if ver == "v2" and st.get("v1_dev_success") == len(DEV_SEEDS):
                    st["v2_skipped"] = "v1 passed every dev seed"; save(); break
                if f"{ver}_program" not in st:
                    if calls >= args.max_calls:
                        print(f"[stop] reached --max-calls {args.max_calls}; rerun to resume"); save(); return
                    if ver == "v1":
                        prompt = context + "\nWrite the program for this task."
                    else:
                        prompt = (context + f"\n# Your previous program\n```python\n{st['v1_program']}\n```\n"
                                  f"\n# What happened on the development seeds\n{feedback(st['v1_dev'])}\n"
                                  "\nFix the program. Return the complete corrected program.")
                    try:
                        prog, usage = claude(prompt, args.model)
                    except Exception as e:
                        print(f"[stop] {key} {ver}: {e}"); save(); return  # e.g. usage limit: resume later
                    calls += 1
                    st[f"{ver}_program"], st[f"{ver}_usage"], st[f"{ver}_prompt_chars"] = prog, usage, len(prompt)
                    save(); print(f"[gen] {key} {ver}: {len(prog)} chars, usage {usage}")
                prog_path = out / "programs" / f"{tag}_{ver}.py"; prog_path.parent.mkdir(exist_ok=True)
                prog_path.write_text(st[f"{ver}_program"] or "result = {'empty_program': True}\n")
                for phase, seeds, videos in (("dev", DEV_SEEDS, False), ("heldout", HELDOUT_SEEDS, True)):
                    if ver == "v2" and phase == "dev":
                        continue
                    k = f"{ver}_{phase}"
                    st.setdefault(k, [])
                    done = {r["seed"] for r in st[k]}
                    for s in seeds:
                        if s in done:
                            continue
                        r = run_program(prog_path, suite, tid, s, out / "runs" / tag / ver / f"{phase}_s{s}", videos)
                        st[k].append(r); save()
                        print(f"[run] {key} {ver} {phase} seed {s}: success={r['success']} ({r.get('termination')}, {r.get('wall_s')}s)")
                    st[f"{k}_success"] = sum(r["success"] for r in st[k]); save()
            print(f"[done] {key}: v1 dev {st.get('v1_dev_success')}/{len(DEV_SEEDS)}, v1 heldout "
                  f"{st.get('v1_heldout_success')}/{len(HELDOUT_SEEDS)}, v2 heldout {st.get('v2_heldout_success', '-')}")
    print("PILOT COMPLETE")


if __name__ == "__main__":
    main()
