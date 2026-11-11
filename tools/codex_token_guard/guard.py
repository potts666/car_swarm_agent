#!/usr/bin/env python3
"""Advisory Codex hooks. No model calls, command execution, or transcript reads."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile
import time

POLICY = """Token-efficient execution: follow the user and required checks. For long work,
reuse a bounded stage driver that advances normal stages, preserves full logs,
and reports failures with exit code, layer, and evidence path. Prefer supported
completion/failure events; otherwise use low-frequency interruptible observations
within tool and user-update limits. Do not repeatedly reason just to keep waiting.
Read small summaries first, then targeted failure windows. Batch independent,
authorized mechanical checks without hiding errors or changing approvals.
At a natural phase boundary, save a short handoff: goal, constraints, source
version, evidence paths, unresolved issues, exact resume command. Suggest a new
session when appropriate; never abandon active work or erase history yourself.
Reuse validated validation/archive tools and content-checked checkpoints. Preserve
required repetitions, independent acceptance, and original evidence. Report measured
usage separately from heuristic hook counts; never infer billing from token totals.
These hooks cannot see native write_stdin polls or shrink existing context."""

NOTES = {
    "wait": "Several hook-visible wait/poll-like calls were observed (heuristic). Prefer stage completion/failure events or fewer bounded observations; preserve interruption and required user updates. Native write_stdin polls may be invisible. Do not end the task just to save tokens.",
    "output": "A large explicit output budget was detected. Prefer a small status/test summary and preserve full output on disk; expand the relevant failure window when needed. Do not drop evidence or change this pending command automatically.",
    "handoff": "Many hook-visible tools have run. At the next natural phase boundary, save a short handoff with constraints, source/evidence paths, issues and exact resume entry. Continue authorized work; do not force a reset or start another model session automatically.",
}


def signals(data):
    name = str(data.get("tool_name", ""))
    args = data.get("tool_input", {})
    if isinstance(args, str):
        # Code-mode inputs can be JS source; inspect text, never evaluate it.
        text = args
        args = {}
    else:
        text = json.dumps(args, ensure_ascii=True)
    wait = bool(re.search(r"(?:^|[._])(?:sleep|wait|write_stdin)$", name))
    wait |= bool(re.search(r"\b(?:tools\.)?(?:write_stdin|clock__sleep)\s*\(|\bclock\.sleep\s*\(", text))
    command = args.get("command", args.get("cmd", "")) if isinstance(args, dict) else ""
    wait |= bool(re.fullmatch(r"\s*sleep\s+\d+(?:\.\d+)?\s*", str(command)))
    budgets = re.findall(r'["\']?(?:max_output_tokens|max_tokens)["\']?\s*:\s*(\d+)', text)
    large = any(int(n) == 0 or int(n) > 4000 for n in budgets)
    return wait, large


def update(data, state):
    event = data.get("hook_event_name")
    if event == "SessionStart":
        if not state.get("policy_sent") or data.get("source") in {"compact", "clear", "fork"}:
            state["policy_sent"] = True
            return POLICY
        return ""
    if event != "PreToolUse":
        return ""
    call_id = data.get("tool_use_id")
    if call_id:
        digest = hashlib.sha256(str(call_id).encode()).hexdigest()[:24]
        recent = state.setdefault("recent_calls", [])
        if digest in recent:
            return ""
        state["recent_calls"] = (recent + [digest])[-32:]
    state["covered_tool_events"] = state.get("covered_tool_events", 0) + 1
    wait, large = signals(data)
    state["wait_like_events"] = state.get("wait_like_events", 0) + int(wait)
    state["large_output_budget_events"] = state.get("large_output_budget_events", 0) + int(large)
    sent = state.setdefault("notes_sent", [])
    due = {"wait": state["wait_like_events"] >= 3,
           "output": large,
           "handoff": state["covered_tool_events"] >= 50}
    for kind in NOTES:
        if due[kind] and kind not in sent:
            sent.append(kind)
            return NOTES[kind]
    return ""


def handle(data, directory=None):
    if not isinstance(data, dict) or data.get("hook_event_name") not in {"SessionStart", "PreToolUse"}:
        return None
    # Approval reviewers do not need workflow reminders or parent-shared counters.
    if "auto-review" in str(data.get("model", "")) or data.get("agent_type") == "guardian":
        return None
    sid = data.get("session_id")
    if not isinstance(sid, str) or not sid:
        return None
    root = Path(directory) if directory is not None else Path(__file__).resolve().parent / "state"
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    key = hashlib.sha256(sid.encode()).hexdigest()
    path = root / (key + ".json")
    with (root / (key + ".lock")).open("a") as lock:
        # Hook failures must not block experimental work or become retry loops.
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return None
        try:
            state = json.loads(path.read_text()) if path.exists() else {}
            if not isinstance(state, dict):
                state = {}
        except (ValueError, OSError):
            state = {}
        note = update(data, state)
        state["updated_at_unix"] = time.time()
        with tempfile.NamedTemporaryFile(mode="w", dir=root, delete=False) as out:
            temp = Path(out.name)
            try:
                json.dump(state, out, sort_keys=True)
                out.flush()
                os.fsync(out.fileno())
                os.replace(temp, path)
            finally:
                temp.unlink(missing_ok=True)
    if note:
        return {"hookSpecificOutput": {"hookEventName": data["hook_event_name"],
                                       "additionalContext": note}}
    return None


def main():
    os.umask(0o077)
    try:
        raw = sys.stdin.buffer.read(1048577)
        if len(raw) > 1048576:
            return
        result = handle(json.loads(raw))
        if result:
            print(json.dumps(result))
    except (ValueError, OSError, TypeError, AttributeError):
        # No stdout on failure: normal tool execution and approvals remain intact.
        return


if __name__ == "__main__":
    main()
