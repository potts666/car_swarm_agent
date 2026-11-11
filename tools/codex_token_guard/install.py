#!/usr/bin/env python3
"""Merge only this hook's entries; never edit trust, permissions, or models."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import tempfile


def install(target):
    target = target.expanduser().resolve()
    source = Path(__file__).with_name("guard.py")
    destination = target / "hooks" / "token_guard" / "guard.py"
    hook_file = target / "hooks.json"
    original = hook_file.read_bytes() if hook_file.exists() else None
    config = json.loads(original) if original is not None else {"hooks": {}}
    groups = config.setdefault("hooks", {})
    command = "python3 " + shlex.quote(str(destination))
    for event in ("SessionStart", "PreToolUse"):
        entries = groups.setdefault(event, [])
        existing = [h for group in entries for h in group.get("hooks", []) if h.get("command") == command]
        handler = {"type": "command", "command": command, "timeout": 2, "additionalContextLimit": 400}
        if existing and existing != [handler]:
            raise ValueError("Existing token_guard entry differs; review it before replacing")
        if not existing:
            entries.append({"hooks": [handler]})
    destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if destination.exists() and destination.read_bytes() != source.read_bytes():
        raise ValueError("Existing guard.py differs; review upgrade before replacing")
    shutil.copyfile(source, destination)
    destination.chmod(0o600)
    updated = (json.dumps(config, indent=2, ensure_ascii=False) + "\n").encode()
    if updated != original:
        backup = target / "hooks.json.before-token-guard"
        if original is not None and not backup.exists():
            with backup.open("xb") as out:
                out.write(original)
            backup.chmod(0o600)
        with tempfile.NamedTemporaryFile(dir=target, delete=False) as out:
            temp = Path(out.name)
            try:
                out.write(updated)
                out.flush()
                os.fsync(out.fileno())
                os.replace(temp, hook_file)
            finally:
                temp.unlink(missing_ok=True)
    return {"hooks_file": str(hook_file), "script": str(destination),
            "trust": "not modified; review SessionStart and PreToolUse in Codex /hooks or Settings > Hooks"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=Path, default=Path(os.environ.get("CODEX_HOME", "~/.codex")))
    args = parser.parse_args()
    os.umask(0o077)
    print(json.dumps(install(args.target), indent=2))
