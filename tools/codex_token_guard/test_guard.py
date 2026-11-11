import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import guard
from install import install


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def event(self, kind="PreToolUse", **fields):
        return {"session_id": "test-session", "hook_event_name": kind, **fields}

    def test_policy_once_and_after_compaction(self):
        start = self.event("SessionStart", source="startup")
        self.assertIsNotNone(guard.handle(start, self.root))
        self.assertIsNone(guard.handle(self.event("SessionStart", source="resume"), self.root))
        self.assertIsNotNone(guard.handle(self.event("SessionStart", source="compact"), self.root))

    def test_rare_notes_and_no_permission_decision(self):
        replies = []
        for i in range(80):
            event = self.event(tool_name="functions.exec", tool_use_id=str(i),
                               tool_input="await tools.write_stdin({session_id: 12, max_output_tokens: 9000})")
            reply = guard.handle(event, self.root)
            if reply:
                replies.append(reply)
        self.assertEqual(len(replies), 3)
        for reply in replies:
            inner = reply["hookSpecificOutput"]
            self.assertEqual(set(inner), {"hookEventName", "additionalContext"})
            self.assertLess(len(inner["additionalContext"]), 500)

    def test_duplicate_call_id_and_privacy(self):
        event = self.event(tool_name="Bash", tool_use_id="same", tool_input={"command": "echo SECRET_TEXT"})
        guard.handle(event, self.root)
        guard.handle(event, self.root)
        text = next(self.root.glob("*.json")).read_text()
        self.assertNotIn("SECRET_TEXT", text)
        self.assertNotIn("same", text)
        self.assertEqual(json.loads(text)["covered_tool_events"], 1)

    def test_guardian_missing_session_and_unknown_events(self):
        self.assertIsNone(guard.handle(self.event("SessionStart", model="codex-auto-review"), self.root))
        self.assertIsNone(guard.handle({"hook_event_name": "SessionStart"}, self.root))
        self.assertIsNone(guard.handle(self.event("Stop"), self.root))
        self.assertFalse(list(self.root.iterdir()))

    def test_invalid_state_recovers_and_signals_are_only_heuristics(self):
        event = self.event(tool_name="Bash", tool_input={"command": "sleep 5"})
        self.assertEqual(guard.signals(event), (True, False))
        self.assertEqual(guard.signals(self.event(tool_name="Bash", tool_input={"command": "python run.py"})), (False, False))
        guard.handle(event, self.root)
        next(self.root.glob("*.json")).write_text("broken")
        self.assertIsNone(guard.handle(event, self.root))

    def test_install_preserves_other_hooks_and_is_idempotent(self):
        target = self.root / "codex"
        target.mkdir()
        before = {"description": "existing", "hooks": {"Stop": [{"hooks": [{"type": "command", "command": "echo original"}]}]}}
        original = json.dumps(before).encode()
        (target / "hooks.json").write_bytes(original)
        install(target)
        config = (target / "hooks.json").read_bytes()
        install(target)
        self.assertEqual(config, (target / "hooks.json").read_bytes())
        result = json.loads(config)
        self.assertEqual(result["hooks"]["Stop"], before["hooks"]["Stop"])
        self.assertEqual((target / "hooks.json.before-token-guard").read_bytes(), original)
        self.assertFalse((target / "config.toml").exists())

    def test_malformed_configuration_is_not_overwritten(self):
        target = self.root / "codex"
        target.mkdir()
        (target / "hooks.json").write_text("broken")
        with self.assertRaises(ValueError):
            install(target)
        self.assertEqual((target / "hooks.json").read_text(), "broken")

    def test_cli_invalid_input_and_oversized_input_are_silent(self):
        for data in (b"broken", b"[]", b"x" * 1048577):
            result = subprocess.run([sys.executable, str(Path(guard.__file__))], input=data, capture_output=True)
            self.assertEqual((result.returncode, result.stdout, result.stderr), (0, b"", b""))


if __name__ == "__main__":
    unittest.main()
