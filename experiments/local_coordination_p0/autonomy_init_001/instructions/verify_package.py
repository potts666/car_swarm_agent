"""Verify the portable instruction package; never invokes an optimizer."""
from pathlib import Path
import hashlib
import json


def main():
    root = Path(__file__).resolve().parent
    manifest = json.loads((root / 'SHA256SUMS.json').read_text(encoding='utf-8'))
    for name, expected in manifest.items():
        target = (root / name).resolve()
        if not target.is_relative_to(root):
            raise ValueError('Manifest path outside package: ' + name)
        if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError('SHA256 mismatch: ' + name)
    print(json.dumps({'passed': True, 'files': len(manifest), 'optimizer_calls': 0}))


if __name__ == '__main__':
    main()
