"""Bind already completed independent checks to exact files and source versions."""
from common import *

def main():
    for side in ('A','B'):assert read(HERE/'results'/side/'independent_acceptance.json')['passed'] and read(HERE/'results'/side/'rejection/summary.json')['passed']
    code={str(f.relative_to(HERE)):sha(f) for f in HERE.rglob('*.py') if 'delivery' not in f.relative_to(HERE).parts and '__pycache__' not in f.parts}
    files={str(f.relative_to(HERE)):sha(f) for section in ('results','objects') for f in (HERE/section).rglob('*') if f.is_file() and f!=HERE/'results/independent_checkpoints.json'}
    write(HERE/'results/independent_checkpoints.json',dict(full_replay_completed=True,native_calls=2,code=code,files=files,mode='post-execution binding; default relocation reuses content-checked full replay and freshly checks source/prefix'))
if __name__=='__main__':main()
