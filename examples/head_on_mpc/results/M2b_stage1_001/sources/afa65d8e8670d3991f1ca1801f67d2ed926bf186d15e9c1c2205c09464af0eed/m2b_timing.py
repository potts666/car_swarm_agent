"""Optional nested wall timing; exclusive categories do not double count children."""
from collections import defaultdict
from contextlib import contextmanager
from contextvars import ContextVar
import copy
from functools import wraps
import sys
import time

ACTIVE = ContextVar('m2b_timing', default=None)


class CycleTiming:
    def __init__(self):
        self.rows = defaultdict(lambda: dict(calls=0, inclusive_s=0., exclusive_s=0.))
        self.stack = []
        self.phase = 'before_last_final'
        self.phases = defaultdict(lambda: defaultdict(float))
        self.marks = {}
        self.final_sends = 0
        self.last = time.perf_counter()

    def charge(self, now):
        if self.stack:
            self.phases[self.phase][self.stack[-1][2]] += now-self.last
        self.last = now

    def set_phase(self, name):
        now = time.perf_counter()
        self.charge(now)
        self.phase = name
        self.marks[name] = now

    @contextmanager
    def span(self, name):
        now = time.perf_counter()
        self.charge(now)
        frame = [now, 0., name]
        self.stack.append(frame)
        try:
            yield
        finally:
            now = time.perf_counter()
            self.charge(now)
            elapsed = now-frame[0]
            self.stack.pop()
            row = self.rows[name]
            row['calls'] += 1
            row['inclusive_s'] += elapsed
            row['exclusive_s'] += elapsed-frame[1]
            if self.stack:
                self.stack[-1][1] += elapsed


@contextmanager
def span(name):
    active = ACTIVE.get()
    if active is None:
        yield
    else:
        with active.span(name):
            yield


def clone(value):
    with span('copy'):
        return copy.deepcopy(value)


def timed(name):
    def decorate(fn):
        @wraps(fn)
        def wrapped(*args, **kwargs):
            with span(name):
                return fn(*args, **kwargs)
        return wrapped
    return decorate


@contextmanager
def instrument():
    """Patch imported aliases together, then restore; no physical function is replaced."""
    specs = {
        'bicycle_core': {'rollout':'prediction_integration', 'corners':'body_geometry',
                         'outside_area':'road_geometry', 'gaps':'pair_geometry', 'clipped_area':'polygon_clip'},
        'm2b_scene': {'audit':'physical_audit'},
        'm2a_protocol': {'canonical':'encoding'},
        'm2b_control': {'reference_qp':'reference_total', 'bicycle_qp':'bicycle_total', 'solve_matrix':'matrix_solver'},
        'm2a_local': {'local_corridors':'corridor_construction'},
        'linearized_joint': {'condensed':'condensed_model'},
    }
    originals = {}
    for mod, entries in specs.items():
        for name, label in entries.items():
            module = sys.modules.get(mod)
            if module and hasattr(module, name):
                originals[getattr(module, name)] = label
    changed = []
    for module in list(sys.modules.values()):
        if not module or not str(getattr(module, '__file__', '')).startswith(str(__file__.rsplit('/',1)[0])):
            continue
        for name, value in list(vars(module).items()):
            if not callable(value):
                continue
            try:
                label = originals.get(value)
            except TypeError:
                continue
            if label:
                @wraps(value)
                def wrapped(*args, _fn=value, _label=label, **kwargs):
                    with span(_label):
                        return _fn(*args, **kwargs)
                changed.append((module, name, value))
                setattr(module, name, wrapped)
    try:
        yield
    finally:
        for module, name, value in changed:
            setattr(module, name, value)
