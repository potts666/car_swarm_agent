"""Common recorded-state reader for video and RViz display."""
import bisect
import csv
import json
import math
from pathlib import Path


class Recording:
    """Load immutable recorded states; interpolate display frames only."""

    def __init__(self, directory):
        directory = Path(directory).expanduser().resolve()
        self.config = json.loads((directory/'config.json').read_text())
        self.summary = json.loads((directory/'summary.json').read_text())
        if 'road' not in self.config or 'settings' not in self.config:
            raise ValueError('Choose a merging case folder containing config.json and actual.csv')
        self.rows = {}
        with (directory/'actual.csv').open(newline='') as stream:
            for row in csv.DictReader(stream):
                sample = {key: float(row[key]) for key in
                          ('absolute_time_s', 'x', 'vx', 'y', 'acceleration')}
                if not all(math.isfinite(v) for v in sample.values()):
                    raise ValueError('Recording contains a non-finite state')
                sample['phase'] = row['phase']
                self.rows.setdefault(row['car'], {})[round(sample['absolute_time_s'], 10)] = sample
        if not self.rows:
            raise ValueError('Recording has no executed states')
        self.cars = sorted(self.rows)
        if self.cars != ['A', 'B', 'C']:
            raise ValueError('This package supports exactly the three merging cars A/B/C')
        self.times = {car: sorted(rows) for car, rows in self.rows.items()}
        self.rows = {car: [rows[t] for t in self.times[car]] for car, rows in self.rows.items()}
        self.start = max(t[0] for t in self.times.values())
        self.end = min(t[-1] for t in self.times.values())
        if self.end <= self.start:
            raise ValueError('Recording has no common positive playback duration')
        self.predictions = {car: [] for car in self.cars}
        with (directory/'predictions.jsonl').open() as stream:
            for line in stream:
                packet = json.loads(line)
                self.predictions[packet['car']].append(packet)
        self.stamps = {}
        for car in self.cars:
            self.predictions[car].sort(key=lambda p: p['stamp'])
            self.stamps[car] = [p['stamp'] for p in self.predictions[car]]

    def sample(self, car, stamp):
        stamp = min(self.end, max(self.start, stamp))
        times, rows = self.times[car], self.rows[car]
        i = max(0, bisect.bisect_right(times, stamp)-1)
        result = rows[i].copy()
        if i+1 < len(times):
            fraction = (stamp-times[i])/(times[i+1]-times[i])
            for key in ('x', 'vx', 'y'):
                result[key] += fraction*(rows[i+1][key]-rows[i][key])
        return result

    def trail(self, car, stamp):
        index = bisect.bisect_right(self.times[car], stamp)
        return [(r['x'], r['y']) for r in self.rows[car][:index]]

    def plan(self, car, stamp):
        index = bisect.bisect_right(self.stamps[car], stamp)-1
        if index < 0:
            return []
        packet = self.predictions[car][index]
        return [(s[0], s[2]) for t, s in zip(packet['times'], packet['states']) if t >= stamp-1e-8]
