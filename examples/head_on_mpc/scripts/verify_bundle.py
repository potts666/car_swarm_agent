#!/usr/bin/env python3
"""Independently audit source/artifact hashes, raw motion, packets and videos."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def audit_case(directory, video=True):
    cfg = json.loads((directory/'config.json').read_text())
    settings,road = cfg['settings'],cfg['road']
    by_car = {'A':{},'B':{}}
    with (directory/'actual.csv').open() as stream:
        for row in csv.DictReader(stream):
            values = {key:float(row[key]) for key in ('x','vx','y','acceleration')}
            assert all(np.isfinite(list(values.values()))), 'non-finite executed state'
            by_car[row['car']][float(row['absolute_time_s'])] = values
    assert sorted(by_car['A'])==sorted(by_car['B']), 'unsynchronized execution samples'
    minimum_gap = float('inf')
    for car,rows in by_car.items():
        times = sorted(rows)
        for left,right in zip(times[:-1],times[1:]):
            s,n = rows[left],rows[right]
            dt = right-left
            assert abs(n['x']-(s['x']+dt*s['vx']+.5*dt*dt*s['acceleration']))<1e-8
            assert abs(n['vx']-(s['vx']+dt*s['acceleration']))<1e-8
        for state in rows.values():
            assert abs(state['vx'])<=settings['max_speed']+1e-5
            assert abs(state['acceleration'])<=settings['max_acceleration']+1e-5
            x0,x1 = state['x']-settings['length']/2,state['x']+settings['length']/2
            bottom,top = state['y']-settings['width']/2,state['y']+settings['width']/2
            assert x0>=road['xmin']-1e-8 and x1<=road['xmax']+1e-8 and bottom>=road['ymin']-1e-8
            # Partition the whole body at bay boundaries; check the free-space
            # ceiling on every interval rather than checking body corners only.
            cuts = sorted(set([x0,x1]+[b[k] for b in road['bays'] for k in ('xmin','xmax') if x0<b[k]<x1]))
            for a,b in zip(cuts[:-1],cuts[1:]):
                middle = (a+b)/2
                ceiling = max([road['ymax']]+[bay['ymax'] for bay in road['bays'] if bay['xmin']<=middle<=bay['xmax']])
                assert top<=ceiling+1e-8, f'{car} body outside map'
    for t,a in by_car['A'].items():
        b = by_car['B'][t]
        gap = max(abs(a['x']-b['x'])-settings['length'],abs(a['y']-b['y'])-settings['width'])
        minimum_gap = min(minimum_gap,gap)
        assert gap>=cfg['sampled_gap_min_m']-1e-5
    solves = json.loads((directory/'solves.json').read_text())
    control = {(r['car'],r['cycle']):r['controls'] for r in solves}
    for solve in solves:
        assert solve['accepted'], 'rejected local optimization'
        for stamp in solve['peer_stamps'].values():
            expected_age = settings['dt'] if solve['cycle'] else 0.
            assert abs(solve['stamp']-stamp-expected_age)<1e-8
    packets = 0
    with (directory/'predictions.jsonl').open() as stream:
        for line in stream:
            packet = json.loads(line)
            assert len(packet['states'])==settings['horizon']+2
            assert abs(packet['times'][-1]-packet['stamp']-(settings['horizon']+1)*settings['dt'])<1e-8
            actual = by_car[packet['car']][round(packet['stamp'],10)]
            x,v = actual['x'],actual['vx']
            assert abs(packet['states'][0][0]-x)<1e-8 and abs(packet['states'][0][1]-v)<1e-8
            for index,acceleration in enumerate([*control[packet['car'],packet['cycle']],0.]):
                dt = settings['dt']
                x += dt*v+.5*dt*dt*acceleration
                v += dt*acceleration
                predicted = packet['states'][index+1]
                assert abs(predicted[0]-x)<1e-8 and abs(predicted[1]-v)<1e-8
            packets += 1
    summary = json.loads((directory/'summary.json').read_text())
    assert summary['expectation_met']
    assert abs(summary['minimum_sampled_sat_gap_m']-minimum_gap)<1e-8
    if directory.name=='head_on_reverse':
        events = json.loads((directory/'events.json').read_text())
        start = next(e['time_s'] for e in events if e['to']=='retreat')+settings['epoch']
        assert abs(by_car['B'][round(start,10)]['vx'])<cfg['arrival_speed_tolerance_m_s']
        assert summary['B_reverse_distance_m']>4.
    decoded = 0
    if video:
        metadata = json.loads((directory/'video.json').read_text())
        stream = cv2.VideoCapture(str(directory/'replay.mp4'))
        assert stream.isOpened()
        assert int(stream.get(cv2.CAP_PROP_FRAME_WIDTH))==1280
        assert int(stream.get(cv2.CAP_PROP_FRAME_HEIGHT))==720
        assert abs(stream.get(cv2.CAP_PROP_FPS)-metadata['fps'])<1e-6
        while True:
            ok,_ = stream.read()
            if not ok:
                break
            decoded += 1
        stream.release()
        assert decoded==metadata['frames']
        for car in ('A','B'):
            final = by_car[car][max(by_car[car])]
            for key in ('x','vx','y'):
                assert abs(final[key]-metadata['final_display_states'][car][key])<1e-9
    return {'case':directory.name,'passed':True,'execution_samples_per_car':len(by_car['A']),
            'prediction_packets_verified':packets,'minimum_sampled_sat_gap_m':minimum_gap,
            'video_decoded_frames':decoded}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=ROOT/'results/run_001')
    parser.add_argument('--write-summary',action='store_true')
    args = parser.parse_args()
    run = args.run.resolve()
    provenance = json.loads((run/'provenance.json').read_text())
    for name,digest in provenance['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest, f'executed source changed: {name}; generate a new run'
    manifest = json.loads((run/'manifest.json').read_text())
    for name,digest in manifest.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest, name
    checks = [audit_case(run/name) for name in ('head_on_inline_stop','head_on_pause_bay','head_on_reverse')]
    if args.write_summary:
        (run/'verification.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps({'manifest_files':len(manifest),'cases':checks},indent=2))


if __name__=='__main__':
    main()
