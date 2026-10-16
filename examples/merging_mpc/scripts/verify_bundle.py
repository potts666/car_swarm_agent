#!/usr/bin/env python3
"""Independent audit of merging raw logs, admission, packets and video files."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def audit_case(directory,video=True):
    cfg = json.loads((directory/'config.json').read_text())
    settings,road = cfg['settings'],cfg['road']
    rows = {car:{} for car in ('A','B','C')}
    with (directory/'actual.csv').open() as stream:
        for row in csv.DictReader(stream):
            values = {key:float(row[key]) for key in ('x','vx','y','acceleration')}
            assert np.all(np.isfinite(list(values.values())))
            rows[row['car']][float(row['absolute_time_s'])] = values
    times = sorted(rows['A'])
    assert all(sorted(car)==times for car in rows.values())
    minimum_gap = float('inf')
    minimum_headway = float('inf')
    for car,samples in rows.items():
        for left,right in zip(times[:-1],times[1:]):
            s,n,dt = samples[left],samples[right],right-left
            assert abs(n['x']-s['x']-dt*s['vx']-.5*dt*dt*s['acceleration'])<1e-8
            assert abs(n['vx']-s['vx']-dt*s['acceleration'])<1e-8
        for s in samples.values():
            assert -1e-5<=s['vx']<=settings['max_speed']+1e-5
            assert abs(s['acceleration'])<=settings['max_acceleration']+1e-5
            assert s['x']-settings['length']/2>=road['xmin']-1e-8
            assert s['x']+settings['length']/2<=road['xmax']+1e-8
            assert s['y']-settings['width']/2>=road['ymin']-1e-8
            assert s['y']+settings['width']/2<=road['ymax']+1e-8
    for t in times:
        for a,b in itertools.combinations(rows,2):
            left,right = rows[a][t],rows[b][t]
            gap = max(abs(left['x']-right['x'])-settings['length'],abs(left['y']-right['y'])-settings['width'])
            minimum_gap = min(minimum_gap,gap)
            assert gap>=cfg['sampled_gap_min_m']-1e-5
        if cfg['cases'][cfg['case']]['cooperative'] and t>=settings['epoch']+cfg['request_time_s']:
            for a,b in (('A','C'),('C','B')):
                residual = rows[a][t]['x']-rows[b][t]['x']-settings['length']-settings['margin']-settings['headway']*rows[b][t]['vx']
                minimum_headway = min(minimum_headway,residual)
                assert residual>=-1e-5
    solves = json.loads((directory/'solves.json').read_text())
    controls = {(r['car'],r['cycle']):r['controls'] for r in solves}
    for solve in solves:
        assert solve['accepted']
        following = cfg['cases'][cfg['case']]['cooperative'] and solve['phase']!='following' and solve['car']!='A'
        expected_front = None if solve['car']=='A' else 'C' if solve['car']=='B' and following else 'A'
        assert solve['front_car']==expected_front and solve['following_cost_active']==following
        for stamp in solve['peer_stamps'].values():
            assert abs(solve['stamp']-stamp-(settings['dt'] if solve['cycle'] else 0.))<1e-8
    packets = {}
    with (directory/'predictions.jsonl').open() as stream:
        for line in stream:
            packet = json.loads(line)
            packets[packet['car'],round(packet['stamp'],10)] = packet
            assert len(packet['states'])==settings['horizon']+2
            assert abs(packet['times'][-1]-packet['stamp']-(settings['horizon']+1)*settings['dt'])<1e-8
            s = rows[packet['car']][round(packet['stamp'],10)]
            x,v = s['x'],s['vx']
            np.testing.assert_allclose(packet['states'][0],[x,v,s['y']],atol=1e-8,rtol=0)
            for index,acceleration in enumerate([*controls[packet['car'],packet['cycle']],0.]):
                dt = settings['dt']
                x += dt*v+.5*dt*dt*acceleration
                v += dt*acceleration
                np.testing.assert_allclose(packet['states'][index+1][:2],[x,v],atol=1e-8,rtol=0)
    guards = json.loads((directory/'admission_checks.json').read_text())
    assert guards, 'missing admission records'
    for guard in guards:
        now = settings['epoch']+guard['time_s']
        stamps = now-settings['dt']
        query = now+np.linspace(0,settings['lateral_seconds'],31)
        aligned = {}
        for car in ('A','C','B'):
            packet = packets[car,round(stamps,10)]
            assert query[-1]<=packet['times'][-1]+1e-8
            data = np.asarray(packet['states'])
            aligned[car] = np.column_stack([np.interp(query,packet['times'],data[:,i]) for i in (0,1)])
        residuals = {}
        for a,b in (('A','C'),('C','B')):
            residuals[f'{a}-{b}'] = float(np.min(aligned[a][:,0]-aligned[b][:,0]-settings['length']-
                settings['margin']-settings['headway']*np.maximum(0.,aligned[b][:,1])-cfg['admission_buffer_m']))
        for key,value in residuals.items():
            assert abs(value-guard['pair_residuals_m'][key])<1e-8
        assert guard['admitted']==(min(residuals.values())>=-settings['tolerance'])
    summary = json.loads((directory/'summary.json').read_text())
    assert summary['expectation_met']
    assert abs(summary['minimum_sampled_sat_gap_m']-minimum_gap)<1e-8
    if cfg['cases'][cfg['case']]['cooperative']:
        assert summary['merge_completed_at_s'] is not None
        assert abs(summary['merge_completed_at_s']-summary['admitted_at_s']-settings['lateral_seconds'])<1e-8
        assert abs(rows['C'][times[-1]]['y']-cfg['main_lane_y'])<1e-8
        assert rows['A'][times[-1]]['x']>rows['C'][times[-1]]['x']>rows['B'][times[-1]]['x']
        assert summary['B_minimum_speed_m_s']<cfg['cruise_speed']-.1
        assert all(abs(car[times[-1]]['vx']-cfg['cruise_speed'])<cfg['settled_speed_tolerance_m_s'] for car in rows.values())
    else:
        assert summary['status']=='blocked' and summary['admitted_at_s'] is None
        assert not any(guard['admitted'] for guard in guards)
        assert all(abs(s['y']-cfg['merge_lane_y'])<1e-8 for s in rows['C'].values())
    decoded = 0
    if video:
        metadata = json.loads((directory/'video.json').read_text())
        stream = cv2.VideoCapture(str(directory/'replay.mp4'))
        assert stream.isOpened()
        assert (int(stream.get(cv2.CAP_PROP_FRAME_WIDTH)),int(stream.get(cv2.CAP_PROP_FRAME_HEIGHT)))==(1280,720)
        assert abs(stream.get(cv2.CAP_PROP_FPS)-metadata['fps'])<1e-6
        fourcc = int(stream.get(cv2.CAP_PROP_FOURCC))
        assert ''.join(chr((fourcc>>(8*i))&255) for i in range(4))=='avc1'
        while True:
            ok,_ = stream.read()
            if not ok:
                break
            decoded += 1
        stream.release()
        assert decoded==metadata['frames']
        for car in rows:
            for key in ('x','vx','y'):
                assert abs(rows[car][times[-1]][key]-metadata['final_display_states'][car][key])<1e-9
    return {'case':directory.name,'passed':True,'execution_samples_per_car':len(times),
            'prediction_packets_verified':len(packets),'admission_decisions_recomputed':len(guards),
            'minimum_sampled_sat_gap_m':minimum_gap,
            'minimum_active_headway_residual_m':minimum_headway if np.isfinite(minimum_headway) else None,
            'video_decoded_frames':decoded}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=ROOT/'results/run_001')
    parser.add_argument('--write-summary',action='store_true')
    args = parser.parse_args()
    run = args.run.resolve()
    version = json.loads((run/'provenance.json').read_text())
    for name,digest in version['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest, f'executed source changed: {name}'
    manifest = json.loads((run/'manifest.json').read_text())
    for name,digest in manifest.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    checks = [audit_case(run/name) for name in ('merge_no_cooperation','merge_cooperative')]
    if args.write_summary:
        (run/'verification.json').write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps({'manifest_files':len(manifest),'cases':checks},indent=2))


if __name__=='__main__':
    main()
