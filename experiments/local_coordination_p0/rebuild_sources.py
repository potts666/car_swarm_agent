#!/usr/bin/env python3
"""Bounded reconstruction of supplied P0 objects; never writes the input package."""
import argparse
from collections import defaultdict
import hashlib
import heapq
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def budgets(work):
    base = work/'NTU_DMPC_Dissertation/graph_certificate'
    graph = json.loads((base/'joint_graph.json').read_text())
    motions = json.loads((base/'motions.json').read_text())
    view = json.loads((base/'expanded_graph_derived.json').read_text())
    expanded = []
    for i, edge in enumerate(graph['winning_edges']):
        previous = edge['source']
        blocks = motions[edge['motion']]['blocks']
        for j, block in enumerate(blocks):
            target = edge['target'] if j+1 == len(blocks) else f'@{i}:{j+1}'
            expanded.append(dict(source=previous, target=target, ticks=block['ticks'],
                actor=edge['actor'], motion=edge['motion'], block_index=j, coarse_edge_index=i))
            previous = target
    assert expanded == view['edges'], 'Derived expanded edges differ from reconstruction'
    nodes = set(graph['goal_levels']) | {e['source'] for e in expanded} | {e['target'] for e in expanded}
    assert nodes == set(view['nodes'])
    outgoing, reverse = defaultdict(list), defaultdict(list)
    for edge in expanded:
        assert type(edge['ticks']) is int and 1 <= edge['ticks'] <= 32
        outgoing[edge['source']].append(edge)
        reverse[edge['target']].append(edge)
    # Dijkstra checks C independently of the archived recursive shortest-path code.
    shortest, policy, queue = {graph['goal']:0}, {}, [(0, graph['goal'])]
    while queue:
        cost, node = heapq.heappop(queue)
        if shortest[node] != cost:
            continue
        for edge in reverse[node]:
            source = edge['source']; candidate = cost+edge['ticks']
            if candidate < shortest.get(source, float('inf')):
                shortest[source] = candidate; policy[source] = edge
                heapq.heappush(queue, (candidate, source))
    assert set(shortest) == nodes
    # Reverse topological removal independently checks acyclicity and R.
    remaining = {node:len(outgoing[node]) for node in nodes}
    ready = [node for node in nodes if remaining[node] == 0]
    assert ready == [graph['goal']], 'A non-goal terminal lacks a continuation'
    longest = {graph['goal']:0}; consumed = 0
    while ready:
        node = ready.pop(); consumed += 1
        for edge in reverse[node]:
            source = edge['source']
            longest[source] = max(longest.get(source,0), edge['ticks']+longest[node])
            remaining[source] -= 1
            if remaining[source] == 0:
                ready.append(source)
    assert consumed == len(nodes), 'Cyclic or missing progress graph'
    for node in nodes-{graph['goal']}:
        assert outgoing[node]
        assert policy[node]['ticks']+shortest[policy[node]['target']] == shortest[node]
        assert all(e['ticks']+longest[e['target']] <= longest[node] for e in outgoing[node])
    initial = graph['initial']; witness = []; node = initial
    while node != graph['goal']:
        edge = policy[node]; witness.append(edge); node = edge['target']
    assert sum(e['ticks'] for e in witness) == shortest[initial]
    return dict(safe_nodes=len(graph['nodes']), safe_edges=len(graph['edges']),
        winning_nodes=len(graph['goal_levels']), winning_edges=len(graph['winning_edges']),
        expanded_nodes=len(nodes), expanded_edges=len(expanded),
        initial=initial, goal=graph['goal'], shortest_ticks=shortest[initial], longest_ticks=longest[initial],
        C=shortest, R=longest, shortest_default_policy=policy, shortest_witness=witness,
        source_sha256={name:digest(base/name) for name in ('motions.json','joint_graph.json','expanded_graph_derived.json')},
        scope='independent graph/budget reconstruction; no claim of motion-optimized controller execution')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stage-timeout', type=int, default=1800)
    args = parser.parse_args()
    if not __debug__:
        parser.error('assertions must remain enabled')
    if args.output.exists() or args.stage_timeout <= 0:
        parser.error('use a new output directory and positive timeout')
    source = args.source.resolve(); out = args.output.resolve(); out.mkdir(parents=True)
    manifest = json.loads((source/'manifest.json').read_text())
    entries = manifest['original_files']
    for name, expected in entries.items():
        path = (source/name).resolve()
        assert path.is_relative_to(source), name
        assert path.stat().st_size == expected['bytes'] and digest(path) == expected['sha256'], name
    work = out/'work'; shutil.copytree(source, work)
    write(out/'input_verification.json', dict(original_files=len(entries), verified=True,
          manifest_sha256=digest(source/'manifest.json'), source=str(source),
          files={str(p.relative_to(source)):digest(p) for p in source.rglob('*') if p.is_file()}))
    shutil.copy2(Path(__file__), out/'rebuild_sources.py')
    stages = []

    def stage(name, command):
        log = out/(name+'.log'); started = time.monotonic()
        stages.append(dict(stage=name,status='running',command=command,evidence=str(log)))
        write(out/'stages.json',stages)
        print(name+': running',flush=True)
        with log.open('w') as stream:
            try:
                result = subprocess.run([sys.executable,*command],cwd=work,stdout=stream,
                    stderr=subprocess.STDOUT,timeout=args.stage_timeout)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = 124
        stages[-1].update(status='passed' if code == 0 else 'failed',exit_code=code,wall_s=time.monotonic()-started)
        write(out/'stages.json',stages)
        print(name+': exit '+str(code)+'; '+str(log),flush=True)
        return code

    commands = [
        ('interval_templates',['NTU_DMPC_Dissertation/certificate_graph.py','--verify']),
        ('proof_interfaces',['NTU_DMPC_Dissertation/局部协调三定理_全文推导_2026-11-11/proof_checks.py']),
        ('mathematical_checks',['NTU_DMPC_Dissertation/运动交接与时间目标_理论核对_2026-11-11/math_checks.py'])]
    for name, command in commands:
        code = stage(name,command)
        if code:
            write(out/'summary.json',dict(status='failed',layer=name,exit_code=code,evidence=str(out/(name+'.log'))))
            return code
    values = budgets(work); write(out/'recomputed_budgets.json',values)
    comparisons = {}
    for name in entries:
        if name.endswith('/proof_checks.json') or name.endswith('/math_checks.json'):
            original = json.loads((source/name).read_text()); current = json.loads((work/name).read_text())
            comparisons[name] = dict(semantic_equal=original==current,original_sha256=digest(source/name),recomputed_sha256=digest(work/name))
            assert original == current, name
    for name, expected in entries.items():
        assert digest(source/name) == expected['sha256'], 'Input archive changed: '+name
    write(out/'archive_comparison.json',comparisons)
    write(out/'summary.json',dict(status='passed',exit_code=0,
        shortest_ticks=values['shortest_ticks'],longest_ticks=values['longest_ticks'],
        full_P0_passed=False,scope='supplied archival objects independently reconstructed; controller P0 remains separate'))
    print('source reconstruction passed; full controller P0 remains separate',flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
