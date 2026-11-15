"""Read-only C0 stopping/interior audit; Python standard library only."""
import argparse
import gzip
import json
from fractions import Fraction as F
from pathlib import Path


def read_object(root, identifier):
    with gzip.open(root / 'objects' / (identifier + '.json.gz'), 'rt', encoding='utf-8') as stream:
        return json.load(stream)


def audit(root):
    levels = json.loads((root / 'analysis' / 'numerical_levels.json').read_text(encoding='utf-8'))
    output = []
    for case, info in levels.items():
        result = read_object(root, info['result_id'])
        model = read_object(root, info['assembly_id'])
        rows = model['assembled_rows']
        c = F(result['numerical_variable_scale'])
        trust = F(result['physical_trust_radius'])
        assert all(F(row['g_upper']) < 0 for row in rows)
        candidates = [trust / 2, F(1, 2)]
        geometry = []
        for row, normalizer in zip(rows, model['row_normalizers']):
            g, e, hessian = (F(row[key]) for key in ('g_upper', 'gradient_error', 'hessian'))
            assert e >= 0 and hessian >= 0
            if e + hessian / 2 > 0:
                candidates.append(-g / (2 * (e + hessian / 2)))
            if hessian > 0:
                r = -g / F(normalizer)
                assert 0 < r <= 1
                geometry.append((2 * r / (1 + r), row['name']))
                beta = 2 * hessian * c * c / (-g)
                # Squared cone identity, tested with exact rational values.
                for w in (F(-2), F(0), F(1), F(3, 2)):
                    for tau in (F(-1), F(0), F(1, 3)):
                        determinant = (2 - w) ** 2 - w ** 2 - beta * tau ** 2
                        scalar = 1 - w - beta * tau ** 2 / 4
                        assert determinant == 4 * scalar
                        assert ((2 - w >= 0 and determinant >= 0) == (scalar >= 0))
        t = min(candidates)
        assert t > 0 and t < trust
        for row in rows:
            g, e, hessian = (F(row[key]) for key in ('g_upper', 'gradient_error', 'hessian'))
            assert g + e * t + hessian * t * t / 2 <= g / 2 < 0
        # The stored binary64 coefficients are treated as exact rationals here.
        tau = t / c
        slacks = [F.from_float(float(b)) - F.from_float(float(a[-1])) * tau
                  for a, b in zip(model['A'], model['b'])]
        dimensions = model['cone_dimensions']
        assert sum(dimensions) == len(slacks)
        assert all(x > 0 for x in slacks[:dimensions[0]])
        start = dimensions[0]
        for size in dimensions[1:]:
            block = slacks[start:start + size]
            assert block[0] > 0 and block[0] ** 2 > sum(x * x for x in block[1:])
            start += size
        primal, dual = result['raw_objective'], result['raw_dual_objective']
        gap = abs(primal - dual)
        relative = gap / max(1, min(abs(primal), abs(dual)))
        failed = []
        if result['primal_residual'] >= 1e-10:
            failed.append('primal_feasibility')
        if result['dual_residual'] >= 1e-10:
            failed.append('dual_feasibility')
        if gap >= 1e-10 and relative >= 1e-10:
            failed.append('optimality_gap')
        assert failed, 'Archived case unexpectedly passes all recorded strict tests'
        minimum, name = min(geometry)
        output.append(dict(case=case, status=result['status'], failed=failed,
                           strict_interior=True, t=str(t), quadratic_blocks=len(geometry),
                           minimum_old_axis_ratio=float(minimum), limiting_block=name,
                           new_quadratic_anchor_axis_ratio=1))
    assert len(output) == 12
    counts = {key: sum(row['failed'] == value for row in output) for key, value in (
        ('primal_only', ['primal_feasibility']),
        ('gap_only', ['optimality_gap']),
        ('both', ['primal_feasibility', 'optimality_gap']))}
    assert counts == dict(primal_only=4, gap_only=4, both=4)
    return dict(scope='Read-only exact archive audit; no solver or controller run',
                counts=counts, cases=output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='Existing stage_c directory')
    args = parser.parse_args()
    print(json.dumps(audit(args.source), ensure_ascii=False, indent=2))
