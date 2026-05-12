"""Aggregate failure-mode statistics from `sac_inference --log-dir` CSV logs.

Reads every `<ckpt>_ep<N>.csv` file in --csv-dir and prints:

  1. Per-episode table: outcome, final step, start pose, σ stats, error stats,
     step where |EKF − GT| first crossed 1 m and 2 m.
  2. Outcome breakdown: counts + mean step / mean σ per outcome bucket.

The script is stdlib-only (no pandas). Designed to be cheap to re-run on a
growing collection of inference CSVs.

    ros2 run rl_navigation_pkg failure_analysis \\
        --csv-dir ./sac_runs/phase1_001/inference/csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


NAN_TOKENS = {'', 'nan', 'NaN', 'None'}


def _as_float(token: str | None) -> float:
    if token is None or token in NAN_TOKENS:
        return float('nan')
    try:
        return float(token)
    except ValueError:
        return float('nan')


def _finite(values):
    return [v for v in values if not math.isnan(v)]


def summarize_episode(csv_path: Path) -> dict:
    with csv_path.open() as fh:
        rows = list(csv.DictReader(fh))

    if not rows:
        return {'path': csv_path, 'final_step': 0, 'outcome': 'empty'}

    last = rows[-1]
    first = rows[0]
    outcome = (
        last.get('terminated_reason')
        or last.get('truncated_reason')
        or 'unknown'
    )
    if outcome in NAN_TOKENS:
        outcome = 'unknown'

    sigma_wheels = _finite([_as_float(r['sigma_wheel']) for r in rows])
    sigma_imus = _finite([_as_float(r['sigma_imu']) for r in rows])
    sigma_lidars = _finite([_as_float(r.get('sigma_lidar', '')) for r in rows])
    errors = _finite([_as_float(r['error_l2']) for r in rows])
    rewards = [_as_float(r['reward']) for r in rows]

    step_1m = next(
        (int(r['step']) for r in rows
         if not math.isnan(_as_float(r['error_l2']))
         and _as_float(r['error_l2']) > 1.0),
        None,
    )
    step_2m = next(
        (int(r['step']) for r in rows
         if not math.isnan(_as_float(r['error_l2']))
         and _as_float(r['error_l2']) > 2.0),
        None,
    )

    nav_labels = [r.get('nav_status_label', '') for r in rows]
    # Collapse repeats: keep ordered list of distinct labels actually seen
    # during the episode. Useful for spotting rejected/aborted/canceled.
    seen: list[str] = []
    for lbl in nav_labels:
        if lbl and (not seen or seen[-1] != lbl):
            seen.append(lbl)
    nav_trace = ' → '.join(seen) if seen else ''
    # Dominant label = the one held for the most steps (excludes 'sent' bursts).
    counts: dict[str, int] = {}
    for lbl in nav_labels:
        if lbl:
            counts[lbl] = counts.get(lbl, 0) + 1
    dominant = max(counts.items(), key=lambda kv: kv[1])[0] if counts else ''

    return {
        'path': csv_path,
        'final_step': int(last['step']),
        'outcome': outcome,
        'total_reward': sum(r for r in rewards if not math.isnan(r)),
        'start_x': _as_float(first.get('ekf_x')),
        'start_y': _as_float(first.get('ekf_y')),
        'goal_x': _as_float(first.get('goal_x')),
        'goal_y': _as_float(first.get('goal_y')),
        'sigma_wheel_mean': statistics.mean(sigma_wheels) if sigma_wheels else float('nan'),
        'sigma_wheel_min': min(sigma_wheels) if sigma_wheels else float('nan'),
        'sigma_wheel_max': max(sigma_wheels) if sigma_wheels else float('nan'),
        'sigma_imu_mean': statistics.mean(sigma_imus) if sigma_imus else float('nan'),
        'sigma_imu_min': min(sigma_imus) if sigma_imus else float('nan'),
        'sigma_imu_max': max(sigma_imus) if sigma_imus else float('nan'),
        'sigma_lidar_mean': statistics.mean(sigma_lidars) if sigma_lidars else float('nan'),
        'sigma_lidar_min': min(sigma_lidars) if sigma_lidars else float('nan'),
        'sigma_lidar_max': max(sigma_lidars) if sigma_lidars else float('nan'),
        'error_max': max(errors) if errors else float('nan'),
        'error_final': errors[-1] if errors else float('nan'),
        'step_1m': step_1m,
        'step_2m': step_2m,
        'nav_dominant': dominant,
        'nav_trace': nav_trace,
    }


def _fmt(x: float, width: int = 6, prec: int = 2) -> str:
    if isinstance(x, float) and math.isnan(x):
        return f'{"nan":>{width}}'
    return f'{x:{width}.{prec}f}'


def print_per_episode(summaries: list[dict]) -> None:
    print(f'\nper-episode summary ({len(summaries)} episodes)\n')
    header = (
        f'{"episode":<28} {"steps":>5} {"outcome":<22} {"reward":>9} '
        f'{"goal_xy":<16} {"err_max":>7} {"@1m":>5} {"@2m":>5} '
        f'{"nav_dom":<12} {"nav_trace"}'
    )
    print(header)
    print('-' * len(header))
    for s in summaries:
        gxy = f'({_fmt(s["goal_x"], 5, 2)},{_fmt(s["goal_y"], 5, 2)})'
        ep1m = '-' if s['step_1m'] is None else f'{s["step_1m"]:>5}'
        ep2m = '-' if s['step_2m'] is None else f'{s["step_2m"]:>5}'
        print(
            f'{s["path"].stem:<28} {s["final_step"]:>5} {s["outcome"]:<22} '
            f'{s["total_reward"]:>+9.1f} {gxy:<16} '
            f'{_fmt(s["error_max"], 7, 2)} {ep1m} {ep2m} '
            f'{s["nav_dominant"]:<12} {s["nav_trace"]}'
        )


def print_outcome_breakdown(summaries: list[dict]) -> None:
    print('\noutcome breakdown\n')
    buckets: dict[str, list[dict]] = {}
    for s in summaries:
        buckets.setdefault(s['outcome'], []).append(s)

    total = len(summaries)
    header = (
        f'{"outcome":<22} {"count":>10} {"mean steps":>12} '
        f'{"σ_wheel μ":>12} {"σ_imu μ":>12} {"σ_lidar μ":>12} {"mean err_max":>14}'
    )
    print(header)
    print('-' * len(header))
    for outcome in sorted(buckets):
        group = buckets[outcome]
        n = len(group)
        share = n / total * 100
        mean_steps = statistics.mean(s['final_step'] for s in group)
        sw = _finite([s['sigma_wheel_mean'] for s in group])
        si = _finite([s['sigma_imu_mean'] for s in group])
        sl = _finite([s['sigma_lidar_mean'] for s in group])
        em = _finite([s['error_max'] for s in group])
        print(
            f'{outcome:<22} {n:>4}/{total:<4} ({share:5.1f}%)  '
            f'{mean_steps:>12.1f} '
            f'{statistics.mean(sw) if sw else float("nan"):>12.2f} '
            f'{statistics.mean(si) if si else float("nan"):>12.2f} '
            f'{statistics.mean(sl) if sl else float("nan"):>12.2f} '
            f'{statistics.mean(em) if em else float("nan"):>14.2f}'
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv-dir', type=Path, required=True,
                        help='directory containing <ckpt>_ep<N>.csv files')
    args = parser.parse_args()

    csv_files = sorted(args.csv_dir.glob('*_ep*.csv'))
    if not csv_files:
        print(f'no CSVs found under {args.csv_dir}')
        return

    summaries = [summarize_episode(p) for p in csv_files]
    print_per_episode(summaries)
    print_outcome_breakdown(summaries)


if __name__ == '__main__':
    main()
