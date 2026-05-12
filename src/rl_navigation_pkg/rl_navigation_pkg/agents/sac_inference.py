"""Roll out a saved SAC checkpoint on RLNavigation-v1, deterministic mode.

Use to inspect a learned policy's behaviour without further updates. Each
episode prints total reward, step count, and the terminated/truncated reason
exposed by ADR-010 / ADR-012.

With --log-dir, per-step σ + EKF/GT pose + error_l2 + goal + reward are
written to <log_dir>/<checkpoint_stem>_ep<N>.csv for offline failure-mode
analysis (consume with `failure_analysis --csv-dir`).

    ros2 run rl_navigation_pkg sac_inference \\
        --checkpoint ./sac_runs/phase1_001/final.zip --episodes 5

    ros2 run rl_navigation_pkg sac_inference \\
        --checkpoint ./sac_runs/phase1_001/final.zip --episodes 5 \\
        --log-dir ./sac_runs/phase1_001/inference/csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import gymnasium as gym
from stable_baselines3 import SAC

import rl_navigation_pkg.envs  # noqa: F401  registers RLNavigation-v0/v1


CSV_COLUMNS = [
    'step', 'sigma_wheel', 'sigma_imu', 'sigma_lidar',
    'ekf_x', 'ekf_y', 'gt_x', 'gt_y', 'error_l2',
    'goal_x', 'goal_y', 'reward',
    'nav_status_code', 'nav_status_label',
    'terminated_reason', 'truncated_reason',
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True,
                   help='path to the SAC zip to load (e.g. .../final.zip)')
    p.add_argument('--episodes', type=int, default=5)
    p.add_argument('--deterministic', action='store_true', default=True,
                   help='use the policy mean instead of sampling (default)')
    p.add_argument('--stochastic', dest='deterministic', action='store_false',
                   help='sample actions from the policy distribution')
    p.add_argument('--log-dir', type=Path, default=None,
                   help='if set, write per-step CSV per episode here')
    return p.parse_args()


def main() -> None:
    args = parse_args()

    env = gym.make('RLNavigation-v1')
    model = SAC.load(str(args.checkpoint), env=env)

    if args.log_dir is not None:
        args.log_dir.mkdir(parents=True, exist_ok=True)
    ckpt_stem = args.checkpoint.stem

    try:
        for ep in range(args.episodes):
            csv_file = None
            csv_writer = None
            csv_path = None
            if args.log_dir is not None:
                csv_path = args.log_dir / f'{ckpt_stem}_ep{ep}.csv'
                csv_file = csv_path.open('w', newline='')
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(CSV_COLUMNS)

            obs, _info = env.reset()
            total_reward = 0.0
            step = 0
            info: dict = {}
            terminated = truncated = False
            try:
                while True:
                    action, _state = model.predict(
                        obs, deterministic=args.deterministic
                    )
                    obs, reward, terminated, truncated, info = env.step(action)
                    total_reward += float(reward)
                    step += 1
                    if csv_writer is not None:
                        goal = info.get('goal_xy') or (None, None)
                        csv_writer.writerow([
                            info.get('step', step),
                            info.get('sigma_wheel'),
                            info.get('sigma_imu'),
                            info.get('sigma_lidar'),
                            info.get('ekf_x'),
                            info.get('ekf_y'),
                            info.get('gt_x'),
                            info.get('gt_y'),
                            info.get('error_l2'),
                            goal[0], goal[1],
                            reward,
                            info.get('nav_status_code'),
                            info.get('nav_status_label'),
                            info.get('terminated_reason'),
                            info.get('truncated_reason'),
                        ])
                    if terminated or truncated:
                        break
            finally:
                if csv_file is not None:
                    csv_file.close()

            suffix = f'  csv={csv_path}' if csv_path is not None else ''
            print(
                f'episode {ep:2d}  steps={step:3d}  reward={total_reward:+.2f}  '
                f'terminated={terminated}({info.get("terminated_reason")})  '
                f'truncated={truncated}({info.get("truncated_reason")})'
                f'{suffix}'
            )
    finally:
        env.close()


if __name__ == '__main__':
    main()
