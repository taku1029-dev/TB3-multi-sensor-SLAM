"""SAC trainer for RLNavigation-v1.

Phase 1 only — 2-dim action (sigma_wheel, sigma_imu). Phase 2/3 head expansion
+ LR warmup (ADR-009:54-57) is deferred until Phase 1 converges.

Convergence criterion (ADR-010:34-40, rolling 50-episode mean reward > −5 +
100-episode plateau < 5%) is currently operator-checked via TensorBoard, not
automated. Adding the check as a custom BaseCallback is cheap once Phase 1
data shows the right empirical thresholds.

Run inside the ROS 2 container:
    ros2 run rl_navigation_pkg sac_trainer \\
        --total-timesteps 100000 --save-dir ./sac_runs/run_001
"""

from __future__ import annotations

import argparse
from pathlib import Path

import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

import rl_navigation_pkg.envs  # noqa: F401  registers RLNavigation-v0/v1


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--total-timesteps', type=int, default=100_000)
    p.add_argument('--save-dir', type=Path, default=Path('./sac_runs/run_001'))
    p.add_argument('--checkpoint-interval', type=int, default=5_000,
                   help='env steps between checkpoints')
    p.add_argument('--learning-starts', type=int, default=1_000,
                   help='steps of pure random exploration before SAC updates')
    p.add_argument('--buffer-size', type=int, default=100_000)
    p.add_argument('--batch-size', type=int, default=256)
    p.add_argument('--learning-rate', type=float, default=3e-4)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--log-interval', type=int, default=4,
                   help='SB3 log frequency in episodes')
    p.add_argument('--resume-from', type=Path, default=None,
                   help='path to a saved SAC zip to continue training from')
    return p.parse_args()


def main() -> None:
    args = parse_args()
    args.save_dir.mkdir(parents=True, exist_ok=True)
    tb_dir = args.save_dir / 'tb'
    ckpt_dir = args.save_dir / 'checkpoints'

    env = gym.make('RLNavigation-v1')

    if args.resume_from is not None:
        model = SAC.load(args.resume_from, env=env, tensorboard_log=str(tb_dir))
    else:
        model = SAC(
            policy='MlpPolicy',
            env=env,
            learning_rate=args.learning_rate,
            buffer_size=args.buffer_size,
            learning_starts=args.learning_starts,
            batch_size=args.batch_size,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            ent_coef='auto',
            verbose=1,
            seed=args.seed,
            tensorboard_log=str(tb_dir),
        )

    checkpoint_cb = CheckpointCallback(
        save_freq=args.checkpoint_interval,
        save_path=str(ckpt_dir),
        name_prefix='sac',
    )

    try:
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=checkpoint_cb,
            log_interval=args.log_interval,
        )
        model.save(args.save_dir / 'final')
    finally:
        env.close()


if __name__ == '__main__':
    main()
