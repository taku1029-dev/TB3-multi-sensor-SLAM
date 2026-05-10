"""Roll out a saved SAC checkpoint on RLNavigation-v1, deterministic mode.

Use to inspect a learned policy's behaviour without further updates. Each
episode prints total reward, step count, and the terminated/truncated reason
exposed by ADR-010 / ADR-012.

    ros2 run rl_navigation_pkg sac_inference \\
        --checkpoint ./sac_runs/run_001/final.zip --episodes 5
"""

from __future__ import annotations

import argparse
from pathlib import Path

import gymnasium as gym
from stable_baselines3 import SAC

import rl_navigation_pkg.envs  # noqa: F401  registers RLNavigation-v0/v1


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True,
                   help='path to the SAC zip to load (e.g. .../final.zip)')
    p.add_argument('--episodes', type=int, default=5)
    p.add_argument('--deterministic', action='store_true', default=True,
                   help='use the policy mean instead of sampling (default)')
    p.add_argument('--stochastic', dest='deterministic', action='store_false',
                   help='sample actions from the policy distribution')
    return p.parse_args()


def main() -> None:
    args = parse_args()

    env = gym.make('RLNavigation-v1')
    model = SAC.load(str(args.checkpoint), env=env)

    try:
        for ep in range(args.episodes):
            obs, _info = env.reset()
            total_reward = 0.0
            step = 0
            while True:
                action, _state = model.predict(obs, deterministic=args.deterministic)
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += float(reward)
                step += 1
                if terminated or truncated:
                    break
            print(
                f'episode {ep:2d}  steps={step:3d}  reward={total_reward:+.2f}  '
                f'terminated={terminated}({info.get("terminated_reason")})  '
                f'truncated={truncated}({info.get("truncated_reason")})'
            )
    finally:
        env.close()


if __name__ == '__main__':
    main()
