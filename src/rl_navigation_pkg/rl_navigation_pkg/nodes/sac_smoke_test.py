"""Wiring smoke-test for the SAC trainer.

Runs SAC on RLNavigation-v1 for a small number of timesteps to confirm:
  - SB3 SAC initialises against our (15-dim obs, 2-dim action) spaces
  - env.step() / env.reset() drive the SAC training loop without crashing
  - learning_starts boundary is crossed cleanly (random → policy actions)
  - model.predict() returns valid actions on the final observation

Not a learning run — 200 steps is far too short for any meaningful policy.
For real training use `ros2 run rl_navigation_pkg sac_trainer ...`.
"""

from __future__ import annotations

import gymnasium as gym
from stable_baselines3 import SAC

import rl_navigation_pkg.envs  # noqa: F401  registers RLNavigation-v0/v1

TOTAL_TIMESTEPS = 200
LEARNING_STARTS = 50  # crosses the random→policy boundary inside the run


def main() -> None:
    env = gym.make('RLNavigation-v1')
    model = SAC(
        policy='MlpPolicy',
        env=env,
        learning_starts=LEARNING_STARTS,
        buffer_size=10_000,
        verbose=1,
    )

    print(f'starting SAC.learn for {TOTAL_TIMESTEPS} timesteps '
          f'(learning_starts={LEARNING_STARTS}); this is a wiring check, not training')
    try:
        model.learn(total_timesteps=TOTAL_TIMESTEPS, log_interval=2)

        obs, _ = env.reset()
        action, _ = model.predict(obs, deterministic=True)
        print(f'final deterministic action on reset obs: {action}')
    finally:
        env.close()


if __name__ == '__main__':
    main()
