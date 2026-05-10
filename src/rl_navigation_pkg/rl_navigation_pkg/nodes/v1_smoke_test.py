"""Smoke-test RLNavigation-v1 with random actions across two short episodes.

Two episodes (rather than one) verify the reset() path is reproducible —
catches buffer-not-clearing and reset-once-only bugs.

What to check in the output:
  - reset info shows gz_reset_ok=True and topics_ready=True
  - step 0's recv flags are ALL True (no boot lag, courtesy of A-3's
    _wait_for_initial_obs)
  - σ samples cover [0.05, 20] log-space
  - set_param_ok=True for every step (gate ack < 50 ms)
  - Block D not stuck at [1, 1, 1] now that COV_MAX = 10
  - reward is non-zero once /odom drifts from /ground_truth_pose
"""

import gymnasium as gym

import rl_navigation_pkg.envs  # noqa: F401  (registers RLNavigation-v0/v1)

EPISODES = 2
STEPS_PER_EPISODE = 25


def _step_log(i: int, action, reward: float, obs, info: dict) -> None:
    sw = info.get('sigma_wheel', float('nan'))
    si = info.get('sigma_imu', float('nan'))
    sp_ok = info.get('set_param_ok', False)
    recv = (
        f"scan={info['has_scan']:1} imu={info['has_imu']:1} "
        f"img={info['has_image']:1} odom={info['has_odom']:1} "
        f"gt={info['has_gt']:1}"
    )
    print(
        f'step={i:3d}  '
        f'a=[{action[0]:+.2f}, {action[1]:+.2f}]  '
        f'σ=({sw:6.3f},{si:6.3f}) ok={sp_ok!s:5}  '
        f'r={reward:+.3f}  '
        f'A={obs[0:4].round(3)}  '
        f'B={obs[4:6].round(3)}  '
        f'C={obs[6:12].round(3)}  '
        f'D={obs[12:15].round(3)}  '
        f'{recv}'
    )


def main() -> None:
    env = gym.make('RLNavigation-v1')

    for ep in range(EPISODES):
        print(f'\n=== episode {ep} ===')
        obs, info = env.reset()
        print(f'reset: obs.shape={obs.shape} dtype={obs.dtype}')
        print(f'  obs min={obs.min():+.3f} max={obs.max():+.3f} mean={obs.mean():+.3f}')
        print(f'  info={info}')

        for i in range(STEPS_PER_EPISODE):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            _step_log(i, action, reward, obs, info)
            if terminated or truncated:
                tr = info.get('terminated_reason')
                tc = info.get('truncated_reason')
                print(
                    f'  episode end at step {i}: '
                    f'terminated={terminated} ({tr}) '
                    f'truncated={truncated} ({tc})'
                )
                break

    env.close()


if __name__ == '__main__':
    main()
