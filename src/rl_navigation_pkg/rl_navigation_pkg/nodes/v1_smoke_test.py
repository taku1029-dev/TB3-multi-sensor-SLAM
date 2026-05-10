"""Smoke-test RLNavigation-v1 with random actions.

Mirrors env_smoke_test (v0) but exercises the 15-dim observation, the
2-dim Box action, and the verified reward subtrahend pipeline. Runs 50
random steps and prints obs blocks A/B/C/D + reward + topic-receipt flags
so the operator can confirm:
  - obs is shape (15,) float32 and within [-1, 1]
  - reward is non-zero once /odom and /ground_truth_pose are both flowing
  - all five topic-receipt flags become True within the first second

In A-1 the action is dropped inside SACEnv.step; reward and observation
are still meaningful because release_driver clocks the gate independently.
"""

import gymnasium as gym

import rl_navigation_pkg.envs  # noqa: F401  (registers RLNavigation-v0/v1)


def main() -> None:
    env = gym.make('RLNavigation-v1')
    obs, info = env.reset()
    print(f'reset: obs.shape={obs.shape} dtype={obs.dtype}')
    print(f'  obs min={obs.min():+.3f} max={obs.max():+.3f} mean={obs.mean():+.3f}')
    print(f'  info={info}')

    for i in range(50):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
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
        if terminated or truncated:
            print(f'  episode end at step {i}: terminated={terminated} truncated={truncated}')
            break

    env.close()


if __name__ == '__main__':
    main()
