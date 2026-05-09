"""Smoke-test the RLNavigation env by stepping random actions for a few episodes."""

import gymnasium as gym

import rl_navigation_pkg.envs  # noqa: F401  (registers RLNavigation-v0)


def main() -> None:
    env = gym.make('RLNavigation-v0')
    obs, info = env.reset()
    print(f'reset: obs.shape={obs.shape} info={info}')

    for i in range(50):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(
            f'step={i} action={action} reward={reward:.2f} '
            f'obs.min={obs.min():.2f} obs.max={obs.max():.2f} '
            f'term={terminated} trunc={truncated} info={info}'
        )
        if terminated or truncated:
            obs, info = env.reset()

    env.close()


if __name__ == '__main__':
    main()
