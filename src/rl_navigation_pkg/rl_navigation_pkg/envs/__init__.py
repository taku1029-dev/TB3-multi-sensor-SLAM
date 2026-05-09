from gymnasium.envs.registration import register

register(
    id='RLNavigation-v0',
    entry_point='rl_navigation_pkg.envs.my_env:MyEnv',
)
