from gymnasium.envs.registration import register

register(
    id='my_pkg/MyEnv-v0',
    entry_point='my_pkg.envs.my_env:MyEnv',
)
