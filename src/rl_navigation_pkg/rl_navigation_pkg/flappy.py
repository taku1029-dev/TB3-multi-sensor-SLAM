import gymnasium as gym
import flappy_bird_gymnasium
from stable_baselines3 import PPO

# 1. 環境の作成
env = gym.make("FlappyBird-rgb-v0", render_mode="rgb_array")

# 2. モデルの定義 (PPOアルゴリズムを使用)
model = PPO("MlpPolicy", env, verbose=1)

# 3. 学習の実行
model.learn(total_timesteps=100000)

# 4. モデルの保存
model.save("ppo_flappybird")
