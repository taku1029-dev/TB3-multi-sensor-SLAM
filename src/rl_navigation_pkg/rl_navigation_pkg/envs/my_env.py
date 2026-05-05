import gymnasium as gym

# 1. 環境の作成
# render_mode="human" を指定すると、ウィンドウが表示されます
env = gym.make("CartPole-v1", render_mode="human")

# 2. 環境の初期化
observation, info = env.reset()

# 3. 学習ループ（ここではランダムな行動をとる）
for _ in range(1000):
    # 環境を描画
    env.render()
    
    # アクションをランダムに選択 (0: 左, 1: 右)
    action = env.action_space.sample()
    
    # ステップを進める
    # observation: 次の状態, reward: 報酬, terminated: 終了, truncated: 切り捨て, info: 追加情報
    observation, reward, terminated, truncated, info = env.step(action)
    
    # 終了フラグが立ったらリセット
    if terminated or truncated:
        observation, info = env.reset()

# 4. 環境を閉じる
env.close()
