import torch

state = [1.0, 2.0, 0, -0.5, 0.7]
state_tensor = torch.tensor(state, dtype=torch.float32)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
state_tensor = state_tensor.to(device)

print(state_tensor)
