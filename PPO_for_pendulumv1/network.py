import torch.nn as nn
import torch


#  正交初始化
def orthogonal_init(module, gain=1):
    nn.init.orthogonal_(module.weight, gain=gain)
    nn.init.constant_(module.bias, 0)


# Beta初始化
class Actor_beta(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim):
        super(Actor_beta, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.alpha = nn.Linear(hidden_dim, action_dim)
        self.beta = nn.Linear(hidden_dim, action_dim)
        orthogonal_init(self.fc1)
        orthogonal_init(self.fc2)
        orthogonal_init(self.alpha, gain=0.01)
        orthogonal_init(self.beta, gain=0.01)

    def forward(self, x):
        x = nn.functional.tanh(self.fc1(x))
        x = nn.functional.tanh(self.fc2(x))
        alpha = nn.functional.softplus(self.alpha(x))+1.0
        beta = nn.functional.softplus(self.beta(x))+1.0
        return alpha, beta

    def get_dist(self, x):
        alpha, beta = self.forward(x)
        dist = torch.distributions.beta.Beta(alpha, beta)
        return dist  #### beta分布，输出0-1
    
        # beta_sample = beta_dist.sample()  # Beta 分布输出 [0, 1]
        # mapped_action = 2 * beta_sample - 1  # 映射到 [-1, 1]
        # scaled_action = mapped_action * max_action  # 缩放到 (-max_action, max_action)
    
    def mean(self, x):
        alpha, beta = self.forward(x)
        mean = alpha/(alpha+beta)
        return mean
   

class Actor_Gaussian(nn.Module):
    def __init__(self, state_dim, action_dim, max_action, hidden_dim):
        super(Actor_Gaussian, self).__init__()
        self.max_action = max_action
        self.fc1 = nn.Linear(state_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.mean = nn.Linear(128, action_dim)
        self.log_std = nn.Parameter(torch.zeros(action_dim))  # 使用nn.Parameter定义log_std
        orthogonal_init(self.fc1)
        orthogonal_init(self.fc2)
        orthogonal_init(self.mean, gain=0.01)
        # orthogonal_init(self.sigma, gain=0.01)

    def forward(self, x):
        x = nn.functional.tanh(self.fc1(x))
        x = nn.functional.tanh(self.fc2(x))
        mean = self.max_action * nn.functional.tanh(self.mean(x))
        sigma = torch.exp(self.log_std)
        return mean, sigma
    
    def get_dist(self, x):
        mean, sigma = self.forward(x)
        dist = torch.distributions.Normal(mean, sigma)
        return dist # 高斯分布
    


class Critic(nn.Module):
    def __init__(self, state_dim, hidden_dim=128):
        super(Critic, self).__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, 1)
        orthogonal_init(self.fc1)
        orthogonal_init(self.fc2)
        orthogonal_init(self.fc3)

    def forward(self, x):
        x = nn.functional.tanh(self.fc1(x))
        x = nn.functional.tanh(self.fc2(x))
        v_s = self.fc3(x)
        return v_s