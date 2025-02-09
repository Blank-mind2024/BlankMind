import torch
import gym


Mode = 'train'      # 'train' or 'test'

env = gym.make('Pendulum-v1')  # train时不需要渲染
env_test = gym.make('Pendulum-v1',render_mode='human')   # test时需要渲染

device = 'cuda' if torch.cuda.is_available() else 'cpu'

state_dim = env.observation_space.shape[0]
action_dim = env.action_space.shape[0]
max_action = float(env.action_space.high[0])


# 算法参数
lr_actor = 3e-4              # actor 学习率
lr_critic = 3e-4           # critic 学习率
gamma = 0.99                 # 奖励折扣因子
lamda = 0.95                 # GAE 系数
entropy_coef = 0.01        # 熵权重,网络探索程度
eps_clip = 0.2               # PPO 算法中的 epsilon 系数
hidden_dim = 128             # actor、critic 的隐藏层维度


# 训练参数
exploration_noise = 0.1      # 探索噪声
batch_size = 128            # 经验缓冲池大小
mini_batch_size = 128        # 训练时 mini-batch 的大小
use_lr = True               # 是否使用学习率衰减
max_train_steps= 5000     # 最大训练步数,用于计算学习率衰减
num_episodes = 5000    # 要进行的总游戏局数
save_interval = 1000         # 保存模型的间隔步数
max_episode_steps = 200      # 每局游戏的最大步数，pendulum-v1 最大步数为 200
K_epochs = 10                # 一个buffer训练 actor、critic 的次数


# 保存文件路径
directory_tensor = 'PPO_1_18/model_runs/'
directory_result = 'PPO_1_18/model_save/'


# 测试参数
test_episodes = 40           # 测试的游戏局数
