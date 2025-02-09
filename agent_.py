import os
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
import time
from network import *
from utils_ import *
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
import torch.nn.functional as F

from torch.utils.tensorboard import SummaryWriter


class PPO(object):
    def __init__(self,
                 state_dim,
                 action_dim,
                 max_action,
                 lr_actor,
                 lr_critic,
                 gamma,
                 lamda,
                 K_epochs,
                 eps_clip,
                 hidden_dim,
                 batch_size,
                 mini_batch_size,
                 entropy_coef,
                 use_lr,
                 max_train_steps):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_action = max_action    
        self.lr_actor = lr_actor
        self.lr_critic = lr_critic
        self.gamma = gamma
        self.lamda = lamda
        self.K_epochs = K_epochs
        self.eps_clip = eps_clip
        self.hidden_dim = hidden_dim
        self.batch_size = batch_size
        self.mini_batch_size = mini_batch_size
        self.entropy_coef = entropy_coef # 熵权重
        self.use_lr = use_lr # 是否使用学习率衰减
        self.max_train_steps = max_train_steps
        directory_tensor = 'PPO_1_18/model_runs_test2/'
        current_time = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        directory_tensor = os.path.join(directory_tensor, current_time)
        self.writer = SummaryWriter(directory_tensor)

        # Network
        self.actor = Actor_Gaussian(self.state_dim, self.action_dim, self.max_action,self.hidden_dim).to(device)
        self.critic = Critic(self.state_dim, self.action_dim).to(device)
        
        # Optimizer
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=lr_actor, eps=1e-5)
        # self.actor_scheduler = torch.optim.lr_scheduler.StepLR(self.actor_optimizer, step_size=1000, gamma=0.9)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=lr_critic, eps=1e-5)

        self.num_actor_update_iteration = 0
        self.num_critic_update_iteration = 0

    def select_action(self, state):
        state = torch.FloatTensor(state.reshape(1, -1)).to(device)
        with torch.no_grad():
            dist = self.actor.get_dist(state)
            action = dist.sample()
            action_logprob = dist.log_prob(action)
        return action.cpu().numpy().flatten(), action_logprob.cpu().numpy().flatten()
    

    def update(self, replay_buffer, total_steps=1):
        state, action, action_logprob, reward, next_state, dw, done = replay_buffer.numpy_to_tensor()
       
        # ----------------------计算advantage--------------
        adv = []
        GAE = 0
        with torch.no_grad():
            vs = self.critic(state)
            vs_next = self.critic(next_state)
            deltas = reward + self.gamma * (1-dw) * vs_next - vs
            for delta, d in zip(reversed(deltas.flatten().cpu().numpy()), reversed(done.flatten().cpu().numpy())):
                GAE = delta + self.gamma * self.lamda * GAE * (1 - d)
                adv.insert(0, GAE)
            adv = torch.tensor(adv, dtype=torch.float).view(-1, 1).to(device)
            v_target = adv + vs

        # ----------------------归一化GAE-----------------
            # adv = (adv - adv.mean()) / (adv.std() + 1e-5)

       
        for _ in range(self.K_epochs):
            for index in BatchSampler(SubsetRandomSampler(range(len(state))), self.mini_batch_size, drop_last=False):
                # ---------------------优化Actor网络------------------
                dist_now = self.actor.get_dist(state[index])
                dist_entropy = dist_now.entropy().sum(1, keepdim=True) # keepdim : 与原始维度相同
                action_logprob_now = dist_now.log_prob(action[index])
                ratio = torch.exp(action_logprob_now.sum(1, keepdim=True) - action_logprob[index].sum(1, keepdim=True))
                surr1 = ratio * adv[index]
                surr2 = torch.clamp(ratio, 1 - self.eps_clip, 1 + self.eps_clip) * adv[index]
                actor_loss = torch.mean( -torch.min(surr1, surr2) - self.entropy_coef * dist_entropy).float()  # 策略熵
                self.writer.add_scalar('Loss/entropy_loss',dist_entropy.mean(),global_step=self.num_actor_update_iteration)
                self.writer.add_scalar('Loss/ppo_loss',-torch.min(surr1, surr2).mean(),global_step=self.num_actor_update_iteration)
                self.writer.add_scalar('Loss/actor_loss',actor_loss.mean(),global_step=self.num_actor_update_iteration)


                self.actor_optimizer.zero_grad()
                actor_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.actor.parameters(), 0.5)  #梯度剪裁为了防止训练过程中梯度爆炸
                self.actor_optimizer.step()
                # self.actor_scheduler.step()
                self.num_actor_update_iteration += 1


                # ---------------------优化Critic网络------------------
                v_s = self.critic(state[index])
                critic_loss = torch.mean(F.mse_loss(v_target[index],v_s)).float()
                self.writer.add_scalar('Loss/critic_loss',critic_loss.mean(),global_step=self.num_critic_update_iteration)
                
                self.critic_optimizer.zero_grad()
                critic_loss.backward()
                # torch.nn.utils.clip_grad_norm_(self.critic.parameters(), 0.5)
                self.critic_optimizer.step()
                self.num_critic_update_iteration += 1
        

        # ----------------------学习率更新------------------
        if self.use_lr:
            self.lr_change(total_steps)

    def lr_change(self, total_steps):
        lr_actor = self.lr_actor * (1 - total_steps / self.max_train_steps)
        lr_critic = self.lr_critic * (1 - total_steps / self.max_train_steps)
        for param_group in self.actor_optimizer.param_groups:
            param_group['lr'] = lr_actor
        for param_group in self.critic_optimizer.param_groups:
            param_group['lr'] = lr_critic


    def save(self, filename):
        torch.save(self.actor.state_dict(), filename + "_actor")
        torch.save(self.critic.state_dict(), filename + "_critic")
        print("Saved model to", filename)

    def load(self, filename):
        self.actor.load_state_dict(torch.load(filename + "_actor"))
        self.critic.load_state_dict(torch.load(filename + "_critic"))
        print("Loaded model from", filename)
