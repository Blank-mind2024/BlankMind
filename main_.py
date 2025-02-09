import os
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'

from agent_ import *
from utils_ import *
from normlization import *
from replay_buffer import ReplayBuffer
import time


def main():
    agent = PPO(state_dim,
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
                 max_train_steps)
    ep_r = 0
    directory_result = 'PPO_1_18/model_save/'

    if Mode == 'train':
        print('Training Mode')
        replay_buffer = ReplayBuffer(batch_size,state_dim,action_dim)
        state_norm = Normalization(state_dim)                   # 状态归一化
        reward_scaling = RewardScaling(shape=1,gamma=gamma)  # 奖励缩放
        test_reward = 0   # 记录测试的奖励，如果奖励上升，则保存模型
        test_reward_old = -1e+8   # 记录测试的奖励，如果奖励上升，则保存模型

        #   ----------episode是一场完整的游戏，一局游戏结束后，重新开始新的一局游戏----------
        #   ------trajecotry可以是从一个episode中抽取的，也可以是多个episode的组合，包含了每一步的状态、动作、奖励、下一步的状态、是否结束等信息----------
        for i in range(1, num_episodes+1):
            state,_ = env.reset()
            state = state_norm(state)  
            reward = reward_scaling.reset()
            episode_steps = 0           # 记录每局游戏的步数
            done = False
            dw = False
            end = False
            # print_action = 0 # 打印动作
            while not done:         
                episode_steps += 1
                action, log_prob= agent.select_action(state)
                #  ----------添加探索噪声，使得agent更加鲁棒，探索更多的可能性----------
                # action = (action + np.random.normal(0,exploration_noise,size=env.action_space.shape[0])).clip(-max_action,max_action)
                
                next_state, reward, done, ter, _ = env.step(action)

                if done:  # done = True表示游戏结束，即agent赢得了这局游戏
                    dw = True

                if done or ter:  # ter = True表示游戏因为某种原因终止,还没结束
                    done = True
                    end = True
                
                next_state = state_norm(next_state)
                reward = reward_scaling(reward)
                replay_buffer.store(state, action, log_prob, reward, next_state, dw, end)
                ep_r += reward
                if replay_buffer.count == batch_size :
                    agent.update(replay_buffer)
                    replay_buffer.count = 0

                if done or episode_steps == max_episode_steps:
                    agent.writer.add_scalar('Reward', ep_r, global_step=i)
                    # if i % 10 == 0:
                    print('Ep: {}, Reward: {}, Steps: {}'.format(i, ep_r, episode_steps))
                    ep_r = 0
                #     break
                state = next_state
            
            if (i+1) % 200 == 0:
                for i in range (3):
                    s, _ = env_test.reset()
                    er = 0
                    done1 = False
                    ss = 0
                    while not done1:
                        ss +=1
                        s = state_norm(s)
                        s = torch.FloatTensor(s).unsqueeze(0).to(device)
                        a,_ = agent.actor(s)  # We use the deterministic policy during the evaluating
                        s_, r, done1,ter, _ = env_test.step(a.cpu().detach().numpy().flatten())
                        if ter:
                            done1 = True
                        er += r
                        s = s_
                    print(f'evaluate step {ss},episode_reward {er}')
                    test_reward = test_reward + er
                test_reward = test_reward/3
                print(f'test_reward {test_reward}')
                if test_reward > test_reward_old:
                    print('Save model')
                    test_reward_old = test_reward
                    directory_result = 'PPO_1_18/model_save/'
                    current_time = time.strftime("%Y%m%d-%H%M%S", time.localtime())
                    directory_result = os.path.join(directory_result, current_time)
                    agent.save(directory_result)
                    

    elif Mode == 'test': 
        print('Testing Mode')
        test_file = 'PPO_1_18/model_save/20250208-171356'
        agent.load(test_file)
        evaluate_reward = 0
        state_norm = Normalization(state_dim)
        for i in range(test_episodes):
            state, _ = env_test.reset()
            # state = state_norm(state)
            episode_steps = 0           # 记录每局游戏的步数
            done = False
            while not done:
                episode_steps += 1
                state = torch.FloatTensor(state).unsqueeze(0).to(device)
                action, log_prob= agent.actor(state)
                next_state, reward, done, _, _ = env_test.step(action.cpu().detach().numpy().flatten())
                env_test.render()
                evaluate_reward += reward
                if done or episode_steps == max_episode_steps:
                    print('Ep: {}, Reward: {:.2f}, Steps: {}'.format(i, evaluate_reward, episode_steps))
                    evaluate_reward = 0
                    break
                # state = state_norm(next_state)
                state = next_state

if __name__ == '__main__':
    main()