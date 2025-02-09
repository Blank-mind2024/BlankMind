# PPO算法实现Pendulumv1摆钟问题

>此代码文件运用`PPO`算法实现Pendulumv1摆钟问题
>
>相关环境说明见[Pendulum - Gym 文档](https://www.gymlibrary.dev/environments/classic_control/pendulum/)

#### :key:1.py——此文件是别人写好的，也是主要参考的文件

* `能找到别人的最好用别人的，新的想法直接在这基础上改就行`

________

#### :key:agent_.py——存储PPO算法核心

* 定义PPO算法的核心，并通过随机抽样的方式选取动作

____

#### :key:main_.py——主函数

* 包括两部分`train`和`test`

  train过程中每经过200轮测试一下训练效果,测试过程中选取神经网络输出的`mean`不经过随机抽样

____

#### :key:network.py——actor、critic网络

* [影响PPO算法性能的10个关键技巧（附PPO算法简洁Pytorch实现） - 知乎](https://zhuanlan.zhihu.com/p/512327050)

* [强化学习中的调参经验与编程技巧(on policy 篇) - 知乎](https://zhuanlan.zhihu.com/p/207435700)

  以上两个博客记录了PPO调参过程中的重要经验，可当作后续参考

___

#### :key:normlization.py——归一化

* 动态归一化程序，随着数据量的增加会逐步更新数据的均值和方差
* 奖励重塑需要在每一个epoisode之后reset一下

___

#### :key:replay_buffer.py——经验池

* 存储过去的经验，形成经验池和训练数据

___

#### :key:utils_.py——存储超参数​

* 一些超参数

#### 

#### 

