# Simulation of LSTM-based Behavioral Cloning for Robotic Arm.

<img src="figures/cheems.jpg" align="right"
     alt="Size Limit logo by Anton Lovchikov" width="250" height="500">


***Background***: Cheems_JH's personal research mainly focuses on machine learning-based low-level learning from demonstration (like DMP). However, modern learning from demonstration also relies on advanced neural network techniques such as behavior cloning or inverse reinforcement learning. Therefore, after gaining rough understandings about behavior cloning and neural network, Cheems_JH aims to implement a behavior cloning-based robotic arm learning from demonstration in simulations. The general objective is to teleoperate the robotic arm to perform a grasping task and obtain the demo, then use behavior cloning to imitate the demo.

***Environment***: 2024 MacBook Air M3, pycharm + python 3.12 + pytorch 2.5.1 + pyBullet 3.25.
Since the following codes does not use complex libraries or functions, similar versions of PyTorch and PyBullet should also be compatible. 
As for other libraries, such as those required for interpolation (*scipy*) or excel processing (*openpyxl*), you can install them according to the imports part of each code.

***General Introduction***: This project mainly consists of the following three parts:
- **Data collection** (using the keyboard to control the Emika Panda robotic arm in the PyBullet environment to complete the object grasp task, and collect data during this process as the demonstration;
- **Network training** (constructing a network based on PyTorch to learn the demonstration);
- **Network testing** (using the trained network to drive the Panda robotic arm in PyBullet to check whether it can complete the task like the demonstration).


## 0. Problem Formulation
- Behavior cloning can be considered the most fundamental method of using neural networks for learning from demonstration. Its goal is to find the mapping between demonstrated state-action pairs through supervised learning.
- Specifically, the demonstrations $X$ can be described as sequences of states $s$ and actions $a$, $X={(s_1,a_1), (s_2,a_2), ... , (s_n,a_n)}$. Then the behavior cloning based on neural networks is to trained the mapping of $S \to A$. Once trained, the network can predict the required action based on the current state, enabling the agent to imitate the demonstration.

However, this approach also has its limitations.

行为克隆应该是最基础且见简单的使用神经网络进行演示学习的方法，其目的是模仿演示的策略检测。
具体来说，行为克隆，本质上是状态-动作的策略监督学习
但这说明也有局限性


## 1. Get Demonstrations in Simulation
## 2. Behavioral Cloning of Demonstrations based on Neural Networks
## 3. Test the Trained Behavioral Cloning Model in Simulation
