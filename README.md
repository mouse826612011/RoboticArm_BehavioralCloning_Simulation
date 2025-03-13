# Simulation of LSTM-based Behavioral Cloning for Robotic Arm.

<img src="figures/cheems.jpg" align="right"
     alt="Size Limit logo by Anton Lovchikov" width="350" height="700">


***Background***: Cheems_JH's personal research mainly focuses on machine learning-based low-level learning from demonstration (like DMP). However, modern learning from demonstration also relies on advanced neural network techniques such as behavior cloning or inverse reinforcement learning. Therefore, after gaining rough understandings about behavior cloning and neural network, Cheems_JH aims to implement a behavior cloning-based robotic arm learning from demonstration in simulations. The general objective is to teleoperate the robotic arm to perform a grasping task and obtain the demo, then use behavior cloning to imitate the demo.

***Environment***: 2024 MacBook Air M3, pycharm + python 3.12 + pytorch 2.5.1 + pyBullet 3.25.
Since the following codes does not use complex libraries or functions, similar versions of PyTorch and PyBullet should also be compatible. 
As for other libraries, such as those required for interpolation (*scipy*) or excel processing (*openpyxl*), you can install them according to the imports part of each code.

***General Introduction***:
- **Data collection** (using the keyboard to control the Emika Panda robotic arm in the PyBullet environment to complete the object grasp task, and collect data during this process as the demonstration;
- **Network training** (constructing a network based on PyTorch to learn the demonstration);
- **Network testing** (using the trained network to drive the Panda robotic arm in PyBullet to check whether it can complete the task like the demonstration).


