# Simulation_RoboticArm_BehavioralCloning

<img src="figures/cheems.jpg" align="right"
     alt="Size Limit logo by Anton Lovchikov" width="350" height="700">


A simple simulation project on Behavioral Cloning of Robotic Arm, LSTM +combined loss

which is based on PyBullet + PyTorch and mainly consists of three parts:
- **Data collection** (using the keyboard to control the Emika Panda robotic arm in the PyBullet environment to complete the object grasp task, and collect data during this process as the demonstration;
- **Network training** (constructing a network based on PyTorch to learn the demonstration);
- **Network testing** (using the trained network to drive the Panda robotic arm in PyBullet to check whether it can complete the task like the demonstration).


