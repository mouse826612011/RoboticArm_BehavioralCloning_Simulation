# Simulation_RoboticArm_BehavioralCloning

<img src="figures/cheems.jpg" align="right"
     alt="Size Limit logo by Anton Lovchikov" width="350" height="500">

A simple simulation project on Behavioral Cloning of Robotic Arm, which is based on PyBullet + PyTorch and mainly consists of three parts:
- **Data collection** (using the keyboard to remotely control the Panda robotic arm in the PyBullet environment to complete the grasping task and collect data during the demonstration process);
- **Network training** (constructing a network based on PyTorch to learn the collected grasping data);
- **Network testing** (using the trained network to drive the Panda robotic arm in PyBullet to check whether it can achieve a similar effect to the demonstration).


