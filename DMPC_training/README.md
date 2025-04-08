
# Safe Distributed Learning-Enhanced Predictive Control for Multiple Quadrupedal Robots -- Policy training

### Introduction

For this project, you will run the codes in the MATLAB environment. The codes verify the convergence condition of our algorithm and the closed-loop stability condition for quadruped robot navigation, ensuring obstacle avoidance and formation stability.
<div align="center">
  <img src="../assets/fig7.png" width="75%"/>
</div>


### Running the code

- Run `main`: Verify safe navigation in obstacle-dense environments.
- Run `MPC_CLF_CBF`: Validate convergence conditions during obstacle avoidance scenarios.
- Run `comparison_with_NN_formation_control`: Compare nonlinear solver-based Distributed Model Predictive Control (DMPC) with our method for multi-robot coordination in leader-follower formations with obstacle avoidance.
- Run `DMPC_formation_control_curved`: Simulate three-robot formation maintenance following a curved virtual leader trajectory while avoiding obstacles using Control Barrier Functions (CBFs) for safety.
- Run `DMPC_formation_control_straight`: Simulate three-robot formation maintenance following a straight virtual leader trajectory while avoiding obstacles using Control Barrier Functions (CBFs) for safety.