
# Safe Distributed Learning-Enhanced Predictive Control for Multiple Quadrupedal Robots -- Simulated Experiments

### Introduction

The proposed planner has been evaluated in [NVIDIA Isaac_Sim](https://developer.nvidia.com/isaac/sim) and tested on our designed quadrupedal robot, XG, using a pre-trained RL locomotion policy from the Orbit Framework. A detailed guide on training and deploying the pre-trained RL locomotion policy, along with the USD file of our quadrupedal robot, is provided in an external document. Our hierarchical control architecture is implemented and trained within this simulated environment. We selected the NVIDIA warehouse environment (approximately 25 × 35m), where cuboid obstacles create narrow corridors. Our experiments focus on formation maintenance, collision avoidance, and deadlock resolution in these challenging scenarios.

### Running the code

# Running the Code

## 1. Copy the Python Files  
1. Open **Home** and enable hidden folders (if not already visible).  
2. Navigate to:  
```bash
Home/.local/share/ov/pkg/isaac-sim-4.20/extension_example
```

3. Locate and open **user_examples**.  
4. Copy and paste the Python files from **Legged_Robots** into this directory.  
5. For the **Camera** project, follow the same path and replace the **hello_world** files with the corresponding Python files.  

### Launch in Isaac Sim  
1. Start **Isaac Sim**.  
2. In the top-left menu, go to **Isaac Examples**.  
3. Select **Legged Robot**, then click **Load**.  
4. For the **Camera** project, select **hello_world**, then click **Load**.  

---

### References  
- [Isaac Sim Extension Templates](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_extension_templates.html)  
- [Isaac Sim Core API Tutorial - Hello World](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html)  

