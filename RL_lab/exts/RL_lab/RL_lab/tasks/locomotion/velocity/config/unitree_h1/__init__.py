import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="RLLab-Isaac-Velocity-Rough-Unitree-H1-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:UnitreeH1RoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeH1RoughPPORunnerCfg",
    },
)

gym.register(
    id="RLLab-Isaac-Velocity-Flat-Unitree-H1-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:UnitreeH1FlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:UnitreeH1FlatPPORunnerCfg",
    },
)
