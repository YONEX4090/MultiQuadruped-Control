import carb
import numpy as np
import omni
import math
import omni.kit.viewport.utility as viewport_utility

from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.quadruped.robots import DOGFlatTerrainPolicy
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from casadi import SX, vertcat, Function, nlpsol
import torch
import torch.nn as nn
import torch.nn.functional as F


# Enable ROS2 bridge extension
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

cbf_model_path = "models/cbf_model.pt"
cbf_model = torch.load(cbf_model_path)
cbf_model.eval()

# Define IPOPT-based MPC-CLF solver
class MPC_CLF_Solver:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        self._setup_solver()
    
    def _setup_solver(self):
        x = SX.sym('x', 4)  # State: [px, py, vx, vy]
        u = SX.sym('u', 2)  # Control input: [ax, ay]
        x_next = x + vertcat(x[2], x[3], u[0], u[1]) * self.dt
        
        cost = x_next[:2].T @ x_next[:2]  # Quadratic cost function for tracking
        nlp = {'x': u, 'f': cost, 'g': x_next}
        self.solver = nlpsol('solver', 'ipopt', nlp, {'ipopt.print_level': 0, 'print_time': 0})
    
    def solve(self, state, desired_state):
        lbx = np.array([-1.0, -1.0])  # Control limits
        ubx = np.array([1.0, 1.0])
        sol = self.solver(x0=np.zeros(2), lbx=lbx, ubx=ubx, lbg=-np.inf, ubg=np.inf)
        return sol['x'].full().flatten()

mpc_clf_solver = MPC_CLF_Solver()

def pooled_observation(neighbors):
    if len(neighbors) == 0:
        return np.zeros(4)  # Return a neutral observation if no neighbors exist
    neighbors_tensor = torch.tensor(neighbors, dtype=torch.float32)
    pooled = torch.max(neighbors_tensor, dim=0)[0].numpy()
    return pooled

def compute_cbf_correction(state, neighbors, weight=0.8):
    pooled_neighbors = pooled_observation(neighbors)
    state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
    neighbors_tensor = torch.tensor(pooled_neighbors, dtype=torch.float32).unsqueeze(0)
    h_value = cbf_model(state_tensor, neighbors_tensor).detach().numpy().squeeze()
    correction = np.clip(h_value * weight, -1.0, 1.0)
    return correction

def compute_mpc_clf_control(state, desired_state, velocity_limit=1.0):
    control_input = mpc_clf_solver.solve(state, desired_state)
    return np.clip(control_input, -velocity_limit, velocity_limit)

def detect_deadlock(velocity, clf_cost, cbf_violation_threshold=0.01):
    if np.linalg.norm(velocity) < 0.05 and clf_cost > cbf_violation_threshold:
        return True
    return False

def resolve_deadlock(state, leader_position):
    perturbation = np.random.uniform(-0.1, 0.1, size=2)
    adjusted_state = state + perturbation
    return np.clip(adjusted_state, leader_position - 2.0, leader_position + 2.0)

class DistributedMPCController():
    def __init__(self, leader_robot, follower_robots, formation_offset):
        self.leader_robot = leader_robot
        self.follower_robots = follower_robots
        self.formation_offset = formation_offset

    def update_commands(self, leader_position, leader_target, obstacles):
        position_error = leader_target - leader_position
        leader_command = compute_mpc_clf_control(leader_position, leader_target)
        commands = [np.clip(leader_command, -1.0, 1.0)]

        for i, follower in enumerate(self.follower_robots):
            desired_position = leader_position + self.formation_offset[i]
            follower_position, _ = follower.DOG.robot.get_world_pose()

            # Compute base control command using MPC-CLF
            command = compute_mpc_clf_control(follower_position, desired_position)
            clf_cost = np.linalg.norm(command)

            # Apply CBF-based correction with pooled observations
            state = np.array(follower_position[:2])
            neighbors = [leader_position[:2]] + [r.DOG.robot.get_world_pose()[0][:2] for r in self.follower_robots if r != follower]
            cbf_correction = compute_cbf_correction(state, np.array(neighbors))
            command[0] += cbf_correction[0] * 0.8
            command[1] += cbf_correction[1] * 0.8

            # Deadlock detection and resolution
            if detect_deadlock(command, clf_cost):
                adjusted_position = resolve_deadlock(follower_position[:2], leader_position[:2])
                command = compute_mpc_clf_control(adjusted_position, desired_position)

            commands.append(command)
        
        return commands
    
class SingleRobot():
    def __init__(self, pos, name):
        self.DOG = DOGFlatTerrainPolicy(
            prim_path="/World/" + name,
            name=name,
            position=pos,
        )
        return


class FormationController():
    def __init__(self, leader_robot, follower_robots, formation_offset):
        self.leader_robot = leader_robot
        self.follower_robots = follower_robots
        self.formation_offset = formation_offset  # List of offsets for each follower robot

    def update_commands(self, leader_position, leader_target):
        """
        Computes movement commands for the leader and follower robots.
        """
        position_error = leader_target - leader_position
        leader_speed = np.clip(0.5 * position_error[:2], -1.0, 1.0)  # Limit max speed
        leader_yaw = math.atan2(position_error[1], position_error[0])  # Compute target direction
        leader_command = [leader_speed[0], leader_speed[1], 0.1 * leader_yaw]

        commands = [leader_command]

        for i, follower in enumerate(self.follower_robots):
            desired_position = leader_position + self.formation_offset[i]
            follower_position, _ = follower.DOG.robot.get_world_pose()
            position_error = desired_position - follower_position

            # Simple proportional controller for following the leader
            command = [
                1.0 * position_error[0],  
                1.0 * position_error[1],  
                0.0 * math.atan2(position_error[1], position_error[0])
            ]
            commands.append(command)
        
        return commands


class LeggedRobot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 500.0
        self._world_settings["rendering_dt"] = 10.0 / 500.0
        self._time = 0.0
        return

    def setup_scene(self):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        # Spawn warehouse scene
        prim = define_prim("/World/Warehouse", "Xform")
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        prim.GetReferences().AddReference(asset_path)

        self.obstacles = []
        obstacle_positions = [
            np.array([0.0, -2.0, 0.0]),
            np.array([1.0, -2.0, 0.0]),
            np.array([2.0, -2.0, 0.0]),
            np.array([0.0, 1.5, 0.0]),
            np.array([1.0, 1.5, 0.0]),
            np.array([2.0, 1.5, 0.0]),
            np.array([2.0, -1, 0.0]),
            np.array([-1.0, -2.5, 0.0]),
            np.array([-1.0, 2.0, 0.0]),
            np.array([5.0, -0.5, 0.0]),
        ]

        for i, pos in enumerate(obstacle_positions):
            obstacle = self._world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Obstacle_{i}",
                    name=f"Obstacle_{i}",
                    position=pos,
                    scale=np.array([0.5, 0.5, 0.5]),
                    color=np.array([1.0, 0.0, 0.0])
                )
            )
            self.obstacles.append(obstacle)

        # Initialize robots
        self.firstRobot = SingleRobot(np.array([0, 0, 0.5]), "DOG1")
        self.secondRobot = SingleRobot(np.array([-2.0, 0, 0.5]), "DOG2")
        self.thirdRobot = SingleRobot(np.array([-5, 0, 0.5]), "DOG3")

        # Set default joint state
        self.firstRobot.DOG.robot.set_joints_default_state(self.firstRobot.DOG._default_joint_pos)
        self.secondRobot.DOG.robot.set_joints_default_state(self.secondRobot.DOG._default_joint_pos)
        self.thirdRobot.DOG.robot.set_joints_default_state(self.thirdRobot.DOG._default_joint_pos)

        # Create formation controller
        formation_offset = [
            np.array([-1.70, 1.3, 0.0]),
            np.array([-1.70, -1.3, 0.0])
        ]
        self.formation_controller = FormationController(self.firstRobot, [self.secondRobot, self.thirdRobot], formation_offset)

        return

    async def setup_post_load(self):
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._world.add_physics_callback("physics_step", callback_fn=self.on_physics_step)
        self._physics_ready = False
        await self._world.play_async()
        self.firstRobot.DOG.initialize()
        self.secondRobot.DOG.initialize()
        self.thirdRobot.DOG.initialize()

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self)-> None:
        await self._world.play_async()
        self._physics_ready = False
        self.firstRobot.DOG.initialize()
        self.secondRobot.DOG.initialize()
        self.thirdRobot.DOG.initialize()

    def on_physics_step(self, step_size) -> None:
        # Define leader's target point
        leader_target = np.array([8.0, 0.0, 0.0])  # Target position
        
        # Get leader's current position
        leader_position, _ = self.firstRobot.DOG.robot.get_world_pose()
        
        if self._physics_ready:
            self._time += step_size

            # Compute movement commands for all robots
            commands = self.formation_controller.update_commands(leader_position, leader_target)

            # Move all robots
            for i, robot in enumerate([self.firstRobot, self.secondRobot, self.thirdRobot], start=0):
                joint_targets = commands[i]

                # Obstacle avoidance (basic implementation)
                for obstacle in self.obstacles:
                    obstacle_position, _ = obstacle.get_world_pose()
                    robot_position = robot.DOG.robot.get_world_pose()[0]
                    distance_to_obstacle = np.linalg.norm(robot_position[:2] - obstacle_position[:2])
                    if distance_to_obstacle < 1.15:
                        avoidance_vector = robot_position[:2] - obstacle_position[:2]
                        avoidance_vector /= np.linalg.norm(avoidance_vector)
                        scaling_factor = min(2.0, 3.5 / distance_to_obstacle)
                        joint_targets[0] += scaling_factor * avoidance_vector[0]
                        joint_targets[1] += scaling_factor * avoidance_vector[1]

                robot.DOG.advance(step_size, joint_targets)

                # Print robot states
                position, _ = robot.DOG.robot.get_world_pose()
                print(f"DOG{i + 1} - Position: {position}")
        else:
            self._physics_ready = True

    def _timeline_timer_callback_fn(self, event) -> None:
        if self.firstRobot.DOG and self.secondRobot.DOG and self.thirdRobot.DOG:
            self.firstRobot.DOG.post_reset()
            self.secondRobot.DOG.post_reset()
            self.thirdRobot.DOG.post_reset()
            self._physics_ready = False

    def world_cleanup(self):
        self._event_timer_callback = None
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")
        return
