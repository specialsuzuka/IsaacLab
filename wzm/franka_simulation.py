# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates a single 7-DOF Franka Panda manipulator on a table.

Usage:
    ./isaaclab.sh -p source/standalone/demos/franka_on_table.py
"""

"""Launch Isaac Sim Simulator first."""

import argparse
from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates a single 7-DOF Franka Panda manipulator on a table.")
# append AppLauncher CLI args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch Omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest of the script."""

import torch
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab_assets import FRANKA_PANDA_CFG


import pygame


def design_scene() -> tuple[dict, list[float]]:
    """Designs a simple scene with a table and a Franka Panda arm."""
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)

    # Lights
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Define the origin for the environment
    origin = [0.0, 0.0, 0.0]

    # Create a table at the origin
    # prim_utils.create_prim("/World/Table", "Xform", translation=origin)
    # cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd")
    # cfg.func("/World/Table", cfg, translation=(0.0, 0.0, 1.05))



    # # Add Franka Panda arm on the table
    # franka_cfg = FRANKA_PANDA_CFG.replace(prim_path="/World/Robot")
    # franka_cfg.init_state.pos = (0.0, 0.0, 1.05)
    # franka_panda = Articulation(cfg=franka_cfg)

    prim_utils.create_prim("/World/Origin1", "Xform", translation=origin)
    # -- Table
    cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd")
    cfg.func("/World/Origin1/Table", cfg, translation=(0.55, 0.0, 1.05))
    # -- Robot
    franka_arm_cfg = FRANKA_PANDA_CFG.replace(prim_path="/World/Origin1/Robot")
    franka_arm_cfg.init_state.pos = (0.0, 0.0, 1.05)
    franka_panda = Articulation(cfg=franka_arm_cfg)



    # Create a camera on the end-effector of the Franka Panda
    camera_pos = [0.0, 0.0, 1.5]  # Positioning the camera at the end-effector
    camera = Camera(
        name="EndEffectorCamera",
        parent=franka_panda,
        translation=camera_pos
    )


    # Return the robot entity and the environment origin
    return {"franka_panda": franka_panda}, origin


def run_simulator(sim: sim_utils.SimulationContext, entity: Articulation, origin: torch.Tensor):
    """Runs the simulation loop."""
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    while simulation_app.is_running():
        if count % 1000 == 0:
            # Reset robot state
            root_state = entity.data.default_root_state.clone()
            root_state[:, :3] += origin
            entity.write_root_link_pose_to_sim(root_state[:, :7])
            entity.write_root_com_velocity_to_sim(root_state[:, 7:])
            joint_pos, joint_vel = entity.data.default_joint_pos.clone(), entity.data.default_joint_vel.clone()
            entity.write_joint_state_to_sim(joint_pos, joint_vel)
            entity.reset()
            print("[INFO]: Resetting robot state...")


        # Apply random joint actions for demonstration
        '''
        #1随机动作
        joint_pos_target = entity.data.default_joint_pos + torch.randn_like(entity.data.joint_pos) * 0.1
        #clamp_函数用于限制关节在安全范围内
        joint_pos_target = joint_pos_target.clamp_(
            entity.data.soft_joint_pos_limits[..., 0], entity.data.soft_joint_pos_limits[..., 1]
        )
        #部署角度
        entity.set_joint_position_target(joint_pos_target)
        #数据到模拟
        entity.write_data_to_sim()
        '''




        #2固定动作
        custom_joint_targets = {
            "franka_panda": torch.tensor([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 1]]),  # example for 7 DoF
            # Add targets for other robots
        }

        joint_pos_target = custom_joint_targets["franka_panda"].to(sim.device)

        # joint_pos_current = entity.data.joint_pos

        # # Smooth transition between current and target positions
        # joint_pos_target = joint_pos_current + 0.1 * (joint_pos_target - joint_pos_current)
        actual_dof = entity.data.joint_pos.shape[1]  # Get the number of actual DoFs



        '''
        
        joint_pos_current = entity.data.joint_pos
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    joint_pos_current[0][0] += 0.2 # Increment joint 0
                    joint_pos_target = joint_pos_current 
                    print("UP")
                elif event.key == pygame.K_DOWN:
                    joint_pos_current[0][0] -= 0.2
                    joint_pos_target = joint_pos_current  # Decrement joint 0
            joint_pos_target = joint_pos_current
        
        '''
        #3 keyboard input

        


        #...represent all dimensions
        joint_pos_target = joint_pos_target.clamp_(
        entity.data.soft_joint_pos_limits[..., 0], entity.data.soft_joint_pos_limits[..., 1]
    )
        





        #apply action
        entity.set_joint_position_target(joint_pos_target)

        #write data to sim
        entity.write_data_to_sim()

        
        # Step the simulation
        sim.step()
        sim_time += sim_dt
        count += 1
        entity.update(sim_dt)



        


def main():

    #3keyboard_control
    # pygame.init()
    # screen = pygame.display.set_mode((400, 300))

    #initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([2.5, 0.0, 2.0], [0.0, 0.0, 1.0])

    # Design the scene
    entities, origin = design_scene()
    origin_tensor = torch.tensor(origin, device=sim.device)


    sim.reset()
    print("[INFO]: Setup complete...")
    
    # Run the simulation loop
    run_simulator(sim, entities["franka_panda"], origin_tensor)


if __name__ == "__main__":
    main()
    simulation_app.close()
