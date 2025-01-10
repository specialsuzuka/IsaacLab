'''
Author:wzm
Description:
this file is used to control the franka robot with a camera on the end-effector
written in interactive mode
'''



#lauch the app first

import argparse

from omni.isaac.lab.app import AppLauncher


# add argparse arguments
parser = argparse.ArgumentParser(description="this file is used to control the franka robot with a camera on the end-effector")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)



# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

#rest cfg

import torch

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass
#sensors
from omni.isaac.lab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns, TiledCameraCfg


#franka cfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.lab_assets import FRANKA_PANDA_CFG

@configclass
class FrankaSceneCfg(InteractiveSceneCfg):
    """Configuration for a franka scene."""

    # ground plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )
    table = AssetBaseCfg(
        prim_path="/World/Table",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 1.05)),

    )

    # articulation
    franka: ArticulationCfg = FRANKA_PANDA_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
        pos = (0.0, 0.0, 1.05),
        joint_pos = {
                # "panda_joint1": 0.0,
                # "panda_joint2": -0.785,
                # "panda_joint3": 0.0,
                "panda_joint4": -1.5708,
                # "panda_joint5": 0.0,
                # "panda_joint6": 1.571,
                # "panda_joint7": 0.785
            },  # 设置初始关节角度
        ),
    )


    
    # camera
    # camera = CameraCfg(
    # prim_path="{ENV_REGEX_NS}/Robot/panda_hand/front_cam",
    # update_period=0.1,
    # height=480,
    # width=640,
    # data_types=["rgb"],
    # spawn=sim_utils.PinholeCameraCfg(
    #     focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
    # ),

    # offset=CameraCfg.OffsetCfg(
    #         pos=(0.0, 0.0, 0.1),  # Adjust position relative to end-effector
    #         rot=(0.0, 0.0, 0.0, 1.0),  # Adjust rotation if necessary
    #         convention="ros",
    # ),
    # )

#     tiled_camera: TiledCameraCfg = TiledCameraCfg(
#         prim_path="{ENV_REGEX_NS}/Robot/panda_hand/front_cam",
#         offset=TiledCameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.1), rot=(0.0, 0.0, 0.0, 1.0), convention="ros"),
#         data_types=["rgb"],
#         spawn=sim_utils.PinholeCameraCfg(
#             focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
#         ),
#         width=80,
#         height=80,
# )

    


def run_simulator(sim: sim_utils.SimulationContext, scence: InteractiveScene):
    """Runs the simulation loop."""

    entity = scence["franka"]
    #define simulation step
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    #simulatio loop
    while simulation_app.is_running():
        # if count % 500 == 0:
        if count == 0:
            # Reset robot state
            root_state = entity.data.default_root_state.clone()
            root_state[:, :3] += scence.env_origins
            entity.write_root_link_pose_to_sim(root_state[:, :7])
            entity.write_root_com_velocity_to_sim(root_state[:, 7:])
            joint_pos, joint_vel = entity.data.default_joint_pos.clone(), entity.data.default_joint_vel.clone()
            #keep safe
            joint_pos = joint_pos.clamp_(
                entity.data.soft_joint_pos_limits[..., 0], entity.data.soft_joint_pos_limits[..., 1]
            )
            # print(joint_pos)
            # joint_pos += torch.rand_like(joint_pos) * 0.1
            entity.write_joint_state_to_sim(joint_pos, joint_vel)
            #clear internal buffers
            scence.reset()
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
            "franka_panda": torch.tensor([[0.0, 0.0, 0.0, -1.5708, 1.0, 1.8675, 0.0, 0.0, 0.0]]),  # example for 7 DoF
            # Add targets for other robots
        }

        joint_pos_target = custom_joint_targets["franka_panda"].to(sim.device)

        # joint_pos_current = entity.data.joint_pos

        # # Smooth transition between current and target positions
        # joint_pos_target = joint_pos_current + 0.1 * (joint_pos_target - joint_pos_current)
        actual_dof = entity.data.joint_pos.shape[1]  # Get the number of actual DoFs
        
        #3 keyboard input
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
        

        #...represent all dimensions
        joint_pos_target = joint_pos_target.clamp_(
        entity.data.soft_joint_pos_limits[..., 0], entity.data.soft_joint_pos_limits[..., 1]
    )

        #apply action
        entity.set_joint_position_target(joint_pos_target)

        #write data to sim
        scence.write_data_to_sim()

        
        # Step the simulation
        sim.step()
        sim_time += sim_dt
        count += 1
        scence.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    sim.set_camera_view([2.5, 0.0, 2.0], [0.0, 0.0, 1.0])

    #design the scene
    scene_cfg = FrankaSceneCfg(num_envs=1, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)

    #play the simulator
    sim.reset()

    #ready
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)

if __name__ == "__main__":
    main()
    simulation_app.close()
