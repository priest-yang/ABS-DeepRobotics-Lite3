# Compared to play.py, this script adds a depth camera to the environment and uses a ResNet model to predict the depth map from the camera image.
# You have to run train_depth_resnet.py to train the ResNet model before running this script.

from legged_gym import LEGGED_GYM_ROOT_DIR
import os
import time
import isaacgym
from legged_gym.envs import *
from legged_gym.utils import  get_args, export_policy_as_jit, task_registry
from legged_gym.scripts.train_depth_resnet import ResNetModel
import numpy as np
import torch
import time

import cv2

EXPORT_POLICY = False
RECORD_FRAMES = False
MOVE_CAMERA = False

# load ResNet model
ResNetModel_dir = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', 'depth_cam')
print('ResNetModel_dir: ', ResNetModel_dir)
ResNetModels = sorted(os.listdir(ResNetModel_dir))
newest_model_path = os.path.join(ResNetModel_dir, ResNetModels[-1])

def normalize_depth_image(depth_image):
    depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_normalized = np.uint8(depth_normalized)
    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)  # Try HOT, MAGMA, or PLASMA
    return depth_colored


def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)
    # overwrite some parameters for testing
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 1)
    env_cfg.terrain.num_rows = 2
    env_cfg.terrain.num_cols = 2
    env_cfg.terrain.curriculum = False
    # env_cfg.terrain.mesh_type = "plane"
    env_cfg.noise.add_noise = False
    env_cfg.domain_rand.randomize_friction = False
    env_cfg.domain_rand.push_robots = True
    env_cfg.domain_rand.max_push_vel_xy = 0.0
    env_cfg.domain_rand.randomize_dof_bias = False
    env_cfg.domain_rand.erfi = False
    env_cfg.domain_rand.randomize_base_mass = True
    env_cfg.domain_rand.added_mass_range = [0, 0]
    env_cfg.domain_rand.randomize_timer_minus = 0.0
    
    # add depth camera
    env_cfg.sensors.depth_cam.enable = True
    
    # load Ray-Prediction Network
    print('Loading Ray-Prediction Network from: ', newest_model_path)
    model = torch.jit.load(newest_model_path)
    model.to('cuda') if torch.cuda.is_available() else model.to('cpu')
    model.eval()
    
    # prepare environment

    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    env.debug_viz = False
    obs = env.get_observations()
    env.terrain_levels[:] = 9
    # load policy
    train_cfg.runner.resume = True
    ppo_runner, train_cfg = task_registry.make_alg_runner(env=env, name=args.task, args=args, train_cfg=train_cfg)
    policy = ppo_runner.get_inference_policy(device=env.device)
    exported_policy_name = str(task_registry.loaded_policy_path.split('/')[-2]) + str(task_registry.loaded_policy_path.split('/')[-1])
    print('Loaded policy from: ', task_registry.loaded_policy_path)
    # export policy as a jit module (used to run it from C++)
    if EXPORT_POLICY:
        path = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'policies')
        export_policy_as_jit(ppo_runner.alg.actor_critic, path, exported_policy_name)
        print('Exported policy as jit script to: ', os.path.join(path, exported_policy_name))

    camera_position = np.array(env_cfg.viewer.pos, dtype=np.float64)
    camera_vel = np.array([1., 1., 0.])
    camera_direction = np.array(env_cfg.viewer.lookat) - np.array(env_cfg.viewer.pos)
    img_idx = 0

    for i in range(20*int(env.max_episode_length)):
        time.sleep(0.02)

        # depth -> ray
        cam_data = env.cam_obs.detach()
        
        # breakpoint()
        
        #! Visualization of the depth camera output
        cv_input = cam_data.squeeze().cpu().numpy()
        depth_vis = normalize_depth_image(cv_input)
        cv2.namedWindow('Depth Camera Output', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Depth Camera Output', 106 * 4, 60 * 4)
        cv2.imshow('Depth Camera Output', depth_vis)
        cv2.waitKey(1)
        
        inputs = cam_data.unsqueeze(1).repeat(1,3,1,1) # refer to train_depth_resnet.py
        pred_rays = model(inputs)

        obs = torch.cat([obs[:, :-(pred_rays.shape[-1])], pred_rays], dim=-1)

        # modify obs

        ## actions and update env ##
        actions = policy(obs.detach())
        obs, _, rews, dones, infos = env.step(actions.detach())
        

        if RECORD_FRAMES:
            if i % 2:
                filename = os.path.join(LEGGED_GYM_ROOT_DIR, 'logs', train_cfg.runner.experiment_name, 'exported', 'frames', f"{img_idx}.png")
                env.gym.write_viewer_image_to_file(env.viewer, filename)
                img_idx += 1 
        if MOVE_CAMERA:
            camera_position += camera_vel * env.dt
            env.set_camera(camera_position, camera_position + camera_direction)


if __name__ == '__main__':
    args = get_args()
    play(args)
