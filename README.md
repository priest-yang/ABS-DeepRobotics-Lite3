# ABS for Deep Robotics Lite3 Robot

This Repo is based on the work [Agile-but-Safe](https://agile-but-safe.github.io/) 



## Contributions

- Adapt to the DeepRobotics Lite3 Model
- Added [`play_cv.py`](ABS/training/legged_gym/legged_gym/scripts/play_cv.py), using trained ResNet (depth cam to Ray2d)
- Add opencv viewer for the depth cam in [`play_cv.py`](ABS/training/legged_gym/legged_gym/scripts/play_cv.py)



## Setup the Env 

Refer to the [original repo](https://github.com/LeCAR-Lab/ABS)



## Step of Training

**run everything in `legged_gym/legged_gym`,**



1. train the agile and recovery policy, can use "--headless" to disable GUI

```
# agile policy
python scripts/train.py --task=Lite3_pos_rough --max_iterations=4000 --headless

# agile policy, lagrangian ver
python scripts/train.py --task=Lite3_pos_rough_ppo_lagrangian --max_iterations=4000

# recovery policy
python scripts/train.py --task=Lite3_rec_rough --max_iterations=1000
```



2. Play the trained policy

```cmd
python scripts/play.py --task=Lite3_pos_rough [--load_run=xxx --resume]
python scripts/play.py --task=Lite3_rec_rough [--load_run=xxx --resume]
```

**Note:** Have to run this to export the serialized ``.pth`` policy in the ``log/[your task]/exported/`` folder 



In case using depth cam (what we only have on Lite3), run the following

```
python scripts/camrec.py --task=Lite3_pos_rough --num_envs=3
```

Tips from original repo: 

+ Tips 1: You can edit the `shift` value in Line 93 and the `log_root` in Line 87 to collect different dataset files in parallel (so you can merge them by simply moving the files), and manually change the obstacles in `env_cfg.asset.object_files` in Line 63.
+ Tips 2: After collecting the data, there's a template code in [`train_depth_resnet.py`](training/legged_gym/legged_gym/scripts/train_depth_resnet.py) to train the ray-prediction network, but using what you like for training CV models is highly encouraged!
+ Tips 3: You may change camera configs of resolution, position, FOV, and depth range in the [config file](training/legged_gym/legged_gym/envs/go1/go1_pos_config.py) Line 151.



After the ResNet Model is trained and saved, you can run test with ResNet:

(modify the ResNet model path at Line 20)

```
python scripts/play_cv.py --task=Lite3_pos_rough
```



3. Use the testbed, and train/test Reach-Avoid network:

**Note:** MUST modify the path to recovery model in Line 222 in ``testbed.py``  Before run with ``--testRA``. 

```
# try testbed
python scripts/testbed.py --task=Lite3_pos_rough [--load_run=xxx --resume] --num_envs=1

# train RA (be patient it will take time to converge (more than 5 hours)) 
# make sure you have at least exported one policy by play.py so the exported folder exists
python scripts/testbed.py --task=Lite3_pos_rough --num_envs=1000 --headless --trainRA

# test RA (only when you have trained one RA)
python scripts/testbed.py --task=Lite3_pos_rough --num_envs=1 --testRA

# evaluate
python scripts/testbed.py --task=Lite3_pos_rough --num_envs=1000 --headless [--load_run=xxx] [--testRA]
```


## Notes During Development

- When migrate from Go1 to Lite3, the reward (**velo_dir**) have to be modified, otherwise the robot will move backward to target and then turn around. Make sure the robot can move forward, otherwise the training of RA network will fail (no info from the depth camera)
- Most optimizers was changed from ``torch.optim.SGD`` to ``torch.optim.AdamW`` . Especially for ResNet model in [`train_depth_resnet.py`](training/legged_gym/legged_gym/scripts/train_depth_resnet.py), a weight decay is crucial to reach a stable loss decrease in the testset. 
- Don't forget to add ``--resume`` when you want to load previous model with ``[--load_run=xxx]``


## Deployment on Lite3

### Hardware
- Nvidia Jetson Xavier NX 
- Realsense D435 depth cam
- 

### Deploy code

#### [`udp_sender.py`](Deploy/NX/udp_sender.py). Should run it firstly on the NX board. 

- Receive Data from depth camera using `pyrealsense2`.
- Resnet inference with CUDA (depth to embedding). (AT 30 fps with 106 * 60 each frame)
- Send embedding to control board using UDP socket. The reason is I fail to build ``libtorch`` with cuda acceleration on NX(aarch64), as a result the fps is less than 3. 
- run by 
    ```
    python3 udp_python.py send
    ```

#### `Lite3 SDK` provided by DEEP Robotics
- run by
```shell
cd Deploy/rl_controller
mkdir build && cd build
sh ../make.sh
```
- You can choose either build project on the control board **(slow)** OR set up cross-compilation by set `-DSEND_REMOTE=ON` in `make.sh`

Note: 
- config the 
- g++ for aarch64 `aarch64-linux-gnu-g++` should be installed if the robot is ARM but PC is X86.
- use `-j1` in `make.sh` if build on robot directly

    
    

