from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class X30RoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.48] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_HipX_joint': 0.1,   # [rad]
            'HL_HipX_joint': 0.1,   # [rad]
            'FR_HipX_joint': -0.1 ,  # [rad]
            'HR_HipX_joint': -0.1,   # [rad]

            'FL_HipY_joint': -0.715,     # [rad]
            'HL_HipY_joint': -0.715,   # [rad]
            'FR_HipY_joint': -0.715,     # [rad]
            'HR_HipY_joint': -0.715,   # [rad]

            'FL_Knee_joint': 1.43,   # [rad]
            'HL_Knee_joint': 1.43,    # [rad]
            'FR_Knee_joint': 1.43,  # [rad]
            'HR_Knee_joint': 1.43,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'HipX': 120. , 'HipY': 120. ,'Knee': 150. }  # [N*m/rad]
        damping = {'HipX': 3. , 'HipY': 3. ,'Knee': 3.5 }     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/X30/urdf/X30.urdf'
        name = "X30"
        foot_name = "FOOT"
        penalize_contacts_on = ["THIGH", "SHANK"]
        collision_state = ["TORSO","THIGH", "SHANK"]
        terminate_after_contacts_on = ["TORSO"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False

  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.95
        base_height_target = 0.49
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.000001#-0.0002
            dof_pos_limits = -10.0

class X30RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_X30'
        max_iterations = 30000

   

  
