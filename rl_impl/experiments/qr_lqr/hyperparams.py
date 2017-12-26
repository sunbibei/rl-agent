""" Hyperparameters for PR2 trajectory optimization experiment. """
from __future__ import division

from datetime import datetime
import os.path

import numpy as np

from gps import __file__ as gps_filepath
from gps.agent.ros.agent_ros import AgentROS
from gps.algorithm.algorithm_traj_opt import AlgorithmTrajOpt
from gps.algorithm.cost.cost_fk import CostFK
from gps.algorithm.cost.cost_action import CostAction
from gps.algorithm.cost.cost_state import CostState
from gps.algorithm.cost.cost_sum import CostSum
from gps.algorithm.cost.cost_utils import RAMP_LINEAR, RAMP_FINAL_ONLY
from gps.algorithm.dynamics.dynamics_lr_prior import DynamicsLRPrior
from gps.algorithm.dynamics.dynamics_prior_gmm import DynamicsPriorGMM
from gps.algorithm.traj_opt.traj_opt_lqr_python import TrajOptLQRPython
from gps.algorithm.policy.lin_gauss_init import init_lqr
from gps.gui.target_setup_gui import load_pose_from_npz
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
from gps.utility.general_utils import get_ee_points
from gps.gui.config import generate_experiment_info


EE_POINTS = np.array([[0.02, -0.025, 0.05], [0.02, -0.025, -0.05],
                      [0.02, 0.05, 0.0]])

SENSOR_DIMS = {
    JOINT_ANGLES: 3,
    JOINT_VELOCITIES: 3,
    END_EFFECTOR_POINTS: 3 * EE_POINTS.shape[0],
    END_EFFECTOR_POINT_VELOCITIES: 3 * EE_POINTS.shape[0],
    ACTION: 3,
}

PR2_GAINS = np.array([0.001, 0.00108, 0.00393])

BASE_DIR = '/'.join(str.split(gps_filepath, '/')[:-2])
EXP_DIR = BASE_DIR + '/../experiments/qr_lqr/'

x0s = []
ee_tgts = []
reset_conditions = []

common = {
    'experiment_name': 'qr_lqr' + '_' + \
            datetime.strftime(datetime.now(), '%m-%d-%y_%H-%M'),
    'experiment_dir': EXP_DIR,
    'data_files_dir': EXP_DIR + 'data_files/',
    'target_filename': EXP_DIR + 'target.npz',
    'log_filename': EXP_DIR + 'log.txt',
    'conditions': 1,
}

ja_x0s = np.array([[0.143058, -0.462141, 1.10278],\
                   [0.103285,-1.06029,-1.1817]])

ee_pos_x0s = np.array([[0.581008,0.185772,0.234795],\
                       [0.581008,0.185772,0.234795]])

ee_rot_x0s = np.array([[[0.83372763, -0.54771745, 0.07002733],\
    [-0.54459356, -0.83658433, -0.05953581],\
    [0.09119257, 0.01150022, -0.99576687]],\
    [[0.83372763, -0.54771745, 0.07002733],\
    [-0.54459356, -0.83658433, -0.05953581],\
    [0.09119257, 0.01150022, -0.99576687]]])

ja_auxs = np.array([[0., 0., 0., 0., 0., 0., 0.],\
    [0., 0., 0., 0., 0., 0., 0.]])

ee_pos_tgts = np.array([[0.7,0.20,-0.129],\
                       [0.7,0.20,-0.129]])

ee_rot_tgts = np.array([[[-0.9987, -0.0500, 0.0094],\
    [-0.0498, 0.9986, 0.0150],\
    [-0.0101, 0.0145, -0.9998]],\
    [[-0.9987, -0.0500, 0.0094],\
    [-0.0498, 0.9986, 0.0150],\
    [-0.0101, 0.0145, -0.9998]]])

tgt_state = np.loadtxt(EXP_DIR + 'target_state', delimiter=',')

# TODO(chelsea/zoe) : Move this code to a utility function
# Set up each condition.
for i in xrange(common['conditions']):
    x0 = np.zeros(24)
    x0[:3] = ja_x0s[0]
    x0[6:(6+3*EE_POINTS.shape[0])] = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_x0s[0], ee_rot_x0s[0]).T
    )

    ee_tgt = np.ndarray.flatten(
        get_ee_points(EE_POINTS, ee_pos_tgts[0], ee_rot_tgts[0]).T
    )

    aux_x0 = np.zeros(3)
    aux_x0[:] = ja_x0s[0]

    reset_condition = {
        TRIAL_ARM: {
            'mode': JOINT_SPACE,
            'data': x0[0:3],
        },
        AUXILIARY_ARM: {
            'mode': JOINT_SPACE,
            'data': aux_x0,
        },
    }

    x0s.append(x0)
    ee_tgts.append(ee_tgt)
    reset_conditions.append(reset_condition)


if not os.path.exists(common['data_files_dir']):
    os.makedirs(common['data_files_dir'])

agent = {
    'type': AgentROS,
    'trial_command_topic': '/RLAgentBase/trial_command_topic',
    'reset_command_topic': '/RLAgentBase/reset_command_topic',
    # 'relax_command_topic': 'gps_controller_relax_command',
    'data_request_topic':  '/RLAgentBase/report_requests_topic',
    'sample_result_topic': '/RLAgentBase/report_pub_topic',
    'dt': 0.5,
    'conditions': common['conditions'],
    'T': 21,
    'x0': x0s,
    'ee_points_tgt': ee_tgts,
    'reset_conditions': reset_conditions,
    'sensor_dims': SENSOR_DIMS,
    'state_include': [JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
    'end_effector_points': EE_POINTS,
    'obs_include': [],
}

algorithm = {
    'type': AlgorithmTrajOpt,
    'conditions': common['conditions'],
    'iterations': 30,
}

algorithm['init_traj_distr'] = {
    'type': init_lqr,
    'init_gains':  1.0 / PR2_GAINS,
    'init_acc': np.zeros(SENSOR_DIMS[ACTION]),
    'init_var': 1.0,
    'stiffness': 0.5,
    'stiffness_vel': 0.25,
    'final_weight': 50,
    'dt': agent['dt'],
    'T': agent['T'],
}

torque_cost = {
    'type': CostAction,
    'wu': 5e-3 / PR2_GAINS,
}

state_cost1 = {
    'type': CostState,
    # Target end effector is subtracted out of EE_POINTS in ROS so goal
    # is 0.
    'data_types': {
        JOINT_ANGLES: {
            'target_state': tgt_state,  # Target state - must be set.
            'wp': 10*np.ones(SENSOR_DIMS[JOINT_ANGLES]),  # State weights - must be set.
        },
    },
}

state_cost2 = {
    'type': CostState,
    # Target end effector is subtracted out of EE_POINTS in ROS so goal
    # is 0.
    'data_types': {
        JOINT_ANGLES: {
            'target_state': tgt_state,  # Target state - must be set.
            'wp': 10*np.ones(SENSOR_DIMS[JOINT_ANGLES]),  # State weights - must be set.
        },
    },
    'wp_final_multiplier': 10.0,  # Weight multiplier on final timestep.
    'ramp_option': RAMP_FINAL_ONLY,
}

algorithm['cost'] = {
    'type': CostSum,
    'costs': [state_cost1, state_cost2],
    'weights': [1.0, 1.0],
}

algorithm['dynamics'] = {
    'type': DynamicsLRPrior,
    'regularization': 1e-6,
    'prior': {
        'type': DynamicsPriorGMM,
        'max_clusters': 20,
        'min_samples_per_cluster': 40,
        'max_samples': 20,
    },
}

algorithm['traj_opt'] = {
    'type': TrajOptLQRPython,
}

algorithm['policy_opt'] = {}

config = {
    'iterations': algorithm['iterations'],
    'common': common,
    'verbose_trials': 0,
    'agent': agent,
    'gui_on': True,
    'algorithm': algorithm,
    'num_samples': 5,
}

common['info'] = generate_experiment_info(config)
