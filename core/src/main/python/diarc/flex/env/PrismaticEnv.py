import sys
import logging

from diarc.pytrade.wrapper import TRADEWrapper
from javax.vecmath import Point3d, Matrix4d, Vector3d, Quat4d

logging.basicConfig(stream=sys.stdout, level=logging.INFO)
import gymnasium as gym
import numpy as np

class PrismaticEnv(gym.Env):
    """
    DIARC Gymnasium environment for prismatic object manipulation.

    State:
        [joint_axis_x, joint_axis_y, joint_axis_z,
        displacement_x, displacement_y, displacement_z]
        - joint_axis: a 3D unit vector indicating the current force application axis.
        - displacement: difference between current end-effector position and initial position.

    Action:
      [force_direction (3D), force_scale (scalar ∈ [0,1])]

      The applied force is computed as:
          applied_force = normalize(force_direction) * force_scale * max_force
    """
    def __init__(self, trade: TRADEWrapper):
        self.trade = trade
        self.joint_type = 'prismatic'
        self.joint_params = None
        self.action_scale = 0.05
        self.success_distance = 0.05
        self.init_tf = None
        self.curr_tf = None
        self.tf_src_frame = 'gpe'
        self.tf_dst_frame = 'hand'

        # TODO: are the action and observation spaces correct?
        # Define observation space: joint_axis (3) and displacement (3)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)
        # Define action space: force_direction (3) and force_scale (1)
        self.action_space = gym.spaces.Box(low=np.array([-1, -1, -1, 0]), high=np.array([1, 1, 1, 1]), dtype=np.float32)

    def _set_init_state(self):
        # get the initial hand position (first step only)
        if self.init_tf is None:
            self.init_tf = self.trade.call_trade("getTransformTo", [], self.tf_src_frame, self.tf_dst_frame)

    def _get_obs(self):
        # Get the initial hand position
        trans = Vector3d()
        self.init_tf.get(trans)
        initial_hand_pos = np.array([trans.x, trans.y, trans.z])

        # Get the current hand position
        curr_tf = self.trade.call_trade("getTransformTo", [], self.tf_src_frame, self.tf_dst_frame)
        trans = Vector3d()
        curr_tf.get(trans)
        current_hand_pos = np.array([trans.x, trans.y, trans.z])

        state_vector = self._construct_state_vector(current_hand_pos, initial_hand_pos)
        return state_vector

        # convert returned observations into target observation data type
        # return np.random.uniform(-np.pi, np.pi, size=(6,))

    def _check_manipulation_success(self, state_vector):
        """Check if manipulation succeeded (existing logic)."""
        distance = np.linalg.norm(state_vector[3:]) # displacement distance
        success = distance >= self.success_distance
        print(f"Prismatic distance: {distance:.3f}m (threshold: {self.success_distance:.3f}m)")
        return success

    def _construct_state_vector(self, current_position, initial_position):
        """
        Construct state vector for policy input.

        Args:
            current_position: Current gripper position [x, y, z]
            initial_position: Initial grasp position [x, y, z]

        Returns:
            np.array: State vector [joint_axis_x, joint_axis_y, joint_axis_z,
                                   displacement_x, displacement_y, displacement_z]
        """
        if self.joint_params is None:
            raise ValueError("Must analyze trajectory first!")

        # Calculate displacement from initial position
        displacement = current_position - initial_position

        # Get normalized joint axis
        joint_axis = self.joint_params["axis"]
        joint_axis = joint_axis / np.linalg.norm(joint_axis)

        # Construct state: [joint_axis (3D), displacement (3D)]
        state = np.concatenate([joint_axis, displacement])

        return state.astype(np.float32)

    def reset(self, seed=None, options=None):
        # reset the initial hand position
        self.init_tf = None

        return self._get_obs(), {}

    def step(self, action):
        # set initial hand position
        self._set_init_state()

        if len(action.shape) > 1:
            action = action.flatten()

        # TODO: why is this needed?
        action[0] = -action[0]  # Invert x action direction for Spot

        sliding_axis = self.joint_params["axis"]
        sliding_axis = sliding_axis / np.linalg.norm(sliding_axis)  # Normalize

        # Project the 3D action onto the 1D sliding direction
        action_magnitude = np.dot(action, sliding_axis)
        constrained_action = action_magnitude * sliding_axis

        # target position in "body" frame
        target_position = constrained_action * self.action_scale

        # Execute movement
        try:
            trans = Vector3d(target_position[0], target_position[1], target_position[2])
            orient = Quat4d()
            self.init_tf.get(orient)

            target_pose = Matrix4d()
            target_pose.setIdentity()
            target_pose.setRotation(orient)
            target_pose.setTranslation(trans)

            # make trade call(s) to execute action
            ret = self.trade.call_trade("moveTo", [], "arm", target_pose)

            # Get new observation
            state_vector = self._get_obs()

            # Check success
            terminated = self._check_manipulation_success(state_vector)

            return state_vector, None, terminated, False, {}

        except Exception as e:
            logging.error(f"Error executing step. {e}")

# Test the environment
if __name__ == "__main__":
    env = PrismaticEnv(trade = TRADEWrapper())

    obs, _ = env.reset()

    for _ in range(10):
        action = env.action_space.sample()  # Random action
        print(action)
        obs, reward, done, _, _ = env.step(action)
        if done:
            break

    env.close()
