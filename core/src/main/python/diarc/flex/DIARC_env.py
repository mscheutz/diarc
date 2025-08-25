import sys
import logging

from diarc.pytrade.wrapper import TRADEWrapper
from flex.policy_manager import PolicyManager
from javax.vecmath import Point3d, Matrix4d, Vector3d, Quat4d

logging.basicConfig(stream=sys.stdout, level=logging.INFO)
import gymnasium as gym
import numpy as np

class DIARCEnv(gym.Env):
        """
        Gym environment for prismatic object manipulation in 2D.

        The goal is to move a box from an initial position to a goal position
        by applying force at the object's centroid. The object moves like a prismatic
        joint, i.e. along a single axis that can change over time.

        State:
          [joint_axis (2D), displacement (2D)]

          - joint_axis: a 2D unit vector indicating the current force application axis.
          - displacement: difference between current box centroid and initial position.

        Action:
          [force_direction (2D), force_scale (scalar ∈ [0,1])]

          The applied force is computed as:
              applied_force = normalize(force_direction) * force_scale * max_force
        """
    def __init__(self, trade: TRADEWrapper):
        self.trade = trade
        self.joint_type = 'prismatic'
        self.joint_params = None
        self.max_steps = 100
        self.action_scale = 0.05
        self.success_distance = 0.05
        self.success_angle = 10
        try:
            self.policy = PolicyManager.load_policy(self.joint_type)
        except FileNotFoundError as e:
            logging.error(f"Policy loading failed: {e}")
            self.policy = None

        # Define observation space: joint_axis (2) and displacement (2)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        # Define action space: force_direction (2) and force_scale (1)
        self.action_space = gym.spaces.Box(low=np.array([-1, -1, 0]), high=np.array([1, 1, 1]), dtype=np.float32)

    def reset(self, seed=None, options=None):
        # todo: Make start symbol
        # ret = self.wrapper.call_trade("goToPose", "start") # todo: make safe, also use ret to check success
        return self._get_obs(), {}

    def step(self, action):
        """Execute FLEX policy-based door opening."""

        # Get initial hand position
        init_tf = self.trade.call_trade("getTransformTo", [], GROUND_PLANE_FRAME_NAME, HAND_FRAME_NAME)
        trans = Vector3d()
        init_tf.get(trans)
        initial_hand_pos = np.array([trans.x, trans.y, trans.z])

        print(f"Starting policy execution with {self.joint_type} joint")
        print(f"Max steps: {self.max_steps}, Action scale: {self.action_scale}")

        # Policy execution loop (existing logic)
        manipulation_success = False

        for step in range(self.max_steps):
            # Get current hand position
            curr_tf = self.trade.call_trade("getTransformTo", [], GROUND_PLANE_FRAME_NAME, HAND_FRAME_NAME)
            trans = Vector3d()
            curr_tf.get(trans)
            current_hand_pos = np.array([trans.x, trans.y, trans.z])

            # Get state and action
            state_vector = _construct_state_vector(current_hand_pos, initial_hand_pos)
            action = policy.select_action(state_vector)

            if len(action.shape) > 1:
                action = action.flatten()
            action[0] = -action[0]  # Invert x action direction for Spot
            target_position = None

            sliding_axis = self.joint_params["axis"]
            sliding_axis = sliding_axis / np.linalg.norm(sliding_axis)  # Normalize

            # Project the 3D action onto the 1D sliding direction
            action_magnitude = np.dot(action, sliding_axis)
            constrained_action = action_magnitude * sliding_axis

            print(f"Original action: {action}")
            print(f"Sliding axis: {sliding_axis}")
            print(f"Constrained action: {constrained_action}")

            # target position in "body" frame
            target_position = constrained_action * self.action_scale

            print(f"Step {step + 1}/{self.max_steps}: Target={target_position}")

            # Execute movement
            try:
                target_pose = SE3Pose(
                    x=target_position[0],
                    y=target_position[1],
                    z=target_position[2],
                    rot=current_hand_pose.rot
                )
                trans = Vector3d(target_position[0], target_position[1], target_position[2])
                orient = Quat4d()

                tmp = Matrix4d()
                tmp.setIdentity()
                tmp.setRotation(Quat4d(tf.rot.x.tf.rot.y, tf.rot.z, tf.rot.w))
                tmp.setTranslation(Vector3d(tf.x, tf.y, tf.z))

                # make trade call(s) to execute action
                ret = self.trade.call_trade("moveTo", [], "arm" action)  # todo: make safe, also use ret to check success

                # Check success (existing logic)
                if check_manipulation_success(interactive_perception, initial_hand_pos, current_hand_pos,
                                              {'distance': self.success_distance, 'angle': self.success_angle}):
                    print(f"Door opening completed successfully in {step + 1} steps!")
                    manipulation_success = True
                    break

            except Exception as e:
                print(f"Error in step {step + 1}: {e}")
                break

        print(f"Policy execution completed. Success: {manipulation_success}")
        time.sleep(2)  # Hold position to observe

        return True

        # make trade call(s) to execute action
        ret = self.trade.call_trade("moveToJointPositions", [], action) # todo: make safe, also use ret to check success
        done = False
        return self._get_obs(), None, done, False, {}

    def _get_obs(self):
        # make trade call(s) to make observation
        ret = self.wrapper.call_trade("moveToJointPositions", [], action) # todo: make safe, also use ret to check success

        # convert returned observations into target observation data type
        return np.random.uniform(-np.pi, np.pi, size=(7,))

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

# Test the environment
if __name__ == "__main__":
    env = DIARCEnv(wrapper = TRADEWrapper())

    obs, _ = env.reset()

    for _ in range(10):
        action = env.action_space.sample()  # Random action
        print(action)
        obs, reward, done, _, _ = env.step(action)
        if done:
            break

    env.close()
