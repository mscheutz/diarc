# Very initial impleemntation of PrismaticEnv for DIARC to be compatible with the existing untested flexExecutor class in the Github

# diarc/flex/env/PrismaticEnv.py
import sys
import logging
import numpy as np
import gymnasium as gym
from javax.vecmath import Point3d, Vector3d, Quat4d
from diarc.pytrade.wrapper import TRADEWrapper

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


class PrismaticEnv(gym.Env):
    """
    DIARC Gymnasium environment for prismatic object manipulation.

    State:
        [joint_axis_x, joint_axis_y, joint_axis_z,
         displacement_x, displacement_y, displacement_z]

    Action:
        [dx, dy, dz] - 3D movement delta (already processed by FlexExecutor)
    """
    
    def __init__(self, trade: TRADEWrapper):
        self.trade = trade
        self.joint_type = 'prismatic'
        self.joint_params = None  # Set by FlexWrapper after analysis
        self.action_scale = 0.02
        
        # Transform tracking
        self.init_tf = None
        self.tf_src_frame = 'odom'
        self.tf_dst_frame = 'hand'
        
        # Gym spaces
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=np.array([-1, -1, -1]), 
            high=np.array([1, 1, 1]), 
            dtype=np.float32
        )
    
    def _get_current_position(self) -> np.ndarray:
        """Get current gripper position. Required by FlexExecutor."""
        curr_tf = self.trade.call_trade("getTransform", [],
                                       self.tf_src_frame, self.tf_dst_frame)
        trans = Vector3d()
        curr_tf.get(trans)
        return np.array([trans.x, trans.y, trans.z])
    
    def _construct_state_vector(self, current_position: np.ndarray, 
                               initial_position: np.ndarray) -> np.ndarray:
        """Construct state vector for policy input."""
        if self.joint_params is None:
            raise ValueError("joint_params must be set before reset/step")
        
        displacement = current_position - initial_position
        joint_axis = self.joint_params["axis"]
        joint_axis = joint_axis / np.linalg.norm(joint_axis)
        state = np.concatenate([joint_axis, displacement])
        
        return state.astype(np.float32)
    
    def reset(self, seed=None, options=None):
        """Reset environment and capture initial state."""
        super().reset(seed=seed)
        
        # Capture initial transform
        self.init_tf = self.trade.call_trade("getTransform", [],
                                            self.tf_src_frame, self.tf_dst_frame)
        
        # Get initial position
        trans = Vector3d()
        self.init_tf.get(trans)
        initial_position = np.array([trans.x, trans.y, trans.z])
        
        # Construct initial observation (displacement is zero)
        obs = self._construct_state_vector(initial_position, initial_position)
        
        # Provide initial position in info for FlexExecutor
        info = {"initial_position": initial_position}
        
        return obs, info
    
    def step(self, action: np.ndarray):
        """Execute action via TRADE."""
        if len(action.shape) > 1:
            action = action.flatten()
        
        # Scale action
        target_position = action * self.action_scale
        
        # Execute movement via TRADE
        try:
            # TODO: is target_position in "hand" frmae already?
            trans = Point3d(target_position[0], target_position[1], target_position[2])
            orient = Quat4d(0, 0, 0, 1)
            
            ret = self.trade.call_trade("moveToRelative", [], "arm", trans, orient)
            
            # Get new observation
            initial_trans = Vector3d()
            self.init_tf.get(initial_trans)
            initial_position = np.array([initial_trans.x, initial_trans.y, initial_trans.z])
            
            current_position = self._get_current_position()
            obs = self._construct_state_vector(current_position, initial_position)
            
            info = {"trade_return": ret}
            
            return obs, 0.0, False, False, info
            
        except Exception as e:
            logging.error(f"Error executing step: {e}")
            try:
                initial_trans = Vector3d()
                self.init_tf.get(initial_trans)
                initial_position = np.array([initial_trans.x, initial_trans.y, initial_trans.z])
                current_position = self._get_current_position()
                obs = self._construct_state_vector(current_position, initial_position)
            except:
                obs = np.zeros(6, dtype=np.float32)
            
            return obs, 0.0, False, True, {"error": str(e)}

# Test the environment
if __name__ == "__main__":
    from diarc.pytrade.wrapper import TRADEWrapper
    
    print("Testing PrismaticEnv...")
    
    # Initialize environment
    env = PrismaticEnv(trade=TRADEWrapper())
    
    # We shhould set joint_params before using
    # (In real usage, we will use FlexWrapper sets this after analyze_joint)
    env.joint_params = {
        "axis": np.array([1.0, 0.0, 0.0]),  # Sliding along x-axis
        "error": 0.01
    }
    
    print("\n1. Resetting environment...")
    obs, info = env.reset()
    print(f"   Initial observation shape: {obs.shape}")
    print(f"   Initial observation: {obs}")
    print(f"   Initial position from info: {info['initial_position']}")
    
    print("\n2. Executing 5 random actions...")
    for i in range(5):
        # Generate random action
        action = env.action_space.sample()
        print(f"\n   Step {i+1}:")
        print(f"   - Action: {action}")
        
        # Execute action
        obs, reward, terminated, truncated, info = env.step(action)
        
        print(f"   - New observation: {obs}")
        print(f"   - Displacement: {obs[3:]}")
        print(f"   - Terminated: {terminated}, Truncated: {truncated}")
        
        if terminated or truncated:
            print("   - Episode ended!")
            break
    
    print("\n3. Test complete!")