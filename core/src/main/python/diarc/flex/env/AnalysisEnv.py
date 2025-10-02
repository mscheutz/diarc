# Very initial impleemntation of AnalysisEnv for DIARC to be compatible with the existing untested flexExecutor class in the Github
import sys
import logging
import numpy as np
import gymnasium as gym
from javax.vecmath import Point3d, Vector3d, Quat4d
from diarc.pytrade.wrapper import TRADEWrapper

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


class AnalysisEnv(gym.Env):
    """
    DIARC Gymnasium environment for joint analysis via wiggle movements.
    
    This environment executes exploratory movements to collect trajectory data
    for determining joint type and parameters.
    
    Action:
        [x, y, z] - Absolute target position in vision frame
    
    Observation:
        [x, y, z] - Current gripper position
    """
    
    def __init__(self, trade: TRADEWrapper):
        self.trade = trade
        
        # Transform parameters
        self.tf_src_frame = 'odom'
        self.tf_dst_frame = 'hand'
        
        # Gym spaces
        # Action: absolute 3D position
        self.action_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32
        )
        
        # Observation: current 3D position
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32
        )
    
    def _get_current_position(self) -> np.ndarray:
        """Get current gripper position from TRADE."""
        try:
            tf = self.trade.call_trade("getTransform", [],
                                      self.tf_src_frame, self.tf_dst_frame)
            trans = Vector3d()
            tf.get(trans)
            return np.array([trans.x, trans.y, trans.z], dtype=np.float32)
        except Exception as e:
            logging.error(f"Error getting current position: {e}")
            raise
    
    def reset(self, seed=None, options=None):
        """
        Reset and return current gripper position.
        
        Returns:
            obs: Current position [x, y, z]
            info: Dict with initial position
        """
        super().reset(seed=seed)
        
        # Get current position as starting point
        current_position = self._get_current_position()
        
        logging.info(f"AnalysisEnv reset at position: {current_position}")
        
        # Return current position as observation
        info = {"initial_position": current_position}
        return current_position, info
    
    def step(self, action: np.ndarray):
        """
        Move gripper to target absolute position.
        
        Args:
            action: Target position [x, y, z] in vision frame (absolute)
        
        Returns:
            obs: Achieved position [x, y, z]
            reward: 0 (not used)
            terminated: False
            truncated: True if error occurred
            info: Dict with movement info
        """
        if len(action.shape) > 1:
            action = action.flatten()
        
        target_position = action
        
        logging.info(f"AnalysisEnv moving to: {target_position}")
        
        try:
            # Get current transform for orientation
            curr_tf = self.trade.call_trade("getTransform", [],
                                           self.tf_src_frame, self.tf_dst_frame)

            # maintain current orientation during wiggle
            orient = Quat4d(0, 0, 0, 1)
            
            # Create target pose (absolute position, current orientation)
            target = Point3d(target_position[0], target_position[1], target_position[2])

            # Execute movement
            # Calculate relative movement from current position
            current_pos = self._get_current_position()
            delta = target_position - current_pos
            ret = self.trade.call_trade("moveToRelative", [], "arm",
                                       Point3d(delta[0], delta[1], delta[2]), orient)
            
            # Get achieved position
            achieved_position = self._get_current_position()
            
            logging.info(f"AnalysisEnv achieved: {achieved_position}")
            
            info = {
                "trade_return": ret,
                "target": target_position,
                "achieved": achieved_position,
                "error": np.linalg.norm(achieved_position - target_position)
            }
            
            return achieved_position, 0.0, False, False, info
            
        except Exception as e:
            logging.error(f"Error in AnalysisEnv step: {e}")
            
            # Try to return current position on error
            try:
                obs = self._get_current_position()
            except:
                obs = np.zeros(3, dtype=np.float32)
            
            return obs, 0.0, False, True, {"error": str(e)}


# Test the environment
if __name__ == "__main__":
    env = AnalysisEnv(trade=TRADEWrapper())
    
    obs, info = env.reset()
    print(f"Initial position: {obs}")
    
    # Simulate wiggle movements
    wiggle_offsets = [
        np.array([0.05, 0.0, 0.0]),   # Forward
        np.array([-0.05, 0.0, 0.0]),  # Back
        np.array([0.0, 0.05, 0.0]),   # Left
        np.array([0.0, -0.05, 0.0]),  # Right
    ]
    
    start_pos = obs.copy()
    
    for i, offset in enumerate(wiggle_offsets):
        target = start_pos + offset
        print(f"\nWiggle {i+1}: Moving to {target}")
        
        obs, reward, done, truncated, info = env.step(target)
        print(f"Achieved: {obs}")
        print(f"Error: {info.get('error', 'N/A')}")
        
        if truncated:
            print("Movement failed!")
            break
        
        # Return to start
        obs, _, _, _, _ = env.step(start_pos)
        print(f"Returned to: {obs}")