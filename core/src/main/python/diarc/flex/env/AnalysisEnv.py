# diarc/flex/env/AnalysisEnv.py
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
    
    Action: [x, y, z] - Absolute target position in odom frame
    Observation: [x, y, z] - Current gripper position
    """
    
    def __init__(self, trade: TRADEWrapper):
        self.trade = trade
        self.tf_src_frame = 'odom'
        self.tf_dst_frame = 'hand'
        
        self.action_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32
        )
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
        """Reset and return current gripper position."""
        super().reset(seed=seed)
        current_position = self._get_current_position()
        logging.info(f"AnalysisEnv reset at position: {current_position}")
        info = {"initial_position": current_position}
        return current_position, info
    
    def step(self, action: np.ndarray):
        """Move gripper to target absolute position in odom frame."""
        if len(action.shape) > 1:
            action = action.flatten()
        
        target_position = action  # Absolute position in odom frame
        logging.info(f"AnalysisEnv moving to: {target_position}")
        
        try:
            # Get current orientation to maintain it
            curr_tf = self.trade.call_trade("getTransform", [],
                                           self.tf_src_frame, self.tf_dst_frame)
            orient = Quat4d()
            curr_tf.get(orient)
            
            # Create target point
            target = Point3d(target_position[0], target_position[1], target_position[2])
            
            # Use moveTo for absolute positioning in odom frame
            ret = self.trade.call_trade("moveTo", [], "arm", target, orient)
            
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
    
    wiggle_offsets = [
        np.array([0.05, 0.0, 0.0]),
        np.array([-0.05, 0.0, 0.0]),
        np.array([0.0, 0.05, 0.0]),
        np.array([0.0, -0.05, 0.0]),
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
        
        obs, _, _, _, _ = env.step(start_pos)
        print(f"Returned to: {obs}")