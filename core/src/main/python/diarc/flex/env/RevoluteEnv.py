# diarc/flex/env/RevoluteEnv.py
import sys
import logging
import numpy as np
import gymnasium as gym
from javax.vecmath import Point3d, Vector3d, Quat4d
from diarc.pytrade.wrapper import TRADEWrapper

logging.basicConfig(stream=sys.stdout, level=logging.INFO)


class RevoluteEnv(gym.Env):
    """
    DIARC Gymnasium environment for revolute joint manipulation.

    State: [joint_axis(3), displacement(3)]
    Action: [dx, dy, dz] - 3D movement delta in odom frame
    """
    
    def __init__(self, trade: TRADEWrapper):
        self.trade = trade
        self.joint_type = 'revolute'
        self.joint_params = None
        self.action_scale = 0.02
        
        self.init_tf = None
        self.tf_src_frame = 'odom'
        self.tf_dst_frame = 'hand'
        
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=np.array([-1, -1, -1]), 
            high=np.array([1, 1, 1]), 
            dtype=np.float32
        )
    
    def _get_current_position(self) -> np.ndarray:
        """Get current gripper position in odom frame."""
        curr_tf = self.trade.call_trade("getTransform", [],
                                       self.tf_src_frame, self.tf_dst_frame)
        trans = Vector3d()
        curr_tf.get(trans)
        return np.array([trans.x, trans.y, trans.z])
    
    def _construct_state_vector(self, current_position: np.ndarray, 
                            initial_position: np.ndarray) -> np.ndarray:
        """Construct state vector for revolute policy input."""
        if self.joint_params is None:
            raise ValueError("joint_params must be set before reset/step")
        
        # Get joint parameters
        joint_axis = self.joint_params["axis"]
        joint_axis = joint_axis / np.linalg.norm(joint_axis)
        joint_center = self.joint_params["center"]  # pr
        
        # For revolute: vt = (pt - pr) - [(pt - pr)·hr]hr
        vector_to_point = current_position - joint_center  # pt - pr
        
        # Project onto rotation axis
        projection_on_axis = np.dot(vector_to_point, joint_axis) * joint_axis
        
        # vt is the component perpendicular to the axis
        vt = vector_to_point - projection_on_axis
        
        # State: [hr, vt]
        state = np.concatenate([joint_axis, vt])
        
        return state.astype(np.float32)
    
    def reset(self, seed=None, options=None):
        """Reset environment and capture initial state."""
        super().reset(seed=seed)
        
        self.init_tf = self.trade.call_trade("getTransform", [],
                                            self.tf_src_frame, self.tf_dst_frame)
        
        trans = Vector3d()
        self.init_tf.get(trans)
        initial_position = np.array([trans.x, trans.y, trans.z])
        
        obs = self._construct_state_vector(initial_position, initial_position)
        info = {"initial_position": initial_position}
        
        return obs, info
    
    def step(self, action: np.ndarray):
        """Execute action via TRADE using absolute positioning in odom frame."""
        if len(action.shape) > 1:
            action = action.flatten()
        
        # Action is delta in odom frame
        delta_odom = action * self.action_scale
        
        try:
            # Get current position in odom frame
            current_pos = self._get_current_position()
            
            # Calculate target position in odom frame
            target_pos = current_pos + delta_odom
            
            logging.info(f"Current position: {current_pos}")
            logging.info(f"Delta: {delta_odom}")
            logging.info(f"Target position: {target_pos}")
            
            # Get current orientation to maintain it
            odom_tf_hand = self.trade.call_trade("getTransform", [],
                                           self.tf_src_frame, self.tf_dst_frame)
            orient = Quat4d()
            odom_tf_hand.get(orient)
            
            # Use moveTo for absolute positioning in odom frame
            target = Point3d(target_pos[0], target_pos[1], target_pos[2])


            #################### transform to "body" frame
            body_tf_odom = self.trade.call_trade("getTransform", [],"body", "odom")
            # position from odom frame to body frame
            body_tf_odom.transform(target)
            # orientation from odom frame to body frame
            body_orient_hand = Quat4d()
            body_tf_hand = self.trade.call_trade("getTransform", [],"body", "hand")
            body_tf_hand.get(body_orient_hand)
            # body_tf_odom.get(orient)
            # orient.mul(odom_orient)
            ################################

            ret = self.trade.call_trade("moveTo", [], "arm", target, body_orient_hand)
            
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
    env = RevoluteEnv(trade=TRADEWrapper())
    
    # Set joint params for revolute joint
    env.joint_params = {
        "center": np.array([0.5, 0.0, 0.5]),  # Rotation center
        "radius": 0.3,
        "axis": np.array([0.0, 0.0, 1.0]),    # Vertical rotation axis
        "error": 0.01
    }
    
    print("\n1. Resetting environment...")
    obs, info = env.reset()
    print(f"   Initial observation: {obs}")
    print(f"   Initial position: {info['initial_position']}")
    
    print("\n2. Executing 3 test actions...")
    for i in range(3):
        action = env.action_space.sample()
        print(f"\n   Step {i+1}:")
        print(f"   - Action: {action}")
        
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"   - New displacement: {obs[3:]}")
        
        if terminated or truncated:
            break
    
    print("\n3. Test complete!")