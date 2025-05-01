import time
import sys
import logging
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
import gymnasium as gym
from gymnasium import spaces
import numpy as np
from pytrade.wrapper import TRADEWrapper
from ai.thinkingrobots.trade import TRADE


class DIARCEnv(gym.Env):
    def __init__(self, wrapper):
        # TODO: Initialize TRADE
        self.wrapper = wrapper
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.pi, high=np.pi, shape=(7,), dtype=np.float32) # Todo: Change to include obj pos
        self.state = np.zeros(7, dtype=np.float32)
        time.sleep(1)

    def reset(self, seed=None, options=None):
        # todo: Make start symbol
        # ret = self.wrapper.call_trade("goToPose", "start") # todo: make safe, also use ret to check success
        return self._get_obs(), {}

    def step(self, action):
        # TRADE call goto(action)
        # Set done
        ret = self.wrapper.call_trade("moveToJointPositions", [], action) # todo: make safe, also use ret to check success
        done = False
        return self._get_obs(), None, done, False, {}

    def close(self):
        """Clean up resources."""
        pass

    def _get_obs(self):
        return np.random.uniform(-np.pi, np.pi, size=(7,))


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
