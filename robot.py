import mujoco
import numpy as np
from mujoco import viewer

class RobotEnv:
    def __init__(self, xml_file):
        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.data = mujoco.MjData(self.model)
        
        self.observation_space = self._get_obs().shape[0]
        self.action_space = self.model.nu

        self.viewer = None

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs()

    def step(self, action):
        self.data.ctrl[:] = action
        
        mujoco.mj_step(self.model, self.data)
        
        obs = self._get_obs()
        reward = self._get_reward()
        done = self._is_done()
        
        return obs, reward, done, {}

    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel])

    def _get_reward(self):
        return 0.0

    def _is_done(self):
        return False

    def render(self):
        if self.viewer is None:
            self.viewer = viewer.launch_passive(self.model, self.data)
        
        self.viewer.sync()

    def close(self):
        if self.viewer:
            self.viewer.close()


if (__name__ == "__main__"):
    env = RobotEnv("robotis_op3/scene.xml")
    obs = env.reset()
    for i in range(1000):
        env.render()
        action = np.random.uniform(-1, 1, env.action_space)
        obs, reward, done, _ = env.step(action)
        if done:
            obs = env.reset()
    env.close()
