import gymnasium
import environment
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env


def create_env():
    return environment.GymEnv()


drone_env = environment.GymEnv()
env = make_vec_env(create_env, n_envs=4)
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=250)

model.save("ppo_custom_env")
del model
model = PPO.load("ppo_custom_env")

# Evaluate the model
obs = env.reset()
while True:
    (action, _states) = model.predict(obs)
    _ = env.step(action)
    # this will need to be implemented
    env.render()
