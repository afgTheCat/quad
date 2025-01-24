import environment
import numpy as np

env = environment.GymEnv()
(state, reward, finished, trun, _) = env.step(np.array([0, 0, 0, 0]))
print(state, reward, finished, trun)

(state, _) = env.reset()
print(state)
