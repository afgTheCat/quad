import environment
import numpy as np

g = environment.GymEnv()
(state, reward, finished, trun, _) = g.step(np.array([0, 0, 0, 0]))
print(state, reward, finished, trun)

(state, _) = g.reset()
print(state)
