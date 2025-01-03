import random
import numpy as np
import plotly.express as px


def cumsum2(sum: list[int], lbow=-1000, hbow=1000) -> list[int]:
    acc = 0
    elems = [acc]
    for e in sum:
        acc = max(min(acc + e, hbow), lbow)
        elems.append(acc)
    return elems


def cumsum3(nsteps, pos_min=-1000, pos_max=1000):
    all_pos = [0.0]
    pos = 0
    vel = 0
    for _ in range(nsteps):
        vel += -0.1 if random.random() < 0.5 else 0.1
        pos += vel
        if pos < pos_min or pos > pos_max:
            pos = max(min(pos, pos_max), pos_min)
            vel = 0.1 if vel < 0 else -0.1
        all_pos.append(pos)
    return all_pos


def input_gen(nsteps, p=0.5, stepsize=1):
    steps = [
        1 * stepsize if random.random() < p else -1 * stepsize for i in range(nsteps)
    ]
    y = cumsum2(steps)
    x = list(range(len(y)))
    return x, list(y)


if __name__ == "__main__":
    nsteps = 10000
    simulation_data = {}
    x, y = input_gen(nsteps)
    y2 = cumsum3(nsteps)
    simulation_data["x"] = x
    simulation_data["y"] = y
    simulation_data["y2"] = y2

    ycols = ["y2"]
    fig = px.line(simulation_data, x="x", y=ycols)
    fig.show()
