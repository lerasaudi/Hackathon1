import gym
import gym_panda
import numpy as np

env = gym.make('panda-v0')
observation = env.reset()
done = False
error = 0.01
fingers = 1


object_position = [0.7, 0, 0.1]
k_p = 10
k_d = 1

# Circular movement parameters
circle_radius = 0.5
angular_velocity = 0.2
total_time_circular = 2 * np.pi / angular_velocity
time_step = 1.0 / 240.0
dt = 1.0 / 240.0  # the default timestep in pybullet is 240 Hz
t = 0

def calculate_pd_control(dx, dy, dz, dt):
    pd_x = k_p * dx + k_d * dx / dt
    pd_y = k_p * dy + k_d * dy / dt
    pd_z = k_p * dz + k_d * dz / dt
    return pd_x, pd_y, pd_z

def stage_0():
    dx_ = object_position[0] - observation[0]
    dy_ = object_position[1] - observation[1]
    dz_ = object_position[2] - observation[2]

    if abs(dx_) < error and abs(dy_) < error and abs(dz_) < error:
        fingers = 0
        if (observation[3] + observation[4]) < error + 0.02 and fingers == 0:
            return True

    return False

def stage_1():
    target_z = 0.5
    target_y = 0
    target_x = 0.4
    dx_ = target_x - observation[0]
    dy_ = target_y - observation[1]
    dz_ = target_z - observation[2]

    return abs(dx_) < error and abs(dy_) < error and abs(dz_) < error

def stage_2():
    target_z = 0.05
    target_y = 0
    target_x = 0.4
    dx_ = target_x - observation[0]
    dy_ = target_y - observation[1]
    dz_ = target_z - observation[2]

    return abs(dx_) < error and abs(dy_) < error and abs(dz_) < error

def stage_3():
    fingers = 1
    if (observation[3] + observation[4]) > error + 0.02 and fingers == 1:
        target_z = 0.2
        target_y = 0
        target_x = 0.4
        dx_ = target_x - observation[0]
        dy_ = target_y - observation[1]
        dz_ = target_z - observation[2]

        dx_ = target_x - observation[0]
        dy_ = target_y - observation[1]
        dz_ = target_z - observation[2]

        return abs(dx_) < error and abs(dy_) < error and abs(dz_) < error

    return False

def stage_4():
    target_x = circle_radius * np.cos(angular_velocity * t * dt)
    target_y = circle_radius * np.sin(angular_velocity * t * dt)
    target_z = 0.2

    dx_ = target_x - observation[0]
    dy_ = target_y - observation[1]
    dz_ = target_z - observation[2]

    return abs(dx_) < error and abs(dy_) < error and abs(dz_) < error


stages = [stage_0, stage_1, stage_2, stage_3, stage_4]
stage_index = 0

for i_episode in range(20):
    observation = env.reset()
    fingers = 1

    dx_, dy_, dz_ = 0, 0, 0

    while True:
        env.render()

        if stages[stage_index]():
            stage_index += 1

        if stage_index == len(stages):
            stage_index = 0

        pd_x, pd_y, pd_z = calculate_pd_control(dx_, dy_, dz_, dt)
        action = [pd_x, pd_y, pd_z, fingers]
        observation, reward, done, info = env.step(action)
        object_position = info['object_position']

        t += 1  

        if done:
            print("Episode finished after {} timesteps".format(t))
            break

env.close()
