import gym
import gym_panda
import numpy as np
import time

def calculate_pd(dx, dy, dz, k_p, k_d, dt):
    return k_p * dx + k_d * dx / dt, k_p * dy + k_d * dy / dt, k_p * dz + k_d * dz / dt


env = gym.make('panda-v0')
observation = env.reset()
done = False
error = 0.01
fingers = 1
###
object_position = [0.7, 0, 0.1]


stage = 0
k_p = 20
k_d = 1

# Circular movement parameters
circle_radius = 0.3
angular_velocity = 0.1
total_time_circular = 2 * np.pi / angular_velocity
t_circular = 0
time_step = 1.0 / 240.0
dt = 1.0 / 240.0  # the default timestep in pybullet is 240 Hz
t = 0
order = 1

for i_episode in range(20):
   observation = env.reset()
   fingers = 1
   for t in range(700):
        env.render()

        if stage==0:
           dx_ = object_position[0]-observation[0]
           dy_ = object_position[1]-observation[1] 
           dz_ = object_position[2]-observation[2]
           if abs(dx_) < error and abs(dy_) < error and abs(dz_) < error:
               fingers = 0
               if (observation[3]+observation[4])<error+0.02 and fingers==0:
                   stage = 1
                   #env.diplay_text1(stage)
               
        elif stage==1:
            target_z = 0.5
            target_y = 0
            target_x = 0.4
            dx_ = target_x-observation[0]
            dy_= target_y-observation[1]
            dz_ = target_z-observation[2]
            
            if abs(dx_) < error and abs(dy_) < error and abs(dz_) < error:
                
                stage = 2
                env.diplay_text1(stage)

        elif stage==2:
            target_z = 0.05
            target_y = 0
            target_x = 0.4
            dx_ = target_x-observation[0]
            dy_= target_y-observation[1]
            dz_ = target_z-observation[2]
            if (abs(dx_) < error and abs(dy_) < error and abs(dz_) < error):
               stage = 3

        elif stage==3:
            fingers = 1
            if (observation[3]+observation[4])>error+0.02 and fingers==1:
                target_z = 0.2
                target_y = 0
                target_x = 0.4
                dx_ = target_x-observation[0]
                dy_= target_y-observation[1]
                dz_ = target_z-observation[2]
                if (abs(dx_) < error and abs(dy_) < error and abs(dz_) < error):
                    stage = 4
        #circular movement 
        elif stage==4:
    
            target_z = 0.2
            target_y = 0
            target_x = 0.4
            target_x = target_x + circle_radius * np.cos(angular_velocity * t)
            target_y = target_y + circle_radius * np.sin(angular_velocity * t)
            

            dx_ = target_x - observation[0]
            dy_ = target_y - observation[1]
            dz_ = target_z - observation[2]

            t_circular += 1

            if t_circular == 55:
                stage = 5

        elif stage==5:
            target_x, target_y, target_z = env.get_object_loc()
            dx_ = target_x-observation[0]
            dy_= target_y-observation[1]
            dz_ = target_z-0.014-observation[2]
            if abs(dx_) < error and abs(dy_) < error and abs(dz_)< error:
               fingers = 0
               if (observation[3]+observation[4])<error+0.02 and fingers==0:
                   stage = 6

        elif stage==6:
            target_z = 0.5
            target_y = 0
            target_x = 0.4
            dx_ = target_x-observation[0]
            dy_= target_y-observation[1]
            dz_ = target_z-observation[2]
            if abs(dx_) < error and abs(dy_) < error and abs(dz_) < error:
                stage = 7
       
        # second tray
        elif stage==7:
            target_z = 0.05
            target_y = -0.45
            target_x = 0.4
            dx_ = target_x-observation[0]
            dy_= target_y-observation[1]
            dz_ = target_z-observation[2]
            if (abs(dx_) < error and abs(dy_) < error and abs(dz_) < error):
               stage = 8

        elif stage==8:
            fingers = 1
            if (observation[3]+observation[4])>error+0.02 and fingers==1:
                target_z = 0.5
                target_y = 0
                target_x = 0.4
                dx_ = target_x-observation[0]
                dy_= target_y-observation[1]
                dz_ = target_z-observation[2]
                if (abs(dx_) < error and abs(dy_) < error and abs(dz_) < error):
                    t = 500
                    env.reset()
                    stage = 0

        default: stage = 0
                  
        print(stage)
        pd_x, pd_y, pd_z = calculate_pd(dx_, dy_, dz_, k_p, k_d, dt)
        action = [pd_x, pd_y, pd_z, fingers]
        observation, reward, done, info = env.step(action)
        
        object_position = info['object_position']
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()