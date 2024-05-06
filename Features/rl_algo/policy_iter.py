import math
import matplotlib.pyplot as plt
# from auto2_raspi_test import xr,yr

# actions
# - L90
# - L45
# - R45
# - R90
actions = [["L",90],["L",45],["R",45],["R",90]];
options = []
rewards = []

xpts = []
ypts = []
obstacle_x = []
obstacle_y = []

# evaluate each option
# want to estimate the distance away from the goal the robot will be after each detour option
# determine new theta_d
# determine new coordinates after moving 5-10 steps in theta_d direction
# determine the distance away from the that point to the goal
# store that ^ distance and evaluate the reward 

# check if obstacle detected has 15deg clearance for robot
# rule out any theta_d that will cause collision with obstacle (check sensor readings against possible actions)
def policy_iter(system_data, usense_data, destination):
    x_init = system_data[0]
    y_init = system_data[1]
    theta_init = system_data [2]
    xpts.append(x_init)
    ypts.append(y_init)    
    
    print("\nx_init - " + str(x_init) + "\ny_init - " + str(y_init) + "\ntheta - " + str(theta_init) + "\nObstacles - " + str(usense_data) + "\nGoal - " + str(destination))
    for i in range(0,4):
        print(actions[i])
        # call system_dynamics and store distnace to wp after det
        match i:
            case 0:   # Turn left 90 deg
                options.append(system_dynamics(x_init, y_init, theta_init, -90, destination, usense_data))
            case 1:   # Turn left 45 deg
                options.append(system_dynamics(x_init, y_init, theta_init, -45, destination, usense_data))
            case 2:   # Turn right 45 deg
                options.append(system_dynamics(x_init, y_init, theta_init, 45, destination, usense_data))       
            case 3:   # Turn right 90 deg
                options.append(system_dynamics(x_init, y_init, theta_init, 90, destination, usense_data))         
            case _:
                print("n/a")
                
    # calculate rewards -- apply negative reward to options where an obstacle exists in the path
    print("Options\n")
    for i in options:
        print(i)
        rewards.append(reward(i, usense_data))
    print("\nRewards - " + str(rewards))
    # find the highest
    h_pick = max(rewards)
    act = rewards.index(h_pick)
    print("\nAction chosen: " + str(options[act]) + "\nReward - " + str(h_pick))
    print("\nDetour waypoint - (" + str(options[act][2]) + ", " + str(options[act][3]) + ")")
    
    return options[act][2], options[act][3]       

def reward(action, us_data):
    d = action[1]
    p = action[4]
    rwd = (1/(d*d))+(p*p)
    return rwd
            
def system_dynamics(x_init, y_init, theta_init, turn, goal, sensor_data):
    
   temp_theta = theta_init + turn 
   detour_len = 10
   xr_temp = goal[0]
   yr_temp = goal[1]
   theta_d = 0
   
   # determine init position after detour
   new_x = detour_len*math.cos(temp_theta) + x_init
   new_y = (detour_len*math.sin(temp_theta) + y_init)#*-1
   print("Localizing \n x_pos = " + str(new_x) + "\ty_pos = " + str(new_y) + "\ttheta_d " + str(temp_theta) + "\n")
       
   # calculate distance to goal from new x_init
   theta_d = (math.atan((yr_temp-new_y)/(xr_temp-new_x))*180)/math.pi;
   d_rem = math.sqrt(((xr_temp-new_x)*(xr_temp-new_x))+((yr_temp-new_y)*(yr_temp-new_y)));
   
   #find closest obstcles of that path
   # calculate distance from edge of obstacle
   possibile_ps = []
   p_temp = 0
   for i in sensor_data:
       # if angle is close enough to care about distance 
       # only want one option to calculate p
        if i[1] < 15 & abs(temp_theta-i[0]) < 20:
            print(str(i))
            p_temp = abs(math.tan(temp_theta-i[0])*i[1])
            print("p - " + str(p_temp)) 
        else:
            p_temp = 0
            print("p - " + str(p_temp))
        possibile_ps.append(p_temp)      
   p = max(possibile_ps)
   
   # return distance and theta    
   return [theta_d, d_rem, new_x, new_y, p]
