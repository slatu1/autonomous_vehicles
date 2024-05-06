import math
import matplotlib.pyplot as plt
import numpy as np

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
        match i:
            case 0:   # Turn left 90deg
                # call system_dynamics and store distnace to wp after det
                options.append(system_dynamics(x_init, y_init, theta_init, -90, destination, usense_data))
                print("\nOption L 90\n" + str(options[0]) + "\n")
            case 1:
                options.append(system_dynamics(x_init, y_init, theta_init, -45, destination, usense_data))
                print("\nOption L 45\n" + str(options[1]) + "\n")
            case 2:
                options.append(system_dynamics(x_init, y_init, theta_init, 45, destination, usense_data))
                print("\nOption R 45\n" + str(options[2]) + "\n")            
            case 3:
                options.append(system_dynamics(x_init, y_init, theta_init, 90, destination, usense_data))
                print("\nOption R 90\n" + str(options[3]) + "\n")            
            case _:
                print("eh")
        xpts.append(options[i][2])
        ypts.append(options[i][3])
    acts = []
    for i in range(0, len(xpts)):
        action = [[x_init, y_init],[xpts[i],ypts[i]]]
        print(str(action))
        acts.append(action)
        plt.plot([x_init,xpts[i]],[y_init,ypts[i]])
        
    # calculate rewards -- apply negative reward to options where an obstacle exists in the path
    print("Options\n")
    for i in options:
        print(i)
        rewards.append(reward(i, usense_data))
    print("\nRewards - " + str(rewards))
    # find the highest
    h_pick = max(rewards)
    act = rewards.index(h_pick)
    print("\nAction " + str(act) + "chosen: " + str(options[act]) + "\nReward - " + str(h_pick))
    print("\nDetour waypoint - (" + str(options[act][2]) + ", " + str(options[act][3]) + ")")
    detour_wp = [options[act][2],options[act][3]]
    # send new coordinates to arduino 
    
    #plots 
    plt.scatter(x_init,y_init, color='yellow')
    plt.scatter(destination[0], destination[1], color='green')
    plt.plot([x_init,destination[0]], [y_init, destination[1]], linestyle='dotted')
    plt.scatter(xpts,ypts, color='blue')
    plt.scatter(detour_wp[0],detour_wp[1], color='pink')
    plt.plot([detour_wp[0],destination[0]],[detour_wp[1],destination[1]],color='orange',linestyle='dotted')
    
    draw_obstacle(x_init,y_init,theta_init,usense_data)    
    plt.scatter(obstacle_x, obstacle_y, color='red')
    plt.grid()
    plt.xlim(0, 50)
    plt.ylim(0,50)
    plt.show()
    return options[act][2], options[act][3]       

def reward(action, us_data):
    d = action[1]
    p = action[4]
    rwd = (1/(d*d))+(p*p)
    return rwd
          
def localize(x, y, t, xi, yi, d):
#    // determine distance traveled in each direction    
    # quad 1
    if (t > 0) & (t < 45):
      quad = 1
    elif  (t > 45) & (t < 90):
      quad = 1
    # quad 2
    elif (t > 90) & (t < 135):
      quad = 2
    elif  (t > 135) & (t < 180):
      quad = 2
    # quad 3
    elif (t > 180) & (t < 225):
      quad = 3
    elif  (t > 225) & (t < 270):   
      quad = 3
    # quad 4
    elif (t > 270) & (t < 315):
      quad = 4
    elif  (t > 315) & (t < 360):   
      quad = 4

    # convert temp_tetha to rad for python math
    temp_rad = t*(np.pi/180)
    print("\n temp rad = " + str(temp_rad))
    
    # temp_deg = temp_rad*(180/np.pi)
    # print("\n temp deg = " + str(temp_deg) + "\n")
      
    temp_x = abs(d*(np.cos(temp_rad))) 
    temp_y = abs(d*(np.sin(temp_rad)))
    # print("cos theta = " + str(np.cos(temp_theta)) + "\n")
    # print("sin theta = " + str(np.sin(temp_theta)) + "\n")
    # print("\ni[0] = " + str(i[1]) + "\nElsey = " + str((np.sin(temp_rad)*180/np.pi))+ "\nElsex = " + str((np.cos(temp_rad)*180/np.pi)))
    print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
    match(quad):
      case 1:
            temp_x += x
            temp_y += y
      case 2:
            temp_x = x - temp_x
            temp_y += y   
      case 3:
            temp_x = x - temp_x
            temp_y = y - temp_y  
      case 4:
            temp_x += x
            # temp_y -= y
            temp_y = y - temp_y 
    # print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
    print("\nQuad - " + str(quad) + "\n")
    # print("\ntheta = " + str(temp_theta) + "\nobs_x = " + str(temp_x) + "\nobs_y = " + str(temp_y) + "\n")             
    
    x_pos = temp_x
    y_pos = temp_y    
    print("Localizing \n x_pos = " + str(x_pos) + "\ty_pos = " + str(y_pos) + "\ttheta_d " + str(t) + "\n")
    xpts.append(x_pos)
    ypts.append(y_pos)
    return x_pos, y_pos          
            
def system_dynamics(x_init, y_init, theta_init, turn, goal, sensor_data):
    
    detour_len = 10
    xr_temp = goal[0]
    yr_temp = goal[1]
    theta_d = 0  
      
    # true angle = current theta_d - turn angle
    abs_theta = theta_init - turn
    temp_theta = turn - theta_init
    if(abs_theta < 0):
        abs_theta = 360 + abs_theta
    elif(abs_theta > 360):
        abs_theta = 360 - abs_theta
    print(str(abs_theta))

    # quad 1
    if (abs_theta > 0) & (abs_theta < 45):
        temp_theta = turn - abs_theta
        quad = 1
    elif  (abs_theta > 45) & (abs_theta < 90):
        temp_theta = 90 - abs_theta
        quad = 1
    # quad 2
    elif (abs_theta > 90) & (abs_theta < 135):
        temp_theta = turn - abs_theta
        quad = 2
    elif  (abs_theta > 135) & (abs_theta < 180):
        temp_theta = 90 - abs_theta 
        quad = 2
    # quad 3
    elif (abs_theta > 180) & (abs_theta < 225):
        temp_theta = turn - abs_theta
        quad = 3
    elif  (abs_theta > 225) & (abs_theta < 270):
        temp_theta = 90 - abs_theta   
        quad = 3
    # quad 4
    elif (abs_theta > 270) & (abs_theta < 315):
        temp_theta = turn - abs_theta
        quad = 4
    elif  (abs_theta > 315) & (abs_theta < 360):
        temp_theta = 360 - abs_theta   
        quad = 4

    # convert temp_tetha to rad just for python math
    temp_rad = temp_theta*(np.pi/180)
    # print("\n temp rad = " + str(temp_rad))

    temp_deg = temp_rad*(180/np.pi)
    # print("\n temp deg = " + str(temp_deg) + "\n")
       
    # localize -- determine init position after detour    
    new_x,new_y = localize(x_init,y_init,abs_theta,x_init,y_init,detour_len)   

   # calculate distance to goal from new x_init
    theta_d = (math.atan((yr_temp-new_y)/(xr_temp-new_x))*180)/math.pi;
    d_rem = math.sqrt(((xr_temp-new_x)*(xr_temp-new_x))+((yr_temp-new_y)*(yr_temp-new_y)));
   
   # calculate distance from edge of obstacles
    possibile_ps = []
    p_temp = 0
    for i in sensor_data:
        print(str(i))
        theta_p = abs(temp_theta-i[0])*np.pi/180
        p_temp = abs(math.tan(theta_p)*i[1])
        print("p - " + str(p_temp))
        possibile_ps.append(p_temp)      
    p = max(possibile_ps)
    # return distance and theta    
    return [theta_d, d_rem, new_x, new_y, p]
   
def draw_obstacle(x,y,t,data):
    # print(str(t))
    for i in data:
        # print(str(i))
        
        temp_x = 0
        temp_y = 0
        # temp_theta = t + i[0] 
        abs_theta = t - i[0]
        temp_theta = i[0] - t
        if(abs_theta < 0):
            abs_theta = 360 + abs_theta
        elif(abs_theta > 360):
          abs_theta = 360 - abs_theta
        # print(str(abs_theta))
        
        # quad 1
        if (abs_theta > 0) & (abs_theta < 45):
          temp_theta = i[0] - abs_theta
          quad = 1
        elif  (abs_theta > 45) & (abs_theta < 90):
          temp_theta = 90 - abs_theta
          quad = 1
        # quad 2
        elif (abs_theta > 90) & (abs_theta < 135):
          temp_theta = i[0] - abs_theta
          quad = 2
        elif  (abs_theta > 135) & (abs_theta < 180):
          temp_theta = 90 - abs_theta 
          quad = 2
        # quad 3
        elif (abs_theta > 180) & (abs_theta < 225):
          temp_theta = i[0] - abs_theta
          quad = 3
        elif  (abs_theta > 225) & (abs_theta < 270):
          temp_theta = 90 - abs_theta   
          quad = 3
        # quad 4
        elif (abs_theta > 270) & (abs_theta < 315):
          temp_theta = i[0] - abs_theta
          quad = 4
        elif  (abs_theta > 315) & (abs_theta < 360):
          temp_theta = 360 - abs_theta   
          quad = 4

        # convert temp_tetha to rad just for python math
        temp_rad = temp_theta*(np.pi/180)
        # print("\n temp rad = " + str(temp_rad))
        
        temp_deg = temp_rad*(180/np.pi)
        # print("\n temp deg = " + str(temp_deg) + "\n")
         
        # temp_x = abs(i[1]*(np.cos(temp_rad))) 
        # temp_y = abs(i[1]*(np.sin(temp_rad)))
        # print("cos theta = " + str(np.cos(temp_theta)) + "\n")
        # print("sin theta = " + str(np.sin(temp_theta)) + "\n")
        # print("\ni[0] = " + str(i[1]) + "\nElsey = " + str((np.sin(temp_rad)*180/np.pi))+ "\nElsex = " + str((np.cos(temp_rad)*180/np.pi)))
        # print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
        # match(quad):
        #   case 1:
        #         temp_x += x
        #         temp_y += y
        #   case 2:
        #         temp_x = x - temp_x
        #         temp_y += y   
        #   case 3:
        #         temp_x = x - temp_x
        #         temp_y = y - temp_y  
        #   case 4:
        #         temp_x += x
        #         # temp_y -= y
        #         temp_y = y - temp_y 
        # # print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
        # print("\nQuad - " + str(quad) + "\n")
        # print("\ntheta = " + str(temp_theta) + "\nobs_x = " + str(temp_x) + "\nobs_y = " + str(temp_y) + "\n")             
        temp_x,temp_y = localize(x,y,abs_theta,x,y,i[1])
        obstacle_x.append(temp_x)
        obstacle_y.append(temp_y)         
       
    # print(str(len(obstacle_x)))  