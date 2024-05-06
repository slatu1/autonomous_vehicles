import math
import matplotlib.pyplot as plt
import numpy as np

# from policy_iter_test import policy_iter

# long distance,a1,a2;//define three distance

step_size=0.01
# step_size=1
# float x_pos;
# float y_pos;
# float d_rem;
# float d_trav;
# float theta;
# float active_wp[2] = {0,0};
# float xg;
# float yg;
# float xd;
# float yd;
# int done = 0;
# int num_wp = 10;
# String input;
# int detour_active = 0;

# wp_map[num_wp][2]={{0,0},{2,3},{0,4},{2,4},{3,5},{1,1},{0,0},{2,3},{0,4},{2,4}};

xd = 10
yd = 25

done = 0

xpts = []
ypts = []
obstacle_x = []
obstacle_y = []

def localize(x, y, t, xi, yi, d):
#    // determine distance traveled in each direction
    # convert theta to rad for python math
    # t = t * np.pi/180;
    # x_pos = d*np.cos(t) + xi;
    # y_pos = (d*np.sin(t) + yi)
    
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

    # convert temp_tetha to rad just for python math
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



def go(xr, yr):
#   Serial.print("Waypoint = {");
#   Serial.print(xr,3);
#   Serial.print(",");
#   Serial.print(yr,3);
#   Serial.println("}");
  #while(done != 1):
    x_init = 0
    y_init = 0
    theta = 0
    d_trav = 0
    x_pos = x_init
    y_pos = x_init
    
    # data = [[-90,43.45],[-70,65.45],[-50,30.45],[-30,8.5],[0,7.45],[30,8.45],[50,70.45],[70,53.45],[90,38.45]]
    data = [[-90,5],[-70,5],[-50,5],[-30,5],[0,5],[30,5],[50,5],[70,5],[90,5]]
    # data = [[-90,5],[-45,5],[0,5],[45,5],[90,5]]
    # // orient
    print("- Setting Orientation -\n")
    # delay(200);
    theta = (np.arctan((yr-y_pos)/(xr-x_pos))*180)/np.pi  
    print("- Orientation Achieved -\ntheta_d = " + str(theta) + "\n")

    # //determine distance
    print("- Calculating Distance to Waypoint -\n") #// from initial position
    d_rem = np.sqrt(((xr-x_init)*(xr-x_init))+((yr-y_init)*(yr-y_init)))
    print("d_rem = " + str(d_rem) + "\n");
    # //loop until waypoint is reached 
  
    print("- Traveling to Waypoint -\n")
    steps = 100
    while(d_rem > 7.7):  
    # while(steps >= 0):          
    #   //check distance from objects (if objects need to be avoided)

    #   car_front();//go forward
      d_trav += step_size
      x_pos, y_pos = localize(x_pos, y_pos, theta, x_init, y_init, d_trav); #//update x and y pos
      d_rem = np.sqrt(((xr-x_pos)*(xr-x_pos))+((yr-y_pos)*(yr-y_pos))) #// distance from current position
      print("d_trav = " + str(d_trav) + "\td_rem = " + str(d_rem) + "\n")
      steps-=1

    done = 1
    print("Destination Reached\n")
    draw_obstacle(x_pos,y_pos,theta,data)
    plt.scatter(xpts, ypts)
    plt.scatter(xd,yd,color='green')
    plt.scatter(obstacle_x, obstacle_y, color='red')
    plt.grid()
    plt.show()
    x_init = x_pos
    y_init = y_pos
    
    
  



def detour():
#   // rl algo
#   // determine possible detour paths [-90, -60, -30, 30, 60, 90]degrees and 5 units up
#   // calculate the reward for each policy 

    usound_data = [[-90,43.45],[-70,65.45],[-50,30.45],[-30,8.5],[0,7.45],[30,8.45],[50,70.45],[70,53.45],[90,38.45]]



# def draw_obstacle(x,y,t,data):
#     print(str(t))
#     for i in data:
#         print(str(i))
        
#         temp_x = 0
#         temp_y = 0
#         # temp_theta = t + i[0] 
#         abs_theta = t - i[0]
#         temp_theta = i[0] - t
#         if(abs_theta < 0):
#             abs_theta = 360 + abs_theta
#         elif(abs_theta > 360):
#           abs_theta = 360 - abs_theta
#         print(str(abs_theta))
        
#         # quad 1
#         if (abs_theta > 0) & (abs_theta < 45):
#           temp_theta = i[0] - abs_theta
#           quad = 1
#         elif  (abs_theta > 45) & (abs_theta < 90):
#           temp_theta = 90 - abs_theta
#           quad = 1
#         # quad 2
#         elif (abs_theta > 90) & (abs_theta < 135):
#           temp_theta = i[0] - abs_theta
#           quad = 2
#         elif  (abs_theta > 135) & (abs_theta < 180):
#           temp_theta = 90 - abs_theta 
#           quad = 2
#         # quad 3
#         elif (abs_theta > 180) & (abs_theta < 225):
#           temp_theta = i[0] - abs_theta
#           quad = 3
#         elif  (abs_theta > 225) & (abs_theta < 270):
#           temp_theta = 90 - abs_theta   
#           quad = 3
#         # quad 4
#         elif (abs_theta > 270) & (abs_theta < 315):
#           temp_theta = i[0] - abs_theta
#           quad = 4
#         elif  (abs_theta > 315) & (abs_theta < 360):
#           temp_theta = 360 - abs_theta   
#           quad = 4

#         # convert temp_tetha to rad just for python math
#         temp_rad = temp_theta*(np.pi/180)
#         print("\n temp rad = " + str(temp_rad))
        
#         temp_deg = temp_rad*(180/np.pi)
#         print("\n temp deg = " + str(temp_deg) + "\n")
         
#         # temp_x = abs(i[1]*(np.cos(temp_rad))) 
#         # temp_y = abs(i[1]*(np.sin(temp_rad)))
#         # print("cos theta = " + str(np.cos(temp_theta)) + "\n")
#         # print("sin theta = " + str(np.sin(temp_theta)) + "\n")
#         # print("\ni[0] = " + str(i[1]) + "\nElsey = " + str((np.sin(temp_rad)*180/np.pi))+ "\nElsex = " + str((np.cos(temp_rad)*180/np.pi)))
#         # print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
#         # match(quad):
#         #   case 1:
#         #         temp_x += x
#         #         temp_y += y
#         #   case 2:
#         #         temp_x = x - temp_x
#         #         temp_y += y   
#         #   case 3:
#         #         temp_x = x - temp_x
#         #         temp_y = y - temp_y  
#         #   case 4:
#         #         temp_x += x
#         #         # temp_y -= y
#         #         temp_y = y - temp_y 
#         # # print("\ndist_x = " + str(temp_x) + "\tdist_y = " + str(temp_y))
#         # print("\nQuad - " + str(quad) + "\n")
#         # print("\ntheta = " + str(temp_theta) + "\nobs_x = " + str(temp_x) + "\nobs_y = " + str(temp_y) + "\n")             
#         temp_x,temp_y = localize(x,y,abs_theta,x,y,i[1])
#         obstacle_x.append(temp_x)
#         obstacle_y.append(temp_y)         
       
#     print(str(len(obstacle_x)))   

if __name__ == "__main__":
    # ser.write("1 10 15\n") #initial goal point
    # while connected loop
     go(xd,yd)
