import math, serial
# import matplotlib.pyplot as plt
from policy_iter_test import policy_iter

#raspi side
# once arduino detects obstacle, it will be reported to the raspi and arduino will wait for instruction 
# raspi expects to recieve "Obstacle detected", x_now, y_now, theta_d, [sensor readings at each angle]
# send detour flag with destination report -- or just self activate goal waypoint once detour destination 
# is reached

dest = [36, 45] # destination
# dest = [21, 15] # destination

# "current" position
# x = 10  # x position
# y = 25  # y position
x = 10  # x position
y = 25  # y position
theta_d = (math.atan((dest[1]-y)/(dest[0]-x))*180)/math.pi; # heading angle toward destination
print(str(theta_d) + "\n")

#esablish comms with arduino
# ser = serial.Serial('/dev/ttyUSB0')  # open serial port
# print(ser.name)         # check which port was really used
connected = 1           #set connection flag

def scan():    
    input = []
    # data = [[-90,43.45],[-70,65.45],[-50,30.45],[-30,8.5],[0,7.45],[30,8.45],[50,70.45],[70,53.45],[90,38.45]]
    # data = [[-90,23.45],[-70,65.45],[-50,30.45],[-30,10.5],[0,5.45],[30,6.45],[50,10.45],[70,23.45],[90,38.45]]
    data = [[-90,6.45],[-70,12.45],[-50,17.45],[-30,80.5],[0,5.45],[30,6.45],[50,10.45],[70,23.45],[90,38.45]]
    # wait for arduino to be ready before storing values
    # rready = ser.readline()
    # if(rready == "Ready"):
    #     input = ser.readline().split()
    #     while input[0] != "Done":
    #         data.append(input) # store data
    #         ser.write(b'1\n'); # send ack
            
    return data        
            
def detour():
    # send request for environment scan
    # ser.write(b'3\n')
    usound_data = scan() #10 entries
    # send request for current data
    # ser.write(b'2\n')
    # read information
    # report = ser.readline()
    # system_data = report.split() # should be array of 3 items
    system_data = [x, y, theta_d]
    detour_wp = policy_iter(system_data, usound_data, dest)
    xd = detour_wp[0]
    yd = detour_wp[1]
    # ser.write("1 " + xd + " " + yd + "\n")
    
# establish comms with arduino
# send arduino destination coordinates
# wait for arduino to report an obstacle
# provide detour coordinates to arduino 
if __name__ == "__main__":
    # ser.write("1 10 15\n") #initial goal point
    # while connected loop
    detour()
    # while connected:
    #     # read serial
    #     # report = ser.readline()   # read a '\n' terminated line
    #     # print(report)
    #     # rep_ort = report.split();
    #     if(rep_ort[0] == "Obstacle"):
    #         detour()
    #     elif(rep_ort[0] == "Destination"):
    #         print("Robot has reached destination!")
    #         ser.close()             # close port 
    #         connected = 0         
    



          
            
            
  