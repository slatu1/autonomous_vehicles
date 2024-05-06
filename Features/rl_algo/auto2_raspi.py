import math, serial, time
from policy_iter_fin import policy_iter
#raspi side
# once arduino detects obstacle, it will be reported to the raspi and arduino will wait for instruction 
# raspi expects to recieve "Obstacle detected", x_now, y_now, theta_d, [sensor readings at each angle]
# send detour flag with destination report -- or just self activate goal waypoint once detour destination 
# is reached

dest = [16, 25]

x = 10  # x position
y = 25  # y position
theta_d = 23.67
# corresponds to how far the robot is from the obstacles 
#               L90    L45   R45   R90
sensor_data = [3.556, 10.32, 6.2, 4.75]


detour_len = 10
temp_theta = 0
new_x = 0
new_y = 0
options = []

#esablish comms with arduino
ser = serial.Serial('/dev/ttyUSB0')  # open serial port
print(ser.name)         # check which port was really used
connected = 1           #set connection flag

def scan():    
    input = []
    data = []
    # wait for arduino to be ready before storing values
    # rready = ser.readline()
    input = ser.readline().split()
    while input[0] != "Done":
        if(input != " "):
            data.append(input) # store data
            ser.write(b'1 \n'); # send ack
        time.sleep(2)
    # if(rready == "Ready"):
    #     input = ser.readline().split()
    #     while input[0] != "Done":
    #         data.append(input) # store data
    #         ser.write(b'1\n'); # send ack
    #         time.sleep(2)
    return data        
            
def detour():
    # send request for environment scan
    ser.write(b'3 \n')
    time.sleep(2)
    usound_data = scan() #10 entries
    # send request for current data
    ser.write(b'2 \n')
    time.sleep(2)
    # read information
    report = ser.readline()
    system_data = report.split() # should be array of 3 items
    detour_wp = policy_iter(system_data, usound_data, dest)
    xd = detour_wp[0]
    yd = detour_wp[1]
    ser.write(bytearray("1 " + xd + " " + yd + " \n", 'ascii'))
    time.sleep(2)
    
# establish comms with arduino
# send arduino destination coordinates
# wait for arduino to report an obstacle
# provide detour coordinates to arduino 
if __name__ == "__main__":
    print("Entering..\n")
    ser.write(bytearray("1 10 15 \n", 'ascii'))
    time.sleep(2)
    #while connected loop
    while connected:
        # read serial
        report = ser.readline()   # read a '\n' terminated line
        print(report)
        rep_ort = report.split();
        if(rep_ort[0] == "Obstacle"):
            detour()
        elif(rep_ort[0] == "Destination"):
            print("Robot has reached destination!")
            ser.close()             # close port 
            connected = 0
        time.sleep(2)   
    print("\nHost has disconnected from robot\n")           
    



          
            
            
  