# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

import os
import signal
import numpy as np
from threading import Thread
import time
import csv

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar import QLabsQCar
from qvl.spline_line import QLabsSplineLine
from qvl.real_time import QLabsRealTime


tf =500
startDelay = 1
controllerUpdateRate = 100

K_p = -2.0
K_d = -1.5
time_shift = 250

#For trajectory Derivative
error_array_1p = []
error_array_2p = []
error_array_3p = []

#For mutual Derivative
error_array_1f = []
error_array_2f = []
error_array_3f = []

#For Point storing
theta1_array = []
theta2_array = []
theta3_array = []

S0 = [] #To record the timestamp
#For each thetas
S1 = []
S2 = []
S3 = []

#Velocity
V1 = []
V2 = []
V3 = []


#For Steering Control
error_array = []
error_array2 = []
error_array3 = []

time_counter_array = []


enableSteeringControl = True
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

default_rtModel = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        '../../spawn_models/QCar_Workspace'
    )
)


# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

FREQ = 100
CYCLETIME = 1/FREQ


class SpeedController:

    def __init__(self, kp, kd):
        self.kp = kp
        self.ei_1 = 0
        self.kd1 = kd


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, theta1, theta2, dt, t):     
           
        if t<time_shift:
            # Desired Poition
            theta_ref_1 = 0*2*(np.pi/3) + (5/20) * t
            
            # Trajectory Error and Derivative
            error1 = theta1 - theta_ref_1
            error_net = error1
            if len(error_array_1p) == 0:
                error_array_1p.append(0)
                error_array_1p.append(error_net)
            else:
                error_array_1p.append(error_net)
            error_der1p =((error_array_1p[len(error_array_1p) -1] - error_array_1p[len(error_array_1p) - 2])/dt) * self.kd1
            output1p = self.kp * error_net + error_der1p  
            output1f = 0
            self.ei_1 +=dt*output1p
            v1 = self.ei_1 * 1
            v1 = np.clip(v1, 0, 35)
       
        else:
            theta_ref_1 = 0*2*(np.pi/3) + (6/20) * t - ((6* time_shift/20) - (5* time_shift/20))
            error1 = theta1 - theta_ref_1
            error_net = error1
            error_array_1p.append(error_net)
            error_der1p =((error_array_1p[len(error_array_1p) -1] - error_array_1p[len(error_array_1p) - 2])/dt) * self.kd1
            output1p = self.kp * error_net + error_der1p  
            
            #Formation Error
            error12 = -(theta2 - theta1 - 1*2*(np.pi/3))
            error_net_new = error12 
           
            if len(error_array_1f) == 0:
                error_array_1f.append(0)
                error_array_1f.append(error_net_new)
            else:
                error_array_1f.append(error_net)
            error_der1f =((error_array_1f[len(error_array_1f) -1] - error_array_1f[len(error_array_1f) - 2])/dt) * self.kd1

            output1f = (self.kp * error_net_new) + error_der1f  
            #Output from Position error + Formation error
            output_net = output1p + output1f
            self.ei_1 +=dt*output_net
            v1 = self.ei_1 * 1
            v1 = np.clip(v1, 0, 35)
           

        return v1, output1p, output1f

class SpeedController2:

    def __init__(self, kp, kd):
        self.kp = kp
        self.ei_2 = 0
        self.kd1 = kd
       
    # ==============  SECTION A -  Speed Control  ====================
    def update(self, theta1, theta2, theta3, dt, t):     
       
        if t<time_shift:
             # Desired Poition            
            theta_ref_2 = 1*2*(np.pi/3) + (5/20) * t
            
            # Trajectory Error and Derivative
            error2 = theta2 - theta_ref_2
            if len(error_array_2p) == 0:
                error_array_2p.append(0)
                error_array_2p.append(error2)
            else:
                error_array_2p.append(error2)
            error_der2p =((error_array_2p[len(error_array_2p) -1] - error_array_2p[len(error_array_2p) - 2])/dt) * self.kd1
       
            output2p = (self.kp * error2) + error_der2p 
            output2f = 0
            self.ei_2 +=dt*output2p
            v2 = self.ei_2 * 1
            v2 = np.clip(v2, 0, 35)
        else:
            theta_ref_2 = 1*2*(np.pi/3) + (6/20) * t - ((6* time_shift/20) - (5* time_shift/20))
            error2 = theta2 - theta_ref_2
            error_array_2p.append(error2)
            error_der2p =((error_array_2p[len(error_array_2p) -1] - error_array_2p[len(error_array_2p) - 2])/dt) * self.kd1
            output2p = (self.kp * error2) + error_der2p 
            
            #Formation Error
            error21 =  (theta2 - theta1 - 1*2*(np.pi/3))
            error23 = -(theta3 - theta2 - 1*2*(np.pi/3))
            error_net2 = error21 + error23
            
            if len(error_array_2f) == 0:
                error_array_2f.append(0)
                error_array_2f.append(error_net2)
            else:
                error_array_2f.append(error_net2)
            error_der2sp =((error_array_2f[len(error_array_2f) -1] - error_array_2f[len(error_array_2f) - 2])/dt) * self.kd1
       
            output2f = (self.kp * error_net2) + error_der2sp 
            #Output from Position error + Formation error
            output_net = output2p + output2f
            self.ei_2 +=dt*output_net
            v2 = self.ei_2 * 1
            v2 = np.clip(v2, 0, 35)
           
        return v2, output2p, output2f
 
class SpeedController3:

    def __init__(self, kp, kd):
        self.kp = kp
        self.ei_3 = 0
        self.kd1 = kd

    # ==============  SECTION A -  Speed Control  ====================
    def update(self, theta2, theta3, dt, t):

        if t<time_shift:     
            # Desired Poition            
            theta_ref_3 = 2*2*(np.pi/3) + (5/20) * t
            
            # Trajectory Error and Derivative
            error3 = theta3 - theta_ref_3        
            if len(error_array_3p) == 0:
                error_array_3p.append(0)
                error_array_3p.append(error3)
            else:
                error_array_3p.append(error3)
            error_der3p =((error_array_3p[len(error_array_3p) -1] - error_array_3p[len(error_array_3p) - 2])/dt) * self.kd1
       
            output3p = self.kp * error3 + error_der3p
            output3f = 0
            self.ei_3 +=dt*output3p
            v3 = self.ei_3  * 1
            v3 = np.clip(v3, 0, 35)
        else:
            theta_ref_3 = 2*2*(np.pi/3) + (6/20) * t - ((6* time_shift/20) - (5* time_shift/20))
            error3 = theta3 - theta_ref_3
            error_array_3p.append(error3)
            error_der3p =((error_array_3p[len(error_array_3p) -1] - error_array_3p[len(error_array_3p) - 2])/dt) * self.kd1
            output3p = self.kp * error3 + error_der3p
            
            #Formation Error
            error32 =  (theta3 - theta2 - 1*2*(np.pi/3))
            error_net_3 = error32
            
            if len(error_array_3f) == 0:
                error_array_3f.append(0)
                error_array_3f.append(error_net_3)
            else:
                error_array_3f.append(error_net_3)
            error_der3f =((error_array_3f[len(error_array_3f) -1] - error_array_3f[len(error_array_3f) - 2])/dt) * self.kd1
       
            output3f = (self.kp * error_net_3) + error_der3f
            #Output from Position error + Formation error
            output_net3 = output3p + output3f
            self.ei_3 +=dt*output_net3
            v3 = self.ei_3  * 1
            v3 = np.clip(v3, 0, 35)
           
        return v3, output3p, output3f




class SteeringController:
    def update(self, p, dt, kp1, kd1):
        radius1 =20
        x1 = p[0]
        y1 = p[1]
        
        # Compute the sign of the point
        x12 = x1 * x1
        y12 = y1 * y1
        radius11 = radius1 * radius1
        sign1 = x12 + y12 - radius11
        
        theta1 = np.arctan2(y1,x1)
        x1_d = (np.cos(theta1) * radius1)
        y1_d = (np.sin(theta1) * radius1)
        
        # Compute the Error
        x1_e = x1_d - x1
        y1_e = y1_d - y1
        
        # Compute the square of each error term and adding them
        x1_mag = x1_e * x1_e
        y1_mag = y1_e * y1_e
        error1 = x1_mag + y1_mag
        
        if len(error_array) == 0:
            error_array.append(0)
            error_array.append(error1)
        else:
            error_array.append(error1)
        
        # Computing de/dt
        error_der =((error_array[len(error_array) -1] - error_array[len(error_array) - 2])/dt) * kd1
        
        # output = Kp*error + Kd*de/dt
        output1 = (kp1 * error1) + (error_der)
        output1 = np.clip(output1, -np.pi/6, np.pi/6)
        if sign1>0:
            output1 = -output1

        return output1
   
class SteeringController2:
    def update(self, p2, dt, kp2, kd2):
        radius1 = 20
        radius11 = radius1 * radius1
        x2 = p2[0]
        y2 = p2[1]
        
        # Compute the sign of the point
        x22 = x2 * x2
        y22 = y2 * y2
        sign2 = x22 + y22 - radius11
        theta2 = np.arctan2(y2,x2)
        x2_d = (np.cos(theta2) * radius1)
        y2_d = (np.sin(theta2) * radius1)
        
        # Compute the Error
        x2_e = x2_d - x2
        y2_e = y2_d - y2
        
        # Compute the square of each error term and adding them
        x2_mag = x2_e * x2_e
        y2_mag = y2_e * y2_e
        error2 = x2_mag + y2_mag
        if len(error_array2) == 0:
            error_array2.append(0)
            error_array2.append(error2)
        else:
            error_array2.append(error2)
            
        # Computing de/dt
        error_der2 =((error_array2[len(error_array2) -1] - error_array2[len(error_array2) - 2])/dt) * (kd2 )
        
        # output = Kp*error + Kd*de/dt
        output2 = (kp2 * error2) + (error_der2)
        output2 = np.clip(output2, -np.pi/6, np.pi/6)
        if sign2>0:
            output2 = -output2

        return output2
   
class SteeringController3:

    def update(self, p3, dt, kp3, kd3):
        radius1 = 20
        radius11 = radius1 * radius1
        x3 = p3[0]
        y3 = p3[1]
        
        # Compute the sign of the point
        x33 = x3 * x3
        y33 = y3 * y3
        sign3 = x33 + y33 - radius11
        theta3 = np.arctan2(y3,x3)
        x3_d = (np.cos(theta3) * radius1)
        y3_d = (np.sin(theta3) * radius1)
        
        # Compute the Error
        x3_e = x3_d - x3
        y3_e = y3_d - y3
        
        # Compute the square of each error term and adding them
        x3_mag = x3_e * x3_e
        y3_mag = y3_e * y3_e
        error3 = x3_mag + y3_mag
        if len(error_array3) == 0:
            error_array3.append(0)
            error_array3.append(error3)
        else:
            error_array3.append(error3)
        
        # Computing de/dt
        error_der3 =((error_array3[len(error_array3) -1] - error_array3[len(error_array3) - 2])/dt) * kd3 
        
        # output = Kp*error + Kd*de/dt
        output3 = (kp3 * error3) + (error_der3)
        output3 = np.clip(output3, -np.pi/6, np.pi/6)
        if sign3>0:
            output3 = -output3

        return output3
 

class Theta1_comp():
    def compute(self, p):
        x1 = p[0]
        y1 = p[1]
        theta1 = np.arctan2(y1,x1)
        theta1_array.append(theta1)
        if len(theta1_array)>1:
            aaa = np.unwrap(theta1_array, period = np.pi)
            theta1 = aaa[len(theta1_array) - 1]
        return theta1
   
class Theta2_comp():
    def compute(self, p2):
        x2 = p2[0]
        y2 = p2[1]
        theta2 = np.arctan2(y2,x2)
        if theta2<0:
            theta2 = 2*(np.pi) + theta2
            theta2_array.append(theta2)
        else:
            theta2_array.append(theta2)
        if len(theta2_array)>1:
            aaa2 = np.unwrap(theta2_array, period = np.pi)
            theta2 = aaa2[len(theta2_array) - 1]
        return theta2

class Theta3_comp():
    def compute(self, p3):
        x3 = p3[0]
        y3 = p3[1]
        theta3 = np.arctan2(y3,x3)
        if theta3<0:
            theta3 = 2*(np.pi) + theta3
            theta3_array.append(theta3)
        else:
            theta3_array.append(theta3)
        if len(theta3_array)>1:
            aaa3 = np.unwrap(theta3_array, period = np.pi)
            theta3 = aaa3[len(theta3_array) - 1]
        return theta3
 



def controlLoop():
    global KILL_THREAD

    #region Theta Class Initialization
    Theta1 = Theta1_comp()
    Theta2 = Theta2_comp()
    Theta3 = Theta3_comp()

    #endregion

    #region Controller initialization
    speedController  = SpeedController(kp=K_p, kd=K_d)
    speedController2 = SpeedController2(kp=K_p, kd=K_d)
    speedController3 = SpeedController3(kp=K_p, kd=K_d)

    steeringController  = SteeringController()
    steeringController2 = SteeringController2()
    steeringController3 = SteeringController3()


    t01 = time.perf_counter()    # Time ref point in ms
    t = 0
    i = 0
    while t<(tf + startDelay) and (not KILL_THREAD):

        u1 = hqcar1.set_velocity_and_request_state(0, 0, 0, 0, 0, 0, 0)
        p = np.array([u1[1][0], u1[1][1]])

        u2 = hqcar2.set_velocity_and_request_state(0, 0, 0, 0, 0, 0, 0)
        p2 = np.array([u2[1][0], u2[1][1]])

        u3 = hqcar3.set_velocity_and_request_state(0, 0, 0, 0, 0, 0, 0)
        p3 = np.array([u3[1][0], u3[1][1]])


        t0 = time.perf_counter()    # Time ref point in ms
        time_counter = t0 
        time_counter_array.append(time_counter)
        if len(time_counter_array)<2:
            dt = CYCLETIME
        else:
            dt = time_counter_array[len(time_counter_array)-1] - time_counter_array[len(time_counter_array) - 2]
      

        if time_counter < startDelay:
            u1 = 0
            delta = 0
            u2 = 0
            delta2 = 0
            u3 = 0
            delta3 = 0
        else:
            time_counter = t0
           
            theta1 = Theta1.compute(p)
            theta2 = Theta2.compute(p2)
            theta3 = Theta3.compute(p3)

            #theta2c = np.abs(theta2) - np.abs(theta1) - 2*(np.pi/5)
            #theta3c = np.abs(theta3) - np.abs(theta2) - 2*(np.pi/5)
            #theta4c = np.abs(theta4) - np.abs(theta3) - 2*(np.pi/5)

            #print(theta2c, theta3c, theta4c)

            delta = steeringController.update(p, dt, kp1 = 2, kd1 = 1)
            speed1 = speedController.update(theta1, theta2, dt, t)
            bb = speed1[0]
            u1 = hqcar1.set_velocity_and_request_state(bb, delta, 0, 0, 0, 0, 0)

            delta2 = steeringController2.update(p2, dt, kp2 = 2, kd2 = 1)
            speed2 = speedController2.update(theta1, theta2, theta3, dt, t)
            bb2 = speed2[0]
            u2 = hqcar2.set_velocity_and_request_state(bb2, delta2, 0, 0, 0, 0, 0)
            
            delta3 = steeringController3.update(p3, dt, kp3 = 2, kd3 = 1)
            speed3 = speedController3.update(theta2, theta3, dt, t)
            bb3 = speed3[0]
            u3 = hqcar3.set_velocity_and_request_state(bb3, delta3, 0, 0, 0, 0, 0)

            #print(bb,bb2,bb3,bb4,bb5, delta)

            S0.append(t)
            S1.append(theta1)
            S2.append(theta2)
            S3.append(theta3)

            V1.append(bb)
            V2.append(bb2)
            V3.append(bb3)

            now = time.perf_counter()
            elapsed_time = now - time_counter
            t= now - t01


            if elapsed_time < CYCLETIME:
                target_time =  CYCLETIME - elapsed_time
                time.sleep(target_time)
            i = i+1
            print(elapsed_time, CYCLETIME, t,i)
       
            time_counter += CYCLETIME

    filename1 = 'example.csv'
    with open(filename1, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(S0)
        csvwriter.writerow(S1)
        csvwriter.writerow(S2)
        csvwriter.writerow(S3)

        csvwriter.writerow(V1)
        csvwriter.writerow(V2)
        csvwriter.writerow(V3)




if __name__ == "__main__":

    os.system('cls')
    qlabs = QuanserInteractiveLabs()
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
    except:
        print("Unable to connect to QLabs")
        quit()

    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    R1 = 20
    rtModel1=default_rtModel

    initialPose=[R1*np.cos(0*2*(np.pi/3) - np.pi/5), R1*np.sin(0*2*(np.pi/3) - np.pi/5), 0.000]
    initialOrientation1=[0, 0, np.pi/2]
    hqcar1 = QLabsQCar(qlabs)
    hqcar1.spawn_id(actorNumber=10, location=[x for x in initialPose], rotation=initialOrientation1, waitForConfirmation=True)
    QLabsRealTime().start_real_time_model(rtModel1)
    gps1 = hqcar1.get_world_transform()
   
    initialPose2=[R1*np.cos(1*2*(np.pi/3) - np.pi/6), R1*np.sin(1*2*(np.pi/3) - np.pi/6), 0.000]
    initialOrientation2=[0, 0, np.pi]
    hqcar2 = QLabsQCar(qlabs)
    hqcar2.spawn_id(actorNumber=20, location=[x for x in initialPose2], rotation=initialOrientation2, waitForConfirmation=True)
    QLabsRealTime().start_real_time_model(rtModel1)
    gps2 = hqcar2.get_world_transform()
    
    initialPose3=[R1*np.cos(2*2*(np.pi/3) - np.pi/7), R1*np.sin(2*2*(np.pi/3) - np.pi/7), 0.000]
    initialOrientation3=[0, 0, np.pi]
    hqcar3 = QLabsQCar(qlabs)
    hqcar3.spawn_id(actorNumber=30, location=[x for x in initialPose3], rotation=initialOrientation3, waitForConfirmation=True)
    QLabsRealTime().start_real_time_model(rtModel1)
    gps3 = hqcar3.get_world_transform()

    spline1 = QLabsSplineLine(qlabs)
    spline1.spawn_id(actorNumber=70, location=[0, 0, 0])
    spline1.circle_from_center(radius=18, lineWidth=0.5, color = [0,0,0], numSplinePoints=50, waitForConfirmation=True)

    spline2 = QLabsSplineLine(qlabs)
    spline2.spawn_id(actorNumber=80, location=[0, 0, 0])
    spline2.circle_from_center(radius=22, lineWidth=0.5, color = [0,0,0], numSplinePoints=50, waitForConfirmation = True)

    spline3 = QLabsSplineLine(qlabs)
    spline3.spawn_id(actorNumber=90, location=[0, 0, 0])
    spline3.circle_from_center(radius=20, lineWidth=0.5, color = [1,0,0], numSplinePoints=50, waitForConfirmation = True)




    #endregion

    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()
    #endregion

    input('Experiment complete. Press any key to exit...')
