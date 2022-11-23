import numpy as np
from PID import leadLag, PID_RT, actionP, actionI, actionD, IMCTuning


e1 = [] #"Ceci doit etre = à incrementation roue :: +1 chaque tour"
e2 = [] #"Same"

#constant for the target of encoder ticks you want the motors to achieve;
#  make this value about 75% of the ‘encoder ticks per sample’ value you made a note of earlier (in our case 60 × 0.75 = 45):
Target = 45
Target = []

#Kp : good starting point is 1 divided by the ‘encoder ticks per sample’ (e.g. 1 / 60 = 0.0166~)
#Kd : a good starting value is half the value of KP:
#Ki : a good starting point is half the value of KD:
Kp = 0.02
Kd = 0.01
Ki = 0.005

e1_error = []
e2_error = []

m1_speed = [] #"vitesse qu'on donne au moteur 1 ==> pin pwm je pense == "
m2_speed = [] #"same"

MVP_M1 = []
MVI_M1 = []
MVD_M1 = []

MVP_M2 = []
MVI_M2 = []
MVD_M2 = []

alpha = 0.6
MVMin = 0
MVMax = 255
Ts = 1

MVFFLL1 = []
#Parameters for input-output dynamics
Kp2 = 0.63
T1p = 146.0
T2p = 1.0
thetap = 1.0

#Parameters for disturbance dynamics
Kd2 = 0.41
T1d = 153.0
T2d = 16.0
thetad = 10.0
#working point
MV0 = 0
DV0 = 0
PV0 = 0
DV = []
MVFF = []
ActivateFF = True

while True:    
    
    e1.append("Communication_avec_arduino")
    e2.append("same")

    #Feedforward
    leadLag(m1_speed, -Kd2/Kp2, T1p,T1d, Ts, MVFFLL1)
    if ActivateFF:
        leadLag(MVFFLL1, 1, T2p, T2d, Ts, MVFF)
    else: 
        leadLag(MVFFLL1, 0, T2p, T2d, Ts, MVFF)

    PID_RT(Target, e1.value, False, None, None, Kp, Ki, Kd, alpha, Ts, MVMin, MVMax, m1_speed, MVP_M1, MVI_M1, MVD_M1, e1_error, False)
    PID_RT(Target, e2.value, False, None, None, Kp, Ki, Kd, alpha, Ts, MVMin, MVMax, m2_speed, MVP_M2, MVI_M2, MVD_M2, e2_error, False)

    #PID has appended my vector lets use it
    arduino_Com_m1_speed = m1_speed[-1]
    arduino_Com_m2_speed = m2_speed[-1]
    
    #sleep(SAMPLETIME)
