
import numpy as np

def wrist_controller(parameters, process, setpoint):

    # remember some older values HOE?
    global wrist_out, wrist_set
    wrist_out  = np.zeros(4) 
    wrist_set  = np.zeros(4) 



    # PID controller parameters:
    Kp, Ki, Kd = parameters

    # Vectors Theta, dTheta and eTheta
    Theta   = np.array([Theta1, Theta2])
    dTheta  = np.array([dTheta1, dTheta2])
    iTheta  = np.array([iTheta1, iTheta2])
    eTheta  = np.array([eTheta1, eTheta2])
    deTheta = np.array([deTheta1, deTheta2])
    ieTheta  = np.array([ieTheta1, ieTheta2])

    vKp = - Kp.dot(Theta  - eTheta);
    vKd = - Kd.dot(dTheta - deTheta);
    vKi = - Ki.dot(iTheta - ieTheta);

    # no gravity compensation when moving down. 

    tau = vKp + vKd + vKi

    # Splitting up the matrix tau in two vectors tau1 and tau2

    tau1 = tau[0]
    tau2 = tau[1]

    # Compute the tau that has to go to the motors
    # The minus sign is needed due to the positive definition of u1 and u2
    taumotor1 = (tau1+tau2)/2;
    taumotor2 = (tau1-tau2)/2;

    #Motor specifications
    kM  = 21.2  # 53.8    Shoulder    23.2    Elbow   21.2 Wrist Torque Constant [nMn/A]
    G   = 29    # 66      Shoulder    33      Elbow  29    Wrist Gear Ratio
    eff = 0.70  # 0.70    Shoulder    0.75    Elbow  0.70  Wrist Max Eff of the Gear [%]

    # The calculated torque needs to be compensated for the efficiency of the
    # gearing
    torquem1=taumotor1/(G*eff)
    torquem2=taumotor2/(G*eff)

    # The current in mA is related to the number of counts in the output
    # I(A)=torque(mNm)/km(mNm/A). We need I(mA) and have torque(Nm), so
    # I(mA)=torque(Nm)*1000*1000/km(mNm/A)
    i1=torquem1*1000*1000/kM/4.3
    i2=torquem2*1000*1000/kM/4.3

    # Getting the sign of i, which will be used for the sign of u
    s1=np.sign(i1)
    s2=np.sign(i2)

    # Compensation needs the absolute value of i
    i1 = abs(i1)
    i2 = abs(i2)

    # Polynomic compensation for non-linear Amplifier Behavior
    u1 = -0.01159*i1**2+36.96*i1+1593
    u2 = -0.01159*i2**2+36.96*i2+1593

    # Putting back the real sign
    u1=u1*s1
    u2=u2*s2

    # Define a limit for the reference signal to protect the motors
    Final1=16000
    Final2=16000

    if u1>Final1:
        u1=Final1
    elif u1<-Final1:
        u1=-Final1

    if u2>Final2:
        u2=Final2
    elif u2<-Final2:
        u2=-Final2

        
    return  u1, u2, tau, vKp, vKd

