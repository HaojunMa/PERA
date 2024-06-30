

function [u1, u2, tau, vKp, vKd]=PERA_Wrist_Controller(Theta1, Theta2, dTheta1, dTheta2, iTheta1, iTheta2, eTheta1, eTheta2, deTheta1, deTheta2, ddeTheta1, ddeTheta2, ieTheta1, ieTheta2, t)

    % SENDING: 
    % Last position: Theta1(i) and Theta2(i)
    % Last speed: dTheta1(i) and dTheta2(i)
    % Desired position: eTheta1(i) and eTheta2(i)[SampTimeArray, Encoder, Theta, dTheta, eTheta, deTheta, Time, ControlSig, Tau, V, Beta, F, Fd, VKp, VKd, D, FF]=PERA_Shoulder_last(PeriodT1, AmpT1, OffT1, BiasT1, PeriodT2, AmpT2, OffT2, BiasT2, ExecTime);
    % First integral over desired position: deTheta1(i) and deTheta2(i)
    % Second integral over desired position: ddeTheta1(i) and ddeTheta2(i)  
    % RECEIVING:
    % Control Law for the motors: u1 and u2 (secured, it means, within the
    % boundaries to protect the motors)
    % Control Law in terms of the Torque (tau1 and tau2) (just for analsys)
    % Gravity vector that is being compensated
    % All parts of tau
