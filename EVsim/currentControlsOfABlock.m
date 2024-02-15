%% CURRENT CONTROLS FROM THERMAL MODELING
% This script demonstrates a PID regulator on a current controller through
% a block of copper. 

clear
clc

lengthOfBlock  = 0.1; % m
breadthOfBlock = 0.03; % m
widthOfBlock   = 0.05; % m
volumeOfBlock = lengthOfBlock * breadthOfBlock * widthOfBlock; % m3
densityOfBlock = 2700; % kg/m3
massOfBlock = densityOfBlock * volumeOfBlock; % kg
dt = 1; % sec
resistivityOfCopper = 2.82e-8; % ohm-m
rBlock = resistivityOfCopper * lengthOfBlock / (breadthOfBlock * widthOfBlock);
cP = 902; % Specific heat capacity of copper in J/kgC
tAmbient = 27; % deg C
initialTemp = 27; % deg C
rToAmbient = 10; % C/W
% current = 1000; % A

simulationTimeLimit = 3600 * 5; % sec
simulationTime = 1:simulationTimeLimit;

% Controls
referenceTempVector = [35 35 40 40 35 35];
referenceTempTime = [1 1200 1201 2400 2401 3600] * 5;
kp = 300;
ki = 0.065;
kd = 0;

% Simulation
for time = simulationTime
    if time == 1
        currentTemp = initialTemp;
        prevITerm = 0;
        prevError = 0;
    else
        currentTemp = Tt(time - 1);
    end

    referenceTemp = interp1(referenceTempTime, referenceTempVector, time, "linear", "extrap");
    
    [current, prevITerm, prevError, pTerm, dTerm] = pid(dt, referenceTemp, currentTemp, kp, ki, kd, prevError, prevITerm);
    Tt(time) = ((current^2) * rBlock * rToAmbient + massOfBlock*cP*rToAmbient*currentTemp + tAmbient) / ...
                    ( 1 + massOfBlock*cP*rToAmbient);
    currentVector(time) = current;
    errorVector(time) = prevError;
    pTermVector(time) = pTerm;
    iTermVector(time) = prevITerm;
    dTermVector(time) = dTerm;
    referenceTemps(time) = referenceTemp;

end

figure(1)
hold on
grid on
plot(simulationTime / 3600, Tt,"LineWidth", 2)
plot(simulationTime / 3600, referenceTemps,"LineWidth", 2, "Color", "black")
% yline(40)
xlabel('Time [hrs]')
ylabel('Temperature of the block')
title('Temperature response')

% figure
% grid on
% plot(simulationTime / 3600, currentVector,"LineWidth", 2)
% xlabel('Time [hrs]')
% ylabel('Current of the block')
% title('Current response')

% figure
% grid on
% hold on
% plot(simulationTime / 3600, errorVector,"LineWidth", 2, "DisplayName", 'Error')
% plot(simulationTime / 3600, pTermVector,"LineWidth", 2, "DisplayName", 'P term')
% plot(simulationTime / 3600, iTermVector,"LineWidth", 2, "DisplayName", 'I term')
% xlabel('Time [hrs]')
% ylabel('Error')
% title('Error response')
% legend

%% LOCAL FUNCTIONS

function [output, iTerm, error, pTerm, dTerm] = pid(dt, reference, state, kp, ki, kd, prevError, prevITerm)

    error = reference - state;
    pTerm = kp * error;
    iTerm = ki * error + prevITerm;
    dTerm = kd * (error - prevError) / dt;
    output = pTerm + iTerm + dTerm;
end



