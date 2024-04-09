%% CHARGE CONTROLS 

clc
clear
close all

%% Initialization

cell = batteryCell_PANA18650PF;

time = 1:1:10e3;
timeLimit = max(time);
chargePower = 5; % W
chargerVoltage = 4.5; % V
current = cell.maxCapacity/2;
% currentProfile = [0 0 current current 0 0];
% timeProfile = [1 (timeLimit/4 - 1)  (timeLimit/4) (3*timeLimit/4) (3*timeLimit/4  + 1) timeLimit];
constantVoltage = cell.maxV;
isCVModeOn = false;

%% Simulation

% simObj = "chargeControlsModel";
% stopTime = max(time);
% load_system(simObj)
% set_param(simObj, "StartTime", num2str(0), "StopTime", num2str(stopTime))
% out = sim(simObj);

% Controller
kp = 5;
ki = 0.01;
kd = 5;

soc = 0;
dt = time(2) - time(1);
vP = [0 0];
for t = 1:length(time)
    if t == 1
        soc(t) = cell.soc(1);
        prevError = 0;
        prevITerm = 0;
        error = 0;

    else 
        % current = interp1(timeProfile, currentProfile, t, "linear");
        soc(t) = soc(t-1) + dt * current * 100 / (cell.maxCapacity * 3600);
    end

    ocv = interp1(cell.soc, cell.ocv.charge, soc(t), "linear", "extrap");
    % ocv = 3;

    [ccv, vP]  = rcModel(ocv, current, dt, vP);
    vT(t) = ccv;
    vOcv(t) = ocv;

    if vT(t) > cell.maxV
    % Constant Voltage based current controls
        isCVModeOn = true;
    end

    % Constant Voltage Mode
    if isCVModeOn
    [current, iTerm, error, pTerm, dTerm] = pid(dt, cell.maxV, vT(t), kp, ki, kd, prevError, prevITerm);
    current = max(0, current);
    end
    currentVector(t) = current;
    errorVector(t) = error;

    if soc(t) > 99
        time = time(1:t);
        break
    end
end




%% Plotting

figure
hold on 
plot(time/60, vT, '-r', "LineWidth", 2, "DisplayName", "Cell Voltage")
plot(time/60, vOcv, '--r', "LineWidth", 2, "DisplayName", "Cell OCV")
xlabel("Time [mins]")
ylabel("Voltage [V]")
yyaxis right
plot(time/60, currentVector, '-b', "LineWidth", 2, "DisplayName", "Current")
title("Cell Voltage and current reponse")
legend

figure
plot(time/60, errorVector)
% figure
% plot(time/60, soc, '--k', "LineWidth", 2, "DisplayName", "SOC")
% xlabel("Time [mins]")
% ylabel("SOC [%]")
% title("SoC vs Time")


function [output, iTerm, error, pTerm, dTerm] = pid(dt, reference, state, kp, ki, kd, prevError, prevITerm)

    error = reference - state;
    pTerm = kp * error;
    iTerm = ki * error + prevITerm;
    dTerm = kd * (error - prevError) / dt;
    output = pTerm + iTerm + dTerm;
end
