%% Thermal Controls of Hot and Cold Water

% Imagine a tank that has a hot water tap and a cold water tap inlets to
% it. The tank has a temperature sensor at its outlet. Hot and cold water
% mix and leave the tank through its outlet. Each inlet has a valve that
% controls the opening % of tap. 
%
% PROBLEM STATEMENT: 
% Develop a control strategy for the valves in such a way that the outlet water
% temperature sensor measures a steady state temperature.

clc
clear
close all

simuationTimeInMin = 7;
simulationTime = simuationTimeInMin * 60; % 7 minutes
time = 1:1:simulationTime;
dt = time(2) - time(1);

% System Model
pressureHead = 30; % kg/m^2
densityOfWater = 1000; % kg/m^3
maxAreaOfValve = 5e-4; 
massOfAccumulator = 2; % kg
velocityAtValve = sqrt(2*(pressureHead) / densityOfWater);

mDot.setpoint = 0.1;

T.hotWater = 60; % deg C
T.coldWater = 10; % deg C
T.request = 44; % deg C
T.init = 20;
T.setpoint = [15 15 55 55];
T.setpointTimeProfile = [1 60 60*5 simulationTime];


% Valve Opening Profile
valveProfile_t = [1 (simulationTime/2 - 1) (simulationTime/2) simulationTime];
hotValveProfile = [0 0 1 1];
coldValveProfile = [0 0 0.2 0.2];

% Controller Parameters

kp = 1;
ki = 0.2;
kd = 0.01;


%% Simulation 1
% Open Loop Simulation 

% dt = time(2) - time(1);
% 
% for t = 1:length(time)
%     if t == 1
%         T.water(t) = T.init;
%     else
% 
%         % Opening Area Percent
%         hotValveOpeningPercent = interp1(valveProfile_t, hotValveProfile, t, "linear", "extrap");
%         coldValveOpeningPercent = interp1(valveProfile_t, coldValveProfile, t, "linear", "extrap");
% 
%         % Opening Area
%         hotValveOpenArea = maxAreaOfValve * hotValveOpeningPercent;
%         coldValveOpenArea = maxAreaOfValve * coldValveOpeningPercent;
% 
%         % Mass flow rate
%         mDot.hotWater = densityOfWater * hotValveOpenArea * velocityAtValve;
%         mDot.coldWater = densityOfWater * coldValveOpenArea * velocityAtValve;
% 
%         temp = getWaterTemp(t, dt, massOfAccumulator, mDot, T);
%         T.water(t) = temp;
%     end
% 
% 
% end

%% Results of Open Loop Simulation

% figure
% plot(time/3600, T.water, "LineWidth", 2)
% xlabel("Time [hrs]")
% ylabel("Temperature [deg C]")
% title("Open Loop System : Accumulator Water Temperature")

%% Simulation 2 
% Controller

% Saturation Constraint on Valve opening for constant output
% mass flow rate
% maxAreaOfValve = mDot.setpoint / (densityOfWater * velocityAtValve);
maxValveOpeningPercent = (mDot.setpoint / (densityOfWater * velocityAtValve * maxAreaOfValve));
minValveOpeningPercent = 1 - maxValveOpeningPercent;

for t = 1:length(time)
    if t == 1
        T.water(t) = T.init;
        mDot.total(t) = 0;
        prevError = 0;
        prevITerm = 0;

    else
        
        currentTemp = T.water(t-1);
        tSetpoint = interp1(T.setpointTimeProfile, T.setpoint, t, "linear", "extrap");

        % Opening Area Percent
        [hotValveOpeningPercent, iTerm, error, pTerm, dTerm] = pid(dt, tSetpoint, currentTemp, kp, ki, kd, prevError, prevITerm);
        
        % Saturation Constraint on Valve opening for constant output
        % mass flow rate
        hotValveOpeningPercent = min(maxValveOpeningPercent, hotValveOpeningPercent);
        hotValveOpeningPercent = max(0, hotValveOpeningPercent);

        coldValveOpeningPercent = maxValveOpeningPercent - hotValveOpeningPercent;
        coldValveOpeningPercent = min(maxValveOpeningPercent, coldValveOpeningPercent);
        coldValveOpeningPercent = max(0, coldValveOpeningPercent);

        % Opening Area
        hotValveOpenArea = maxAreaOfValve * hotValveOpeningPercent;
        coldValveOpenArea = maxAreaOfValve * coldValveOpeningPercent;
        
        % Mass flow rate
        mDot.hotWater(t) = densityOfWater * hotValveOpenArea * velocityAtValve;
        mDot.coldWater(t) = densityOfWater * coldValveOpenArea * velocityAtValve;
        mDot.total(t) = mDot.hotWater(t) + mDot.coldWater(t); 

        temp = getWaterTemp(t, dt, massOfAccumulator, mDot, T);
        T.water(t) = temp;
      
    end

end

% Low pass filter on mass flow rates
mDot.hotWater = lowpass(mDot.hotWater, 0.5);
mDot.coldWater = lowpass(mDot.coldWater, 0.5);

%% Results of Feedback controller

figure
hold on
plot(time/60, T.water, "LineWidth", 2, "DisplayName", "Outlet Temperature")
plot(T.setpointTimeProfile/60, T.setpoint, '--k', "LineWidth", 2, "DisplayName", "Temperature Setpoint")
xlabel("Time [mins]")
ylabel("Temperature [deg C]")
title("Close loop controlled system : Accumulator Water Temperature")
legend

figure
hold on
plot(time/60, mDot.total, "LineWidth", 2, "DisplayName", "Total mass flow rate")
plot(time/60, mDot.hotWater, '-r', "LineWidth", 1, "DisplayName", "Hot Water mass flow rate")
plot(time/60, mDot.coldWater, '-b', "LineWidth", 1, "DisplayName", "Cold Water mass flow rate")
xlabel("Time [mins]")
ylabel("Mass flow rate [kg/s]")
title("Close loop controlled system : Mass Flow rate")
legend


%% Local Functions

function temp = getWaterTemp(t, dt, massOfAccumulator, mDot, T)
    
    term1 = (massOfAccumulator + mDot.hotWater(t)*dt + mDot.coldWater(t)*dt)^-1;
    term2 = (mDot.hotWater(t)*dt*T.hotWater + mDot.coldWater(t)*dt*T.coldWater + massOfAccumulator*T.water(t-1));
    temp = term1 * term2;

end


function [output, iTerm, error, pTerm, dTerm] = pid(dt, reference, state, kp, ki, kd, prevError, prevITerm)

    error = reference - state;
    pTerm = kp * error;
    iTerm = ki * error + prevITerm;
    dTerm = kd * (error - prevError) / dt;
    output = pTerm + iTerm + dTerm;
end

