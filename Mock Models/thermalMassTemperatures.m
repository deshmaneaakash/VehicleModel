% Thermal Modeling of a block

clear
% close all
clc

time = 1:0.5:10e4;
mass = 1; % kg
current = 5; % A
r = 10; % ohms
cPBlock = 900; % J/KgC
cPWater = 4200; % J/KgC
cPAir = 1000; % J/KgC
mDot = 0.1; % kg/s

RFluidToBlock = 1;
RBlockToRoom = 5;
RRoomToAmbient = 1;

roomMass = 0.1 * 0.1 * 0.1 * 1.225; % volume * density

% Initial Conditions
Tequi = 27; % deg C
t1 = Tequi;
t2 = Tequi;
TblockInitial = Tequi;
TroomInitial = Tequi;
TambientInitial = Tequi;

% Controller
kp = 0.02;
ki = 0.000;
kd = 0;
massTempReference = 1100;

% Simulation
tStates = zeros(length(time), 4);

for t = 1:length(time)
    if t == 1
        tStates(t, :) = Tequi;
        prevError = 0;
        prevITerm = 0;

    else 
        dt = time(t) - time(t-1);
        Qgen = current^2 * r * dt;
        
        % Fluid at point 1
        % Energy gained
        dTFluid = Qgen / (mDot * dt * cPWater);
        tStates(t, 1) = tStates(t-1, 2) + dTFluid;

        % Fluid at point 2
        % Energy lost
        QlostToMass = (tStates(t, 1) - tStates(t-1, 3)) * dt / RFluidToBlock;
        dTFluidDrop = QlostToMass / (mDot * dt * cPWater);
        tStates(t, 2) = tStates(t, 1) - dTFluidDrop;

        % Mass 

        % Energy gained
        QgainedByMass = QlostToMass;
        dTMass = QgainedByMass / (mass * cPBlock);
        tStates(t, 3) = tStates(t-1, 3) + dTMass;
        
        % Energy lost
        QlostToRoom = (tStates(t, 3) - tStates(t-1, 4)) * dt / RBlockToRoom;
        dTMassDrop = QlostToRoom / (mass * cPBlock);
        tStates(t, 3) = tStates(t, 3) - dTMassDrop;

        % Room

        % Energy gained
        QgainedByRoom = QlostToRoom;
        dTRoom = QgainedByRoom / (roomMass * cPAir);
        tStates(t, 4) = tStates(t-1, 4) + dTRoom;

        % Energy lost
        QlostToAmbient = (tStates(t, 4) - TambientInitial) * dt / RRoomToAmbient;
        dtRoomDrop = QlostToAmbient / (roomMass * cPAir);
        tStates(t, 4) = tStates(t, 4) - dtRoomDrop;

        massTemp = tStates(t,3);
        [current, prevITerm, prevError, pTerm, dTerm] = pid(dt, massTempReference, massTemp, kp, ki, kd, prevError, prevITerm);

    end

end

%% 

figure(1)
hold on
% plot(time, tStates(:, 1), 'DisplayName', 'T1', 'LineWidth', 2)
% plot(time, tStates(:, 2), 'DisplayName', 'T2', "LineWidth", 2, "LineStyle", "-")
plot(time, tStates(:, 3), 'DisplayName', 'Block Temerature', 'LineWidth', 2)
% plot(time, tStates(:, 4),'DisplayName', 'Room Temerature', 'LineWidth', 2)
xlabel('Time [s]')
ylabel('T [C]')
title('Temperature vs time')
legend

function [output, iTerm, error, pTerm, dTerm] = pid(dt, reference, state, kp, ki, kd, prevError, prevITerm)

    error = reference - state;
    pTerm = kp * error;
    iTerm = ki * error + prevITerm;
    dTerm = kd * (error - prevError) / dt;
    output = pTerm + iTerm + dTerm;
end

