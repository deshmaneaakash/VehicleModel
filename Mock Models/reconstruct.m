% Thermal Modeling of a block

clear
clc

time = 1:0.5:10e4;
m = 1; % kg
current = 2; % A
r = 20; % ohms
Ra = 10;
cPBlock = 900; % J/KgC

% Initial Conditions
Tequi = 27; % deg C
TblockInitial = Tequi;
Tambient = Tequi;

% Simulation

for t = 1:length(time)
    if t == 1
        T(t) = Tequi;
    else 
        dt = time(t) - time(t-1);
        T(t) = ((current^2)*r*dt*Ra + m*cPBlock*Ra*T(t-1) + Tambient*dt) / (m*cPBlock*Ra + dt);
    end

end

figure
plot(time, T)
xlabel('Time [s]')
ylabel('T1 [C]')
title('Temperature of block vs time')

