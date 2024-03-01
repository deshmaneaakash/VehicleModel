clear
clc

data = readmatrix('Data.csv');
vSoc = data(3:end, 1);
vOcv = data(3:end, 2);
% vOcv = ones(1, length(vSoc)) * 3;

time = linspace(1, 100, 100);
initialSoc = 0;
% current  = 0.5;

currentProfile_t = [0 10 11 21 22 32 33 43 44 54 55 65 66 76 77 87 88 100];
currentProfile_A = [0 0 1 1 0 0 1 1 0 0 1 1 0 0 1 1 0 0] * 0.05;
soc = [];
cellCapacity = 100; % Ah
vP = [0 0];

% Simulation
for t = 1:length(time)
    
    % Coulomb Counting
    current = interp1(currentProfile_t, currentProfile_A, t, "linear");
    if t == 1
        soc(t) = initialSoc;
    else
        soc(t) = soc(t-1) + t * current / cellCapacity;
    end

    ocv = interp1(vSoc, vOcv, soc(t), "linear");
    [vTerminal(t), vP] = rcModel(ocv, current, t, vP);

end

figure
hold on 
grid
plot(soc * 100, vTerminal, "LineWidth", 2)
plot(vSoc * 100, vOcv,"LineWidth", 2)
% plot(currentProfile_t, currentProfile_A * 100, '-k', "LineWidth", 2)
xlabel("SOC [%]")
ylabel("Terminal Voltage [V]")
title("Cell charging response")
