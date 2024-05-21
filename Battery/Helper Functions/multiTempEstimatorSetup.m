%% MULTI-TEMPERATURE IMPEDANCE ESTIMATION SETUP
% AAKASH DESHMANE
% 5/21/2024

clc
clear
close all

%% PREPROCESSING IMPEDANCE STRUCTURE

impedance = struct;
tempVector = [-20 -10 0 10 25];

% -20 C
impedance.tminus20C.temp = -20;
impedance.tminus20C.path = "C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\-20degC\5 pulse disch\06-15-17_11.31 n20degC_5Pulse_HPPC_Pan18650PF.mat";

% -10 C
impedance.tminus10C.temp = -10;
impedance.tminus10C.path = "C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\-10degC\5 pulse test\06-07-17_12.29 n10degC_5pulse_HPPC_Pan18650PF.mat";

% 0 C
impedance.t0C.temp = 0;
impedance.t0C.path = "C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\0degC\5 pulse\05-20-17_10.44 0degC_5pulse_HPPC_Pan18650PF.mat";

% 10 C
impedance.t10C.temp = 10;
impedance.t10C.path = "C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\10degC\5 pulse test\03-27-17_09.06 10degC_5Pulse_HPPC_Pan18650PF.mat";

% 25C
impedance.t25C.temp = 25;
impedance.t25C.path = "C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat";

%% ESTIMATION

for tempNum = 1:length(tempVector)
        
    temp = tempVector(tempNum);
    
    % Run constant temperature estimation

    estimatedParamsOutput = multiTempEstimatorFunction(path);
    











