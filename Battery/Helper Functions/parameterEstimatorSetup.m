clear
clc
close all
tic

addpath 'C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Battery'
load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')
% % load('"C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_10.10 3390_dis5_10p.mat"')

%%
cell = batteryCell_PANA18650PF;

time = meas.Time;
[time, uniqueIndices, ~] = unique(time);

voltage = meas.Voltage;
voltage = voltage(uniqueIndices);

current = -meas.Current;
current = current(uniqueIndices);
Ah = meas.Ah(uniqueIndices);
socFromAh = 100 + Ah * 100 / cell.maxCapacity;



loadCurrentIndices = find(current);
pulsePackets = getIndexPacket(cell, loadCurrentIndices, time, current, voltage);
numPulses = size(pulsePackets);
numPulses = numPulses(2);
%%

% Initial parameters for the model
initialParams.soc = 100;
initialParams.r0 = 0.001;
initialParams.r1 = 0.0005;
initialParams.c1 = 0.3;
initialParams.r2 = 0.05;
initialParams.c2 = 4;

for pulseNum = 1:numPulses

    pulse = pulsePackets(pulseNum);

    inputData = [pulse.time, pulse.current];
    measuredTime = pulse.time;
    measuredVoltage = pulse.voltage;
    measuredData = timeseries(measuredVoltage,measuredTime);
    
    soc = cell.soc;
    ocv = cell.ocv.discharge;
    initialOCV = measuredVoltage(1);
    
    % Initial parameters for the model
    r0 = initialParams.r0;  % ohm
    r1 = initialParams.r1;  % ohm
    c1 = initialParams.c1;  % F
    r2 = initialParams.r2;  % ohm
    c2 = initialParams.c2;  %F

    paramNames = ["c1", "c2", "r0", "r1", "r2"];
    paramInitialValues = [initialParams.c1 initialParams.c2 initialParams.r0 initialParams.r1 initialParams.r2];
    params = initializeParams(paramNames, paramInitialValues);
    estimatedParamsRaw = parameterEstimationRcModel(params, measuredData);
    estimatedParams.c1 = estimatedParamsRaw(1,1).Value;
    estimatedParams.c2 = estimatedParamsRaw(1,2).Value;
    estimatedParams.r0 = estimatedParamsRaw(1,3).Value;
    estimatedParams.r1 = estimatedParamsRaw(1,4).Value;
    estimatedParams.r2 = estimatedParamsRaw(1,5).Value;

    r0 = estimatedParams.r0;  % ohm
    r1 = estimatedParams.r1; % ohm
    c1 = estimatedParams.c1;    % F
    r2 = estimatedParams.r2;   % ohm
    c2 = estimatedParams.c2;      %F

    pulsePackets(pulseNum).r0 = estimatedParams.r0;
    pulsePackets(pulseNum).r1 = estimatedParams.r1;
    pulsePackets(pulseNum).c1 = estimatedParams.c1;
    pulsePackets(pulseNum).r2 = estimatedParams.r2;
    pulsePackets(pulseNum).c2 = estimatedParams.c2;

    initialParams = estimatedParams;

    disp(['DONE WITH PACKET NUMBER = ' num2str(pulseNum) 'OUT OF ' num2str(numPulses)])

end
toc
%%
% 
for pulsecheck = 1:66
    pulse = pulsePackets(pulsecheck);
    r0 = pulse.r0;  % ohm
    r1 = pulse.r1; % ohm
    c1 = pulse.c1;    % F
    r2 = pulse.r2;   % ohm
    c2 = pulse.c2;

    inputData = [pulse.time, pulse.current];
    % measuredTime = pulse.time;
    % measuredVoltage = pulse.voltage;
    % measuredData = timeseries(measuredVoltage,measuredTime);
    initialOCV = pulse.voltage(1);
    out = runSim(pulse.time);

    figure
    hold on
    plot(out.simout.Time, out.simout.Data, "LineWidth", 2, "DisplayName", "Estimated Model")
    plot(pulse.time, pulse.voltage, "LineWidth", 2, "DisplayName", "Test")
    legend
end

%%

% figure;plot(flip([pulsePackets.r0]), "DisplayName", "R0");legend
% figure;plot(flip([pulsePackets.r1]), "DisplayName", "R1");legend
% figure;plot(flip([pulsePackets.r2]), "DisplayName", "R2");legend
% figure;plot(flip([pulsePackets.c2]), "DisplayName", "C2");legend
% figure;plot(flip([pulsePackets.c1]), "DisplayName", "C1");legend

ocv = cell.ocv.discharge;
[ocv, uniqueIndices, ~] = unique(ocv);
socs = cell.soc(uniqueIndices);

for i=1:66
    socVector(i) = interp1(ocv, socs, pulsePackets(i).voltage(1), "linear", 100);
    r0Vector(i) = flip([pulsePackets(i).r0]);
    r1Vector(i) = flip([pulsePackets(i).r1]);
    r2Vector(i) = flip([pulsePackets(i).r2]);
    c1Vector(i) = flip([pulsePackets(i).c1]);
    c2Vector(i) = flip([pulsePackets(i).c2]);
end

socVector = (flip(socVector));

%% Local Functions

function packets = getIndexPacket(f, time, current, voltage)
    packets = struct;
    packetNum = 1;
    indexPacket = [];
    for index = 2:length(f)
        if f(index) - f(index - 1) == 1
            indexPacket = [indexPacket f(index - 1)];
        else
            preBufferIndices = 1:2;
            postBufferIndices = 1:50;
            preBuffer = flip(indexPacket(1) - preBufferIndices);
            postBuffer = indexPacket(end) + postBufferIndices;
            indexPacket = [preBuffer indexPacket postBuffer];

            packets(packetNum).indices = indexPacket;
            packets(packetNum).time = time(indexPacket) - time(indexPacket(1));
            packets(packetNum).current = current(indexPacket);
            packets(packetNum).voltage = voltage(indexPacket);

            indexPacket = [];
            packetNum = packetNum + 1;
        end
    end
end


function out = runSim(time)
    simObj = "rcModelParamsEst";
    stopTime = max(time);
    load_system(simObj)
    set_param(simObj, "StartTime", num2str(0), "StopTime", num2str(stopTime))
    out = sim(simObj);
end

% function estimatedParams = paramsOfPulse(cell, initialParams, pulse)
% 
%     inputData = [pulse.time, pulse.current];
%     measuredTime = pulse.time;
%     measuredVoltage = pulse.voltage;
%     measuredData = timeseries(measuredVoltage,measuredTime);
% 
%     soc = cell.soc;
%     ocv = cell.ocv.discharge;
%     initialSoc = initialParams.soc;
% 
%     % Initial parameters for the model
%     r0 = initialParams.r0;  % ohm
%     r1 = initialParams.r1;  % ohm
%     c1 = initialParams.c1;  % F
%     r2 = initialParams.r2;  % ohm
%     c2 = initialParams.c2;  %F
% 
%     paramNames = ["c1", "c2", "r0", "r1", "r2"];
%     paramInitialValues = [initialParams.c1 initialParams.c2 initialParams.r0 initialParams.r1 initialParams.r2];
%     params = initializeParams(paramNames, paramInitialValues);
%     estimatedParamsRaw = parameterEstimationRcModel(initialParams, inputData, params, measuredData);
%     estimatedParams.c1 = estimatedParamsRaw(1,1).Value;
%     estimatedParams.c2 = estimatedParamsRaw(1,2).Value;
%     estimatedParams.r0 = estimatedParamsRaw(1,3).Value;
%     estimatedParams.r1 = estimatedParamsRaw(1,4).Value;
%     estimatedParams.r2 = estimatedParamsRaw(1,5).Value;
% 
%     r0 = estimatedParams.r0;  % ohm
%     r1 = estimatedParams.r1; % ohm
%     c1 = estimatedParams.c1;    % F
%     r2 = estimatedParams.r2;   % ohm
%     c2 = estimatedParams.c2;      %F
% 
%     out = runSim(pulse.time);
%     estimatedParams.soc = out.soc.Data(end);
% 
% end

% function estimatedParams = estimateParams(inputData, initialParams, measuredData)
% 
%     % Initial parameters for the model
%     r0 = initialParams.r0;  % ohm
%     r1 = initialParams.r1;  % ohm
%     c1 = initialParams.c1;  % F
%     r2 = initialParams.r2;  % ohm
%     c2 = initialParams.c2;  %F
% 
%     paramNames = ["c1", "c2", "r0", "r1", "r2"];
%     paramInitialValues = [initialParams.c1 initialParams.c2 initialParams.r0 initialParams.r1 initialParams.r2];
%     params = initializeParams(paramNames, paramInitialValues);
%     estimatedParamsRaw = parameterEstimationRcModel(initialParams, inputData, params, measuredData);
%     estimatedParams.c1 = estimatedParamsRaw(1,1).Value;
%     estimatedParams.c2 = estimatedParamsRaw(1,2).Value;
%     estimatedParams.r0 = estimatedParamsRaw(1,3).Value;
%     estimatedParams.r1 = estimatedParamsRaw(1,4).Value;
%     estimatedParams.r2 = estimatedParamsRaw(1,5).Value;
% end

function params = initializeParams(paramNames, paramInitialValues)
    numParams = length(paramNames);
    for i = 1:numParams
        params(i) = param.Continuous(paramNames(i), paramInitialValues(i));
        params(i).Minimum = 0;     
        params(i).Scale = 0.25;
    end
end