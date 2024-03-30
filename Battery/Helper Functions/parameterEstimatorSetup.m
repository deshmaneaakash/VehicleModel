clear
clc
close all

addpath 'C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Battery'
load('C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_08.47 25degC_5Pulse_HPPC_Pan18650PF.mat')
% % load('"C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Data\Panasonic-18650PF-Data-master\Panasonic 18650PF Data\25degC\5 pulse disch\03-11-17_10.10 3390_dis5_10p.mat"')

%%

time = meas.Time;
[time, uniqueIndices, ~] = unique(time);

voltage = meas.Voltage;
voltage = voltage(uniqueIndices);

current = -meas.Current;
current = current(uniqueIndices);

loadCurrentIndices = find(current);
cell = batteryCell_PANA18650PF;
pulsePackets = getIndexPacket(cell, loadCurrentIndices, time, current, voltage);

%%
pulse = pulsePackets(1);
inputData = [pulse.time, pulse.current];
measuredTime = pulse.time;
measuredVoltage = pulse.voltage;
measuredData = timeseries(measuredVoltage,measuredTime);

soc = cell.soc;
ocv = cell.ocv.discharge;
initialSoc = 100;

% Initial parameters for the model
r0 = 0.01; % ohm
r1 = 0.003; % ohm
c1 = 60;   % F
r2 = 0.04;% ohm
c2 = 50;   %F

paramNames = ["c1", "c2", "r0", "r1", "r2"];
paramInitialValues = [c1 c2 r0 r1 r2];
params = initializeParams(paramNames, paramInitialValues);
estimatedParamsRaw = parameterEstimationRcModel(params, measuredData);
c1 = estimatedParamsRaw(1,1).Value;
c2 = estimatedParamsRaw(1,1).Value;
r0 = estimatedParamsRaw(1,1).Value;
r1 = estimatedParamsRaw(1,1).Value;
r2 = estimatedParamsRaw(1,1).Value;

%%
simObj = "rcModelParamsEst";
stopTime = max(pulse.time);
load_system(simObj)
set_param(simObj, "StartTime", num2str(0), "StopTime", num2str(stopTime))
out = sim(simObj);



%%
% 
figure
hold on
plot(out.simout.Time, out.simout.Data, "LineWidth", 2, "DisplayName", "Estimated Model")
plot(pulse.time, pulse.voltage, "LineWidth", 2, "DisplayName", "Test")
legend

%% Local Functions

function packets = getIndexPacket(cell, f, time, current, voltage)
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

            % Assign initial SOC by coulomb counting
            if packetNum == 1
                packets(packetNum).initialSoc = 100; % Assuming cell is fully charged at the start of the HPPC test
            else
                packets(packetNum).initialSoc = coulombCounting(packets(packetNum).time, packets(packetNum).current, cell.ratedCapacity, packets(packetNum-1).initialSoc);
            end


            indexPacket = [];
            packetNum = packetNum + 1;
        end
    end
end

function finalSoc = coulombCounting(timeData, currentData, cellCapacity, initialSoc)

    for t = 1:length(timeData)
        
        % Coulomb Counting
        current = currentData(t);

        if t == 1
            soc(t) = initialSoc;
        else
            dt = timeData(t) - timeData(t-1);
            soc(t) = soc(t-1) + dt * current * 100 / (cellCapacity * 3600);
        end
    
    end
    finalSoc = soc(end);
end

function params = initializeParams(paramNames, paramInitialValues)
    numParams = length(paramNames);
    for i = 1:numParams
        params(i) = param.Continuous(paramNames(i), paramInitialValues(i));
        params(i).Minimum = 0;       
    end
end