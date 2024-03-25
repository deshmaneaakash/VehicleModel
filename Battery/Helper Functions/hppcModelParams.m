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

current = meas.Current;
current = current(uniqueIndices);

loadCurrentIndices = find(current);
cell = batteryCell_PANA18650PF;
pulsePackets = getIndexPacket(cell, loadCurrentIndices, time, current, voltage);
Ah = ah(time, current);
Ah_raw = ah(meas.Time, meas.Current);
measTime = meas.Time;
Ah_integrate = trapz(current) / 3600;
%% 
% for i = 1:66
%     figure
%     hold on
%     plot(pulsePackets(i).time, pulsePackets(i).voltage)
%     yyaxis right
%     plot(pulsePackets(i).time, pulsePackets(i).current)
%     hold off
% 
% end


%%
pulse = pulsePackets(1);
inputData = [pulse.time, pulse.current];
measuredTime = pulse.time;
measuredVoltage = pulse.voltage;

soc = cell.soc;
ocv = cell.ocv.discharge;
initialSoc = 100;

% Initial parameters for the model
r0 = 0.2; % ohm
r1 = 0.1; % ohm
c1 = 1e4;   % F
r2 = 0.3;% ohm
c2 = 2e5;   %F

pulse.initialParams.r0 = r0;
pulse.initialParams.r1 = r1;
pulse.initialParams.c1 = c1;
pulse.initialParams.r2 = r2;
pulse.initialParams.c2 = c2;

% Solve Non Linear Least Squares Optimization
rcParams = rcParamsSolver(cell, pulse);

r0 = rcParams(1);
r1 = rcParams(2);
c1 = rcParams(3);
r2 = rcParams(4);
c2 = rcParams(5);


simObj = "rcModelSimulink";
stopTime = max(pulse.time);
load_system(simObj)
set_param(simObj, "StartTime", num2str(0), "StopTime", num2str(stopTime))
out = sim(simObj);

%%
% 
figure
hold on
plot(out.ccv.Time, out.ccv.Data, "LineWidth", 2, "DisplayName", "Model")
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

function Ah = ah(timeData, currentData)

    for t = 1:length(timeData)
        
        % Coulomb Counting
        current = currentData(t);

        if t == 1
            Ah(t) = 0;
        else
            dt = timeData(t) - timeData(t-1);
            Ah(t) = Ah(t-1) + dt * current / 3600;
        end
    
    end
end

