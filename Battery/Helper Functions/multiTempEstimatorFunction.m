function estimatedParamsOutput = multiTempEstimatorFunction(path)

    %% MULTI-TEMPERATURE : LI-ION CELL IMPEDANCE PARAMETER ESTIMATION USING NON-LINEAR MULTIVARIABLE LEAST SQUARES OPTIMIZER
    % AAKASH DESHMANE
    % 5/21/2023

    addpath 'C:\Users\deshm\OneDrive\Documents\GitHub\VehicleModel\Battery'
    load(path)
    
    %% ESTIMATION PREPROCESSING
    
    cell = batteryCell_PANA18650PF;
    soc = cell.soc;
    ocv = cell.ocv.discharge;
    
    time = meas.Time;
    [time, uniqueIndices, ~] = unique(time);
    
    voltage = meas.Voltage;
    voltage = voltage(uniqueIndices);
    
    current = -meas.Current;
    current = current(uniqueIndices);
    
    loadCurrentIndices = find(current);
    pulsePackets = getIndexPacket(loadCurrentIndices, time, current, voltage);
    numPulses = size(pulsePackets);
    numPulses = numPulses(2);
    
    %% ESTIMATION 
    
    estimatedParamsOutput = struct;
    
    % Initial parameters for the model
    initialParams.soc = 100;
    initialParams.r0 = 0.001;
    initialParams.r1 = 0.0005;
    initialParams.c1 = 0.3;
    initialParams.r2 = 0.05;
    initialParams.c2 = 4;
    
    for pulseNum = 1:numPulses
    
        % Packet pre processing
        pulse = pulsePackets(pulseNum);
        inputData = [pulse.time, pulse.current];
        measuredTime = pulse.time;
        measuredVoltage = pulse.voltage;
        measuredData = timeseries(measuredVoltage,measuredTime);
        initialOCV = measuredVoltage(1);
    
        % Initialization necessary as a placeholder for parameters!
        r0 = initialParams.r0;  % ohm
        r1 = initialParams.r1;  % ohm
        c1 = initialParams.c1;  % F
        r2 = initialParams.r2;  % ohm
        c2 = initialParams.c2;  % F
    
        % Initialize Parameters for estimation
        paramNames = ["c1", "c2", "r0", "r1", "r2"];
        paramInitialValues = [initialParams.c1 initialParams.c2 initialParams.r0 initialParams.r1 initialParams.r2];
        params = initializeParams(paramNames, paramInitialValues);
    
        % Estimation 
        estimatedParamsRaw = parameterEstimationRcModel(params, measuredData);
        estimatedParams.c1 = estimatedParamsRaw(1,1).Value;
        estimatedParams.c2 = estimatedParamsRaw(1,2).Value;
        estimatedParams.r0 = estimatedParamsRaw(1,3).Value;
        estimatedParams.r1 = estimatedParamsRaw(1,4).Value;
        estimatedParams.r2 = estimatedParamsRaw(1,5).Value;
        
        % Data unpacking and packet post processing
    
        r0 = estimatedParams.r0;  % ohm
        r1 = estimatedParams.r1;  % ohm
        c1 = estimatedParams.c1;  % F
        r2 = estimatedParams.r2;  % ohm
        c2 = estimatedParams.c2;  %F
    
        estimatedParamsOutput(pulseNum).r0 = estimatedParams.r0;
        estimatedParamsOutput(pulseNum).r1 = estimatedParams.r1;
        estimatedParamsOutput(pulseNum).c1 = estimatedParams.c1;
        estimatedParamsOutput(pulseNum).r2 = estimatedParams.r2;
        estimatedParamsOutput(pulseNum).c2 = estimatedParams.c2;
        
        % Initialize parameters for next packet
        initialParams = estimatedParams;
    
        disp(['DONE WITH PACKET NUMBER = ' num2str(pulseNum) ' OUT OF ' num2str(numPulses)])
    end
    
    %% POST PROCESSED PLOTTING
    
    [uniqueOcv, uniqueIndices, ~] = unique(ocv);
    socs = cell.soc(uniqueIndices);
    
    for pulseNum = 1:numPulses
            socVector(pulseNum) = interp1(uniqueOcv, socs, pulsePackets(pulseNum).voltage(1), "linear", 100);
            estimatedParamsOutput(pulseNum).soc = socVector(pulseNum);
    end

end

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
    
    function params = initializeParams(paramNames, paramInitialValues)
        numParams = length(paramNames);
        for i = 1:numParams
            params(i) = param.Continuous(paramNames(i), paramInitialValues(i));
            params(i).Minimum = 0;     
            params(i).Scale = 0.25;
        end
    end
