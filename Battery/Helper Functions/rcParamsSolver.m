function x = rcParamsSolver(cell, pulse)
    
    % Unpack

    initialParams = [pulse.initialParams.r0
        pulse.initialParams.r1
        pulse.initialParams.c1
        pulse.initialParams.r2
        pulse.initialParams.c2];


    initialOcv = interp1(cell.soc, cell.ocv.discharge, pulse.initialSoc, "linear", cell.ocv.discharge(end));
    pulseVoltage = pulse.voltage';

    % V = runRCModel(initialParams, pulse.time, pulse.current, initialOcv);
    
    f = @runRCModel;
    V = f(initialParams, pulse.current);
    opt = optimoptions('lsqcurvefit');
    opt.MaxFunctionEvaluations = 1000 * length(initialParams);
    opt.OptimalityTolerance = 0.01;
    opt.FunctionTolerance = 0.5;
    lb = zeros(1,length(initialParams)) + 0.01;
    ub = [0.5 0.5 30000 0.5 30000];

    x = lsqcurvefit(f, initialParams, pulse.time, pulseVoltage, lb, ub);

end

function ccv = runRCModel(params, currentData)
    
    r0 = params(1);
    r1 = params(2);
    c1 = params(3);
    r2 = params(4);
    c2 = params(5);

    vP = [0 0];
    dt = 0.1;
    initialOcv = 100;

    for i = 1:length(currentData)

        current = currentData(i);

        V0  = r0 * current;

        % RC pair 1
        dV1 = dt*(-vP(1) / (r1 * c1) + current / c1);
        vP1 = vP(1) + dV1;

        % RC pair 2
        dV2 = dt*(-vP(2) / (r2 * c2) + current / c2);
        vP2 = vP(2) + dV2;

        vP(1) = vP1;
        vP(2) = vP2;
        ccv(i) = initialOcv + V0 + vP1 + vP2;

    end

end