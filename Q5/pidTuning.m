% Testing Kp within the range [start,stop] with indicated step size
start = 5.04992;
stop = 6;
step = 0.0000001;

[bestKp,lastBestKp] = kpTuning(start,stop,step);

function [bestKp,lastBestKp] = kpTuning(start,stop,step)
    iters = 0;
    % Composition of transfer functions G_m(s)Y(s)
    sys = tf(0.05,[0.1,1,0,0]);
    lastBestKp = 0;
    bestKp = 0;
    maxPMargin = -10;
    for i = start:step:stop
        % Transfer function C(s) of current Kp i
        ctr = pdTransfer(i);
        % Compute transfer function of entire close loop system T(s)
        wholeTransfer = feedback(ctr*sys,1);
        % Calculate phase margin
        [Gm,Pm,Wcg,Wcp] = margin(wholeTransfer);
        % Compare current phase margin with previous maximal phase margin
        if Pm > maxPMargin && Pm ~= Inf
            lastBestKp = bestKp;
            bestKp = i;
            maxPMargin = Pm;
        end
        iters = iters + 1;
        if mod(iters,100) == 0
            fprintf("Current iter: %i     bestKp: %f      maxPMargin: %f\n",iters,bestKp,maxPMargin)
        end
    end
end

function c = pdTransfer(Kp)
    % Creating controller transfer function given parameter Kp
    c = pid(Kp,0,10*Kp);
end