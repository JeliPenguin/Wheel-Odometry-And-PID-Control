
start = 5.04992;
stop = 6;
step = 0.0000001;

[bestKp,lastBestKp] = kpTuning(start,stop,step);

function [bestKp,lastBestKp] = kpTuning(start,stop,step)
    iters = 0;
    sys = tf(0.05,[0.1,1,0,0]);
    lastBestKp = 0;
    bestKp = 0;
    maxPMargin = -10;
    for i = start:step:stop
        ctr = pdTransfer(i);
        wholeTransfer = feedback(ctr*sys,1);
        [Gm,Pm,Wcg,Wcp] = margin(wholeTransfer);
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
    c = pid(Kp,0,10*Kp);
end