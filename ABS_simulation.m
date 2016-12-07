close all
clear all

m = 450;
g = 9.81;
r = 0.32;
I = 1;
v0 = 25;
w0 = 100;
desiredSlip = 0.2; 

%normal force
FN = m*g;

%temporär Mb
Mb = 1500;

%initiell vinkelhast och hast
v(1) = v0;
w(1) = w0;

%controller
Kp = 1000;
P0 = -50;

%counter for vectors
vecPos = 1;

%step size
h = 0.01;

for t = 1:h:10
    
    %calculate slip ratio
    slipRatio(vecPos) = (v(vecPos) - w(vecPos)*r) / v(vecPos);

    %ajust slip if it exceeds its limits
    if(slipRatio(vecPos) > 1)
        slipRatio(vecPos) = 1;
    elseif(slipRatio(vecPos) < 0)
        slipRatio(vecPos) = 0;
    end
    
    %calculate the error
    slipError(vecPos) =  abs(slipRatio(vecPos)-desiredSlip);
    
    %ajust Mb
    if(slipRatio(vecPos) > desiredSlip)
        Mb = Mb - Kp*slipError(vecPos) + P0;
    elseif(slipRatio(vecPos) < desiredSlip)
        Mb = Mb + Kp*slipError(vecPos) + P0;
    end
      
    %calculate friction with pacejkas agic formula
    u = sin(1.9*atan(10*slipRatio(vecPos) - 0.97*(10*slipRatio(vecPos) - atan(10*slipRatio(vecPos)))));

    %u = 1.28*(1-exp(-23.99*slipRatio(vecPos)) - 0.52*slipRatio(vecPos));
    
    %update wheel angular velocity
    dw = (r*u*FN - Mb) / I;
    w(vecPos+1) = w(vecPos) + h*dw;
    
    %update car velocty
    dv = -u*FN/m;
    v(vecPos+1) = v(vecPos ) + dv*h;
    
    %if car velocity is zero or lower, stop simulation
    if(v(vecPos+1) < 0)
        v(vecPos+1) = 0;
        break
    end
    
    %count array
    vecPos = vecPos + 1; 
end
    
plot(v)
title('Hastighet')
xlabel('t') % x-axis label
ylabel('v') % y-axis label

figure
plot(w)
title('Vinkelhastighet')
xlabel('t') % x-axis label
ylabel('w') % y-axis label

figure
plot(slipRatio)
title('Slip Ratio')
xlabel('t') % x-axis label
ylabel('Slip Ratio') % y-axis label
