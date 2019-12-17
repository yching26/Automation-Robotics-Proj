clc;close all;
[num,~,~] = xlsread('MotorData.xlsx','Sheet2');
num = num(5:end,:);
WSS = [];
for i = 1:2:20  
    t = num(:,i);
    t = t(~isnan(t));
    w = num(:,i+1);
    w = w(~isnan(t));
    
    L = length(t)*.8;
    L = round(L);
    wss = mean(w(L:end))
    WSS = [WSS; wss];
    wc = 0.632*wss;
    [Y,I] = min(w-wc);
    L2 = length(Y);
    I2 = ceil(L2/2);
    Ic = I(I2);
    disp(num2str((i+1)*5))
    Tm = t(Ic)
    Km = wss/(12.24*(i+1)*5/100)
    disp('-----------------------------------')
    
    figure
    plot(num(:,i),num(:,i+1),'o');
    str = num2str((i+1)/2); str = [str '0% duty cycle'];
    title(str); xlabel('time [s]'); ylabel('speed [rev/s]');
    hold on;
    plot(Tm,wss,'m*',Tm,wc,'rx');
end
figure
plot(num(:,21),num(:,22),'o');
title('10% duty cycle + MASS'); xlabel('time [s]'); ylabel('speed [rev/s]');
figure
plot(num(:,23),num(:,24),'o');
title('50% duty cycle + MASS'); xlabel('time [s]'); ylabel('speed [rev/s]');
figure
plot(num(:,25),num(:,26),'o');
title('100% duty cycle + MASS'); xlabel('time [s]'); ylabel('speed [rev/s]');

    i = 21;
    t = num(:,i);
    t = t(~isnan(t));
    w = num(:,i+1);
    w = w(~isnan(t));
    
    L = length(t)*.8;
    L = round(L);
    wss = mean(w(L:end))
    wc = 0.632*wss;
    [Y,I] = min(w-wc);
    L2 = length(Y);
    I2 = ceil(L2/2);
    Ic = I(I2);
    disp('10+m')
    Tm = t(Ic)
    Km = wss/(12.24*(i+1)*5/100)
    disp('-----------------------------------')
    
    i = 23;
    t = num(:,i);
    t = t(~isnan(t));
    w = num(:,i+1);
    w = w(~isnan(t));
    
    L = length(t)*.8;
    L = round(L);
    wss = mean(w(L:end))
    wc = 0.632*wss;
    [Y,I] = min(w-wc);
    L2 = length(Y);
    I2 = ceil(L2/2);
    Ic = I(I2);
    disp('50+m')
    Tm = t(Ic)
    Km = wss/(12.24*(i+1)*5/100)
    disp('-----------------------------------')
    
    i = 25;
    t = num(:,i);
    t = t(~isnan(t));
    w = num(:,i+1);
    w = w(~isnan(t));
    
    L = length(t)*.8;
    L = round(L);
    wss = mean(w(L:end))
    wc = 0.632*wss;
    [Y,I] = min(w-wc);
    L2 = length(Y);
    I2 = ceil(L2/2);
    Ic = I(I2);
    disp('100+m')
    Tm = t(Ic)
    Km = wss/(12.24*(i+1)*5/100)
    disp('-----------------------------------')
